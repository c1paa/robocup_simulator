#include "dribbler.h"
#include "config.h"
#include "robot.h"
#include "ball.h"
#include <algorithm>
#include <cmath>

static const float MM = 1000.0f;
static const float RAD_PER_SEC_PER_RPM = (2.0f * glm::pi<float>()) / 60.0f;

void Dribbler::init(Config& cfg)
{
    // max_speed is configured in RPM; convert to rad/s once at load.
    m_maxSpeed = cfg.getFloat("/robot/dribbler/max_speed", 1000.0f) * RAD_PER_SEC_PER_RPM;

    m_radiusCenter = cfg.getFloat("/robot/dribbler/radius_center", 8.0f)  / MM;
    m_radiusEdge   = cfg.getFloat("/robot/dribbler/radius_edge",   12.0f) / MM;
    m_length        = cfg.getFloat("/robot/dribbler/length",        70.0f) / MM;
    m_forwardOffset = cfg.getFloat("/robot/dribbler/forward_offset", 95.0f) / MM;
    m_heightOffset  = cfg.getFloat("/robot/dribbler/height_offset",  15.0f) / MM;
    m_captureToleranceForward = cfg.getFloat("/robot/dribbler/capture_tolerance_forward", 15.0f) / MM;
    m_captureToleranceHeight  = cfg.getFloat("/robot/dribbler/capture_tolerance_height",  10.0f) / MM;

    // friction is dimensionless; normal_force is already in the codebase's
    // force unit (N — same kg·m/s^2 system Robot::applyDriveForces uses for its
    // normal = mass * gravity / wheelCount). No further conversion.
    m_friction = cfg.getFloat("/robot/dribbler/friction", 1.2f);
    m_normalForce = cfg.getFloat("/robot/dribbler/normal_force", 0.6f);

    m_motorTimeConstant = cfg.getFloat("/robot/dribbler/motor_time_constant", 0.03f);
    m_loadSagGain = cfg.getFloat("/robot/dribbler/load_sag_gain", 0.4f);
    m_responseGain = cfg.getFloat("/robot/dribbler/response_gain", 0.4f);
}

void Dribbler::setTargetSpeed(float speed)
{
    m_targetSpeed = std::clamp(speed, -1.0f, 1.0f);
}

float Dribbler::rpm() const
{
    return m_actualSpeed / RAD_PER_SEC_PER_RPM;
}

void Dribbler::update(const Robot& robot, Ball& ball, float dt)
{
    if (!ball.body() || dt <= 0.0f) return;

    // ---- First-order motor lag toward the commanded speed, with the previous
    // frame's load sag reducing the effective target when a ball is loading the
    // roller (see docs/tasks/dribbler-kicker.md). Same lag pattern as
    // Robot::applyDriveForces's per-wheel m_wheelSpeed.
    float sagFactor = 1.0f - m_loadSagGain * m_loadFraction;
    float target = m_targetSpeed * m_maxSpeed * sagFactor;
    float alpha = dt / (m_motorTimeConstant + dt);
    m_actualSpeed = util::lerp(m_actualSpeed, target, alpha);

    // ---- Capture-zone test: ball position in the robot's local frame. ----
    glm::vec3 ballPos = ball.position();
    glm::vec3 robotPos = robot.position();
    float yaw = robot.orientation();
    float cosYaw = std::cos(yaw), sinYaw = std::sin(yaw);

    float dx = ballPos.x - robotPos.x;
    float dz = ballPos.z - robotPos.z;
    // world -> local (transpose of the yaw rotation; same derivation as
    // Robot::applyDriveForces's linX/linZ).
    float localX = cosYaw * dx - sinYaw * dz;
    float localZ = sinYaw * dx + cosYaw * dz;
    float localY = ballPos.y; // ball center height above ground

    float halfLen = m_length * 0.5f;
    bool inZone = std::fabs(localX - m_forwardOffset) <= m_captureToleranceForward
               && std::fabs(localY - m_heightOffset)  <= m_captureToleranceHeight
               && std::fabs(localZ)                   <= halfLen;

    if (!inZone) {
        m_loadFraction = 0.0f;
        return;
    }

    // ---- Effective roller radius at this lateral contact (taper profile). ----
    float r = util::lerp(m_radiusCenter, m_radiusEdge, std::fabs(localZ) / halfLen);

    // Roller surface velocity at the contact point (robot local frame): purely
    // tangential, along local -X for positive speed (capture direction).
    glm::vec3 rollerSurfLocal(-m_actualSpeed * r, 0.0f, 0.0f);

    // Contact point on the ball's surface: the point nearest the roller axis.
    // This is where the force is applied so Bullet derives the backspin torque
    // from the off-center application point automatically (no separate torque).
    // The roller axis runs along local Z, so its nearest point to the ball is
    // at the *ball's own* localZ, not a fixed Z=0 — using 0 here would point
    // "toRoller" partly along Z whenever the ball isn't perfectly centered,
    // adding a spurious roll/yaw component to the contact offset (and thus to
    // the torque Bullet derives from it) on top of the intended backspin.
    float ballRadius = ball.radius();
    glm::vec3 rollerAxisLocal(m_forwardOffset, m_heightOffset, localZ);
    glm::vec3 ballCenterLocal(localX, localY, localZ);
    glm::vec3 toRoller = rollerAxisLocal - ballCenterLocal;
    float dist = glm::length(toRoller);
    if (dist < 1e-4f) dist = 1e-4f;
    glm::vec3 contactOffsetLocal = (toRoller / dist) * ballRadius;

    // Ball surface velocity at that contact point: linear velocity plus
    // omega x contactOffset, computed in world then rotated into local.
    glm::vec3 ballLinWorld(
        ball.body()->getLinearVelocity().x(),
        ball.body()->getLinearVelocity().y(),
        ball.body()->getLinearVelocity().z());
    glm::vec3 ballAngWorld(
        ball.body()->getAngularVelocity().x(),
        ball.body()->getAngularVelocity().y(),
        ball.body()->getAngularVelocity().z());
    // contact offset local -> world
    glm::vec3 contactOffsetWorld(
        cosYaw * contactOffsetLocal.x + sinYaw * contactOffsetLocal.z,
        contactOffsetLocal.y,
        -sinYaw * contactOffsetLocal.x + cosYaw * contactOffsetLocal.z);
    glm::vec3 surfWorld = ballLinWorld + glm::cross(ballAngWorld, contactOffsetWorld);
    // world -> local
    glm::vec3 surfLocal(
        cosYaw * surfWorld.x - sinYaw * surfWorld.z,
        surfWorld.y,
        sinYaw * surfWorld.x + cosYaw * surfWorld.z);

    // Relative slip velocity (ball surface minus roller surface), projected
    // onto the local X/Y plane. The roller does not constrain lateral (Z) slip
    // — that's what lets the ball self-center along the taper rather than being
    // rigidly pinned in Z.
    glm::vec3 slipLocal = surfLocal - rollerSurfLocal;
    slipLocal.z = 0.0f;

    // Friction opposes the slip, so the deadbeat force is -mass * slip / dt
    // (equivalently mass * (rollerSurf - ballSurf) / dt): it drives the ball's
    // surface velocity *toward* the roller's. Same "target - actual" convention
    // as Robot::applyDriveForces, where the roller surface is the "target" the
    // captured ball should match.
    //
    // m_responseGain scales this down *before* the Coulomb clamp below, same
    // role as Robot::applyDriveForces's m_frictionResponseGain: the raw
    // one-step deadbeat force is almost always far above the friction limit
    // for realistic slip speeds, so without this it saturates at the clamp
    // every single frame regardless of how close the ball already is to the
    // roller's target surface velocity — a bang-bang controller that was
    // measured to pump energy into the ball's spin/position over ~1s of
    // continuous capture until it errupted into a runaway lateral ejection
    // with no turn commanded at all (docs/tasks/dribbler-kicker.md's own
    // "don't script ejection" scenario, but happening spuriously instead of
    // from an actual turn). Scaling the target down first means the clamp
    // only bites under real, sustained slip (e.g. actually pulling a ball in
    // from outside the zone), not on every frame of an already-captured ball.
    float ballMass = ball.mass();
    glm::vec3 forceLocal = -m_responseGain * (ballMass / dt) * slipLocal;
    float mag = glm::length(forceLocal);
    float maxMag = m_friction * m_normalForce;
    float appliedMag = mag;
    if (mag > maxMag && mag > 1e-6f) {
        forceLocal *= maxMag / mag;
        appliedMag = maxMag;
    }

    // local -> world force
    glm::vec3 forceWorld(
        cosYaw * forceLocal.x + sinYaw * forceLocal.z,
        forceLocal.y,
        -sinYaw * forceLocal.x + cosYaw * forceLocal.z);

    // Wake the ball so the force actually integrates (a settled, sleeping ball
    // ignores applied forces — same reason the kicker activates before its
    // impulse).
    ball.body()->activate(true);
    ball.body()->applyForce(
        btVector3(forceWorld.x, forceWorld.y, forceWorld.z),
        btVector3(contactOffsetWorld.x, contactOffsetWorld.y, contactOffsetWorld.z));

    // Load fraction = applied force relative to the Coulomb limit; feeds the
    // motor sag on the next frame.
    m_loadFraction = (maxMag > 1e-6f) ? std::min(1.0f, appliedMag / maxMag) : 0.0f;
}
