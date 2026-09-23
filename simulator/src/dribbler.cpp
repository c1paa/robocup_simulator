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
    // forward_offset is chosen so the captured ball's *center* settles this
    // far forward, which puts its near surface (forward_offset - ballRadius)
    // pocket_depth mm inside the chassis's nominal circle (radius = diameter
    // / 2) — a real dribbler ball sits partly recessed into the robot, not
    // flush against it. Default: chassisRadius(90) - pocket_depth(15) +
    // ballRadius(21.5) = 96.5mm. This is a manually-derived constant, not
    // computed from /robot/diameter or /physics/ball/radius at load time
    // (same style as lip_forward_offset below) — if you change the chassis
    // diameter, ball radius, or pocket_depth, recompute this by hand.
    m_forwardOffset = cfg.getFloat("/robot/dribbler/forward_offset", 96.5f) / MM;
    m_heightOffset  = cfg.getFloat("/robot/dribbler/height_offset",  15.0f) / MM;
    m_captureToleranceForward = cfg.getFloat("/robot/dribbler/capture_tolerance_forward", 15.0f) / MM;
    m_captureToleranceHeight  = cfg.getFloat("/robot/dribbler/capture_tolerance_height",  10.0f) / MM;
    m_pocketDepth = cfg.getFloat("/robot/dribbler/pocket_depth", 15.0f) / MM;
    m_pocketGripGain = cfg.getFloat("/robot/dribbler/pocket_grip_gain", 10.0f);

    // friction is dimensionless; normal_force is already in the codebase's
    // force unit (N — same kg·m/s^2 system Robot::applyDriveForces uses for its
    // normal = mass * gravity / wheelCount). No further conversion.
    m_friction = cfg.getFloat("/robot/dribbler/friction", 1.5f);
    m_normalForce = cfg.getFloat("/robot/dribbler/normal_force", 1.0f);

    m_motorTimeConstant = cfg.getFloat("/robot/dribbler/motor_time_constant", 0.03f);
    m_loadSagGain = cfg.getFloat("/robot/dribbler/load_sag_gain", 0.4f);
    m_responseGain = cfg.getFloat("/robot/dribbler/response_gain", 0.4f);
    m_centeringTimeConstant = cfg.getFloat("/robot/dribbler/centering_time_constant", 0.03f);
    if (m_centeringTimeConstant <= 0.0f) m_centeringTimeConstant = 0.15f;
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

    // contact offset local -> world
    glm::vec3 contactOffsetWorld(
        cosYaw * contactOffsetLocal.x + sinYaw * contactOffsetLocal.z,
        contactOffsetLocal.y,
        -sinYaw * contactOffsetLocal.x + cosYaw * contactOffsetLocal.z);
    btVector3 contactOffsetBt(contactOffsetWorld.x, contactOffsetWorld.y, contactOffsetWorld.z);

    // ---- Target contact-point velocity: "rigidly attached to the rotating
    // pocket, centered, plus the roller's own spin". Three parts, all in
    // world frame:
    //
    // 1) Rigid-attachment: the velocity a point fixed to the robot at this
    //    offset would have, V_robot + omega_robot x offset. This is what
    //    makes turning cost something — holding the ball through a turn
    //    means matching a centripetal/tangential demand that grows with
    //    turn rate, so a fast enough turn saturates the Coulomb clamp below
    //    and the ball falls behind (ejects) instead of being glued on
    //    unconditionally. Same mechanism covers a hard reverse: a sudden
    //    change in V_robot is a sudden change in this target that the
    //    clamped force may not be able to track.
    btVector3 robotLinWorld(robot.linearVelocityWorld().x, robot.linearVelocityWorld().y,
                             robot.linearVelocityWorld().z);
    btVector3 omegaWorld(0.0f, robot.angularVelocity(), 0.0f);
    btVector3 rigidVelWorld = robotLinWorld + omegaWorld.cross(contactOffsetBt);

    // 2) Centering: a spring-like pull, computed in the robot's local frame,
    //    toward the middle of the pocket (forward_offset, lateral center 0).
    //    This is what makes a ball that crosses into the capture zone snap
    //    to the center of the pocket instead of just sitting wherever it
    //    entered — entering the zone at all is what matters, not where in
    //    it. Deliberately uses m_centeringTimeConstant here, *not* dt: a
    //    "close this position error within one physics substep" target
    //    (posErr/dt, dt ~4ms) demands target velocities in the m/s range for
    //    even a few mm of error, which saturates the Coulomb clamp below on
    //    every frame regardless of how small the error is — leaving no
    //    budget left over to also satisfy the rigid-attachment (turning)
    //    demand, so the ball ejected at even a gentle 1 rad/s turn in
    //    testing. m_centeringTimeConstant caps the demanded velocity to a
    //    physically reasonable "pocket pulls it back over ~0.1-0.2s" rate
    //    instead, so the clamp isn't permanently maxed out just holding
    //    still.
    glm::vec3 posErrLocal(m_forwardOffset - localX, 0.0f, 0.0f - localZ);
    glm::vec3 centeringLocal = posErrLocal / m_centeringTimeConstant;
    btVector3 centeringWorld(
        cosYaw * centeringLocal.x + sinYaw * centeringLocal.z,
        0.0f,
        -sinYaw * centeringLocal.x + cosYaw * centeringLocal.z);

    // 3) Spin: roller surface target, local -X for positive speed (capture
    //    direction pulls the contact point toward the robot).
    btVector3 spinWorld(cosYaw * (-m_actualSpeed * r), 0.0f, -sinYaw * (-m_actualSpeed * r));

    btVector3 targetVelWorld = rigidVelWorld + centeringWorld + spinWorld;

    // ---- Ball's actual surface velocity at the contact point. ----
    btVector3 ballLinWorld = ball.body()->getLinearVelocity();
    btVector3 ballAngWorld = ball.body()->getAngularVelocity();
    btVector3 surfVelWorld = ballLinWorld + ballAngWorld.cross(contactOffsetBt);

    // Slip = actual - target. Vertical (Y) is excluded: gravity/ground
    // contact already own the ball's height, the dribbler only grips the
    // horizontal plane (position + rotation + spin), so it shouldn't fight
    // small vertical bounce.
    btVector3 slipWorld = surfVelWorld - targetVelWorld;
    slipWorld.setY(0.0f);

    // Deadbeat-toward-target force, damped by m_responseGain (same role as
    // Robot::applyDriveForces's m_frictionResponseGain — the raw one-step
    // force is almost always far above the friction limit, so without this
    // it saturates the clamp every frame regardless of how small the actual
    // slip is), then clamped to the Coulomb limit: this single clamp is the
    // "прижимная сила" (pressing/grip force) budget shared by centering,
    // co-rotating through a turn, and spin — there is no separate lateral
    // allowance, so a strong enough combination of any of those (fast turn,
    // hard reverse, big centering error) can exceed it and eject the ball,
    // which keeps whatever spin it had at that instant (nothing here zeroes
    // the ball's velocity/angular velocity on zone-exit, only the force
    // stops being applied).
    // The pocket the ball sits in isn't a full circle (see docs) — its
    // concave walls geometrically resist lateral escape a little on top of
    // whatever force the roller itself provides, modeled as a small bonus to
    // the effective normal force (not a separate allowance outside the
    // shared clamp — see the "one Coulomb clamp" note in
    // docs/tasks/dribbler-kicker.md, still true here).
    float effectiveNormalForce = m_normalForce + m_pocketDepth * m_pocketGripGain;

    float ballMass = ball.mass();
    btVector3 forceWorld = -m_responseGain * (ballMass / dt) * slipWorld;
    float mag = forceWorld.length();
    float maxMag = m_friction * effectiveNormalForce;
    float appliedMag = mag;
    if (mag > maxMag && mag > 1e-6f) {
        forceWorld *= maxMag / mag;
        appliedMag = maxMag;
    }

    // Wake the ball so the force actually integrates (a settled, sleeping ball
    // ignores applied forces — same reason the kicker activates before its
    // impulse).
    ball.body()->activate(true);
    ball.body()->applyForce(forceWorld, contactOffsetBt);

    // Load fraction = applied force relative to the Coulomb limit; feeds the
    // motor sag on the next frame.
    m_loadFraction = (maxMag > 1e-6f) ? std::min(1.0f, appliedMag / maxMag) : 0.0f;
}
