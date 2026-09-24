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

    // Decision #4 in docs/tasks/dribbler-kicker.md: zero commanded speed means
    // zero capture force -- an unpowered dribbler should only hold a resting
    // ball via real chassis collision geometry (the recessed pocket), not via
    // this hand-rolled force. Without this gate, the centering/rigid-attachment
    // terms below apply purely from being in the capture zone, independent of
    // dribble_speed -- which actively pulls a ball to pocket-center and drags
    // it along through robot motion even with the roller commanded to 0.
    if (std::fabs(m_actualSpeed) < 1e-3f) {
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

    // ---- Split the slip into a "roller" axis and a "cradle" axis, and grip
    // each with its own budget, instead of clamping one combined-direction
    // force (the previous model). The roller spins about local Z, so its own
    // contact-point target only ever moves along local X (see spinWorld
    // above) — that axis is *its* job: pulling the ball in/out and imparting
    // backspin. Everything else (rigid-attachment's response to the robot
    // rotating/translating, and the centering spring) is what the concave
    // plate below/behind the ball is for: a cradle machined to the ball's own
    // radius that resists the ball being pushed off its seated position in
    // *any* horizontal direction, via a contact force perpendicular to that
    // shared spherical surface.
    //
    // Concretely: for a ball sitting centered (localZ ~ 0), the roller axis
    // is local X and the cradle axis is local Z — and a pure yaw rotation's
    // rigid-attachment target (`omega x offset`) is perpendicular to the
    // ball's own local-position vector (basic circular motion: velocity is
    // tangential to the radius), i.e. it lands on the cradle axis, not the
    // roller axis. Concretely verified empirically: assigning the *ball's
    // own local-position direction* as the "normal/cradle" axis (the first
    // attempt at this) put the turning demand in the wrong (smaller, no
    // pocket-bonus) budget and made a gentle 1 rad/s turn eject the ball,
    // a regression from the previously-tuned/documented behavior — this is
    // why the cradle axis is the one *perpendicular* to the ball's local
    // position, not parallel to it.
    //
    // Previously mixing both into one combined-direction clamp let the
    // roller's own huge along-X spin demand (`actualSpeed * r`, easily over
    // 1 m/s) eat into the same budget a turn needed to hold the ball, which
    // is what produced the "ejects, then flies out sideways on re-capture as
    // if spun very fast" symptom — see the 2026-09-24 discussion.
    glm::vec2 ballDirLocal = glm::normalize(glm::vec2(localX, localZ));
    glm::vec2 cradleAxisLocal(-ballDirLocal.y, ballDirLocal.x); // rotate 90°
    btVector3 cradleAxisWorld(
        cosYaw * cradleAxisLocal.x + sinYaw * cradleAxisLocal.y,
        0.0f,
        -sinYaw * cradleAxisLocal.x + cosYaw * cradleAxisLocal.y);

    float slipCradleMag = slipWorld.dot(cradleAxisWorld);
    btVector3 slipCradleWorld = cradleAxisWorld * slipCradleMag;
    btVector3 slipRollerWorld = slipWorld - slipCradleWorld;

    // Deadbeat-toward-target force, damped by m_responseGain (same role as
    // Robot::applyDriveForces's m_frictionResponseGain — the raw one-step
    // force is almost always far above the friction limit, so without this
    // it saturates the clamp every frame regardless of how small the actual
    // slip is).
    float ballMass = ball.mass();
    btVector3 forceCradleWorld = -m_responseGain * (ballMass / dt) * slipCradleWorld;
    btVector3 forceRollerWorld = -m_responseGain * (ballMass / dt) * slipRollerWorld;

    // Cradle budget: the plate's geometric grip, including the pocket-wall
    // bonus (a deeper pocket resists being pushed out of it more — same
    // reasoning as before, but now it only bonuses the cradle direction,
    // since that's the direction the concave walls actually resist). This is
    // the "сила, которую даёт дриблер для удержания" — exceed it (too sharp
    // a turn) and the cradle slip can't be fully cancelled, so the ball
    // drifts off its seated position and eventually leaves the capture zone,
    // i.e. ejects — exactly when the required normal (centripetal)
    // acceleration exceeds what this budget can supply.
    float maxCradleForce = m_friction * (m_normalForce + m_pocketDepth * m_pocketGripGain);
    // Roller budget: plain roller-vs-ball surface friction, no pocket bonus —
    // the concave plate's walls don't add grip in the direction the roller
    // spins, only in the direction they cradle.
    float maxRollerForce = m_friction * m_normalForce;

    float cradleMag = forceCradleWorld.length();
    if (cradleMag > maxCradleForce && cradleMag > 1e-6f) {
        forceCradleWorld *= maxCradleForce / cradleMag;
    }

    float rollerMag = forceRollerWorld.length();
    float rollerUtil = 0.0f;
    if (rollerMag > maxRollerForce && rollerMag > 1e-6f) {
        forceRollerWorld *= maxRollerForce / rollerMag;
        rollerUtil = 1.0f;
    } else if (maxRollerForce > 1e-6f) {
        rollerUtil = rollerMag / maxRollerForce;
    }

    btVector3 forceWorld = forceCradleWorld + forceRollerWorld;

    // Wake the ball so the force actually integrates (a settled, sleeping ball
    // ignores applied forces — same reason the kicker activates before its
    // impulse).
    ball.body()->activate(true);
    ball.body()->applyForce(forceWorld, contactOffsetBt);

    // Load fraction feeds next frame's motor sag — specifically the roller's
    // own utilization, since that's the roller motor's load; the cradle
    // force is pure plate geometry and doesn't touch the motor at all.
    m_loadFraction = std::min(1.0f, rollerUtil);
}
