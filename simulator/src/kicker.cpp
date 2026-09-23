#include "kicker.h"
#include "config.h"
#include "robot.h"
#include "ball.h"
#include <algorithm>
#include <cmath>

static const float MM = 1000.0f;

void Kicker::init(Config& cfg)
{
    m_maxPower = cfg.getFloat("/robot/kicker/max_power", 8.0f);
    m_chargeTime = cfg.getFloat("/robot/kicker/charge_time", 0.2f);
    if (m_chargeTime <= 0.0f) m_chargeTime = 0.2f;
    m_heightOffset = cfg.getFloat("/robot/kicker/height_offset", 0.0f) / MM;
    m_range = cfg.getFloat("/robot/kicker/range", 40.0f) / MM;

    // The plunger sits behind/below the dribbler; reuse the dribbler's forward
    // offset as the kick origin (the kicker config has no forward key of its
    // own — see docs/tasks/dribbler-kicker.md).
    m_forwardOffset = cfg.getFloat("/robot/dribbler/forward_offset", 95.0f) / MM;

    // See the comment above requestKick for why chip needs its own angle
    // term instead of coming "for free" from the height-offset torque.
    float chipMaxAngleDeg = cfg.getFloat("/robot/kicker/chip_max_angle", 30.0f);
    m_chipMaxAngle = chipMaxAngleDeg * 3.14159265f / 180.0f;

    m_charge = 1.0f; // start fully charged
}

void Kicker::update(float dt)
{
    m_charge = std::min(1.0f, m_charge + dt / m_chargeTime);
}

void Kicker::requestKick(const Robot& robot, Ball& ball, float power)
{
    if (power <= 0.0f || !ball.body()) return;

    // The impulse actually delivered is limited by how much charge is available.
    float delivered = std::min(power, m_charge);
    if (delivered <= 0.0f) return;

    glm::vec3 robotPos = robot.position();
    float yaw = robot.orientation();
    float cosYaw = std::cos(yaw), sinYaw = std::sin(yaw);
    glm::vec3 forwardDir(cosYaw, 0.0f, -sinYaw);

    // Range check: point-distance between the ball center and the kick origin
    // (forward_offset ahead of the robot, at ball-center height). Chose the
    // simpler point check over re-deriving the dribbler's box-zone test because
    // a kick is a single radial impulse, not a continuous capture region.
    glm::vec3 kickOrigin = robotPos + forwardDir * m_forwardOffset;
    kickOrigin.y = ball.radius();
    float dist = glm::length(ball.position() - kickOrigin);
    if (dist > m_range) return; // out of range: no kick, no capacitor drain

    float impulseMag = delivered * m_maxPower;

    // Off-center impulse: the pure-Y height offset still feeds
    // btRigidBody::applyImpulse's rel_pos term, which is correct for spin
    // (residual backspin/topspin on exit is real and worth keeping) but does
    // NOT chip the ball on its own. applyImpulse splits into
    // applyCentralImpulse(impulse) [always dv = impulse/mass, independent of
    // rel_pos] plus a torque impulse from rel_pos.cross(impulse) — the
    // offset only ever changes angular velocity, never the linear velocity
    // Bullet gives the ball's center of mass. A flat plunger contacting a
    // sphere at any height has a horizontal contact normal regardless of
    // contact height, so this isn't a simulator shortcut — a real flat
    // kicker plate can't chip from height offset alone either; real RoboCup
    // chip kickers use a mechanically angled plate for exactly this reason.
    // (An earlier revision of this file assumed height offset alone would
    // "naturally" produce chip/press-down via Bullet's own dynamics — see
    // the superseded note in docs/tasks/dribbler-kicker.md — but that was
    // never actually true; verified here by rigid-body mechanics, and it
    // matches what was observed: the ball never gained any vertical
    // velocity no matter the height_offset value.)
    //
    // So: a negative (low) height_offset now also tilts the impulse itself
    // upward, approximating a hinged/angled plate whose tilt scales with how
    // far below center it contacts (0 at center, chip_max_angle at
    // height_offset == -ball_radius). Positive/neutral height_offset stays
    // flat — pressing "into the ground" from above center has nowhere to go
    // (the ball is already resting on the ground), so it's left as a
    // torque-only (spin) effect, same as before.
    float chipAngle = 0.0f;
    if (m_heightOffset < 0.0f) {
        float t = std::min(1.0f, -m_heightOffset / ball.radius());
        chipAngle = t * m_chipMaxAngle;
    }
    float cosA = std::cos(chipAngle), sinA = std::sin(chipAngle);

    btVector3 contactOffset(0.0f, m_heightOffset, 0.0f);
    // Wake the ball first: applyImpulse only changes velocity, it does not
    // activate a body Bullet has put to sleep after it settled, and a sleeping
    // body's velocity is ignored by stepSimulation.
    ball.body()->activate(true);
    ball.body()->applyImpulse(
        btVector3(forwardDir.x * impulseMag * cosA, impulseMag * sinA, forwardDir.z * impulseMag * cosA),
        contactOffset);

    m_charge -= delivered;
    if (m_charge < 0.0f) m_charge = 0.0f;
}
