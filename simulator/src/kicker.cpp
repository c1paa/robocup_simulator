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

    // Off-center impulse: the pure-Y height offset produces the chip/press-down
    // torque via Bullet's own rigid-body dynamics (a pure-Y offset is
    // rotation-invariant, so it needs no yaw rotation). Neutral height => no
    // torque, low plunger => chip, high plunger => pressed into the ground.
    btVector3 contactOffset(0.0f, m_heightOffset, 0.0f);
    // Wake the ball first: applyImpulse only changes velocity, it does not
    // activate a body Bullet has put to sleep after it settled, and a sleeping
    // body's velocity is ignored by stepSimulation.
    ball.body()->activate(true);
    ball.body()->applyImpulse(
        btVector3(forwardDir.x * impulseMag, 0.0f, forwardDir.z * impulseMag),
        contactOffset);

    m_charge -= delivered;
    if (m_charge < 0.0f) m_charge = 0.0f;
}
