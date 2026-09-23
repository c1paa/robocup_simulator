#pragma once

#include "common.h"

class Config;
class Robot;
class Ball;

// Kicker solenoid: delivers an impulse to the ball, limited by a capacitor
// charge that is actually simulated over time (not just trusted from the
// client). kick_power (0..1) is a *request*; the delivered impulse is
// min(power, charge) * max_power, and firing drains the capacitor, which then
// recharges toward full over charge_time seconds.
class Kicker
{
public:
    void init(Config& cfg);

    // Recharge the capacitor toward full each frame. Call once per sim tick.
    void update(float dt);

    // Request a kick at `power` in [0, 1]. Fires only if the ball is within
    // range of the kicker point; the delivered impulse is capped by the current
    // charge and the charge is drained by whatever is delivered.
    void requestKick(const Robot& robot, Ball& ball, float power);

    // Capacitor charge level, 0.0 (empty) to 1.0 (full). Published as
    // SensorData.capacitor_charge so clients can poll before firing.
    float charge() const { return m_charge; }

private:
    // ---- Config (converted to SI once at load) ----
    float m_maxPower = 8.0f;        // N*s impulse at full charge, full power
    float m_chargeTime = 0.2f;      // s, empty -> full
    float m_heightOffset = 0.0f;    // m, plunger height relative to ball center
    float m_range = 0.04f;          // m, max ball-to-kicker distance
    float m_forwardOffset = 0.095f; // m, kick origin forward of chassis center
    float m_chipMaxAngle = 0.0f;    // rad, launch angle at height_offset == -ball_radius
                                     // (see the comment above requestKick in kicker.cpp for why
                                     // this exists as its own term instead of relying on torque)

    float m_charge = 1.0f; // 0..1
};
