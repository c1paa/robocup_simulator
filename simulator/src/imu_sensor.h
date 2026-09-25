#pragma once

#include "common.h"
#include <random>

class Config;
class Robot;

// A BNO055-modeled 9-axis absolute orientation sensor: three fused Euler
// angles (roll/pitch/yaw) plus gravity-removed linear acceleration and raw
// angular velocity, all in the robot's body frame, matching the real part's
// VECTOR_EULER / VECTOR_LINEARACCEL / VECTOR_GYROSCOPE outputs. See
// docs/tasks/imu-sensor.md for the physical model and every simplification.
//
// Key distinction from a naive "integrate the gyro" model: the real BNO055's
// onboard fusion anchors yaw to the magnetometer, so heading does not
// accumulate drift the way raw gyro integration would -- only small,
// bounded, per-sample noise. Modeled here by reading the robot's *true* yaw
// directly (not integrating gyro_yaw) and adding noise, not bias+drift.
// Roll/pitch are always ~0 + noise: Robot's rigid body is yaw-only
// (setAngularFactor locks out tipping), so a robot mounted flat on this
// chassis physically never has real roll/pitch to report, exactly like the
// real sensor reading true-flat.
class ImuSensor
{
public:
    void init(Config& cfg);

    // Advance the sensor's internal clock; recompute a new sample only once
    // per 1/update_rate_hz (the real part's own fusion output rate) --
    // matches LidarSensor's own-cadence-independent-of-render-tick pattern.
    void update(const Robot& robot, float dt);

    float roll()  const { return m_roll; }
    float pitch() const { return m_pitch; }
    float yaw()   const { return m_yaw; }
    glm::vec3 acceleration() const { return m_accel; }     // body frame, m/s^2, gravity removed
    glm::vec3 angularVelocity() const { return m_gyro; }    // body frame, rad/s

private:
    // ---- Config ----
    float m_updateRateHz = 100.0f;
    float m_rollPitchNoiseStd = 0.0175f; // rad (1 deg)
    float m_yawNoiseStd = 0.0436f;       // rad (2.5 deg)
    float m_accelNoiseStd = 0.03f;       // m/s^2
    float m_gyroNoiseStd = 0.00524f;     // rad/s (0.3 deg/s)
    float m_gravity = 9.81f;             // m/s^2, from /physics/gravity

    // ---- Sample-rate gating ----
    float m_period = 0.01f;
    float m_timeSinceSample = 0.0f;

    // ---- Latest fused sample ----
    float m_roll = 0.0f, m_pitch = 0.0f, m_yaw = 0.0f;
    glm::vec3 m_accel = glm::vec3(0.0f);
    glm::vec3 m_gyro = glm::vec3(0.0f);

    // Previous world-frame linear velocity, for finite-differencing true
    // coordinate acceleration (Bullet doesn't expose acceleration directly --
    // same approach used nowhere else in this codebase yet, but the only
    // practical one without hand-tracking applied forces here too).
    glm::vec3 m_prevVelocityWorld = glm::vec3(0.0f);
    bool m_hasPrevVelocity = false;

    std::mt19937 m_rng{44};

    void sample(const Robot& robot, float dtSinceLastSample);
};
