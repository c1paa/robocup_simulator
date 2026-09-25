#include "imu_sensor.h"
#include "config.h"
#include "robot.h"
#include <cmath>

static const float DEG2RAD = 3.14159265f / 180.0f;

void ImuSensor::init(Config& cfg)
{
    m_updateRateHz = cfg.getFloat("/robot/imu/update_rate_hz", 100.0f);
    if (m_updateRateHz <= 0.0f) m_updateRateHz = 100.0f;
    m_period = 1.0f / m_updateRateHz;

    m_rollPitchNoiseStd = cfg.getFloat("/robot/imu/roll_pitch_noise_std_deg", 1.0f) * DEG2RAD;
    m_yawNoiseStd       = cfg.getFloat("/robot/imu/yaw_noise_std_deg", 2.5f) * DEG2RAD;
    m_accelNoiseStd     = cfg.getFloat("/robot/imu/accel_noise_std", 0.03f);
    m_gyroNoiseStd      = cfg.getFloat("/robot/imu/gyro_noise_std_deg", 0.3f) * DEG2RAD;

    // Same /physics/gravity key Robot/Physics read, in mm/s^2 (project.json
    // convention) -- converted to m/s^2 once here, same MM=1000 pattern as
    // everywhere else (AGENTS.md).
    m_gravity = cfg.getFloat("/physics/gravity", 9810.0f) / 1000.0f;

    m_timeSinceSample = 0.0f;
    m_hasPrevVelocity = false;
}

void ImuSensor::update(const Robot& robot, float dt)
{
    m_timeSinceSample += dt;
    if (m_timeSinceSample < m_period) return;

    float elapsed = m_timeSinceSample;
    m_timeSinceSample = 0.0f;
    sample(robot, elapsed);
}

void ImuSensor::sample(const Robot& robot, float dtSinceLastSample)
{
    std::normal_distribution<float> rollPitchNoise(0.0f, m_rollPitchNoiseStd);
    std::normal_distribution<float> yawNoise(0.0f, m_yawNoiseStd);
    std::normal_distribution<float> accelNoise(0.0f, m_accelNoiseStd);
    std::normal_distribution<float> gyroNoise(0.0f, m_gyroNoiseStd);

    // Orientation: fused absolute (magnetometer-anchored) angles, not
    // integrated from the gyro -- so no accumulating drift, just bounded
    // per-sample noise. Robot's chassis is yaw-only (see Robot's
    // setAngularFactor), so true roll/pitch are always exactly 0.
    m_roll  = rollPitchNoise(m_rng);
    m_pitch = rollPitchNoise(m_rng);
    m_yaw   = robot.orientation() + yawNoise(m_rng);

    // Angular velocity: true yaw rate is exact (read straight from Bullet,
    // not integrated), roll/pitch rates are always exactly 0 for the same
    // reason as above. Real sensor noise only.
    m_gyro = glm::vec3(0.0f, robot.angularVelocity(), 0.0f) +
             glm::vec3(gyroNoise(m_rng), gyroNoise(m_rng), gyroNoise(m_rng));

    // Linear acceleration: reports the real BNO055's VECTOR_LINEARACCEL
    // register (gravity already removed by the onboard fusion), not raw
    // VECTOR_ACCELEROMETER (which would read +g on the up axis at rest --
    // see docs/tasks/imu-sensor.md for why VECTOR_LINEARACCEL is the more
    // useful one to model and the raw register isn't exposed at all).
    // Computed as true coordinate acceleration, finite-differenced from the
    // robot's own world-frame velocity between samples -- Bullet doesn't
    // expose acceleration directly, and this is the only practical way to
    // get it without duplicating Robot's own force accounting here.
    glm::vec3 velWorld = robot.linearVelocityWorld();
    glm::vec3 accelWorld(0.0f);
    if (m_hasPrevVelocity && dtSinceLastSample > 1.0e-6f) {
        accelWorld = (velWorld - m_prevVelocityWorld) / dtSinceLastSample;
    }
    m_prevVelocityWorld = velWorld;
    m_hasPrevVelocity = true;

    // World -> body frame, same rotation Robot::applyDriveForces uses for
    // linX/linZ (AGENTS.md's yaw convention) -- not re-derived by hand.
    float yaw = robot.orientation();
    float cosYaw = std::cos(yaw), sinYaw = std::sin(yaw);
    float accelBodyX = cosYaw * accelWorld.x - sinYaw * accelWorld.z;
    float accelBodyZ = sinYaw * accelWorld.x + cosYaw * accelWorld.z;
    // Vertical: the robot never leaves the ground (no jump/tip model), so
    // true world-Y coordinate acceleration is always 0 -- gravity-removed
    // linear acceleration on the up axis is therefore always ~0, exactly
    // matching the real sensor's VECTOR_LINEARACCEL (which is also ~0 at
    // rest on a level, non-accelerating surface, unlike raw
    // VECTOR_ACCELEROMETER which would read +g).
    glm::vec3 accelBody(accelBodyX, 0.0f, accelBodyZ);

    m_accel = accelBody + glm::vec3(accelNoise(m_rng), accelNoise(m_rng), accelNoise(m_rng));
}
