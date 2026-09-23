#include "lidar_sensor.h"
#include "config.h"
#include <algorithm>
#include <cmath>

static const float MM = 1000.0f;

void LidarSensor::init(Config& cfg, btDiscreteDynamicsWorld* world)
{
    m_world = world;

    m_height    = cfg.getFloat("/robot/lidar/height", 170.0f) / MM;
    m_minRange  = cfg.getFloat("/robot/lidar/min_range", 20.0f) / MM;
    m_maxRange  = cfg.getFloat("/robot/lidar/max_range", 12000.0f) / MM;
    m_pointsPerScan = cfg.getInt("/robot/lidar/points_per_scan", 450);
    m_scanFrequency = cfg.getFloat("/robot/lidar/scan_frequency", 10.0f);
    m_rangeNoiseStd = cfg.getFloat("/robot/lidar/range_noise_std", 15.0f) / MM;
    m_dropoutProbability = cfg.getFloat("/robot/lidar/dropout_probability", 0.01f);

    if (m_scanFrequency <= 0.0f) m_scanFrequency = 10.0f;
    m_scanPeriod = 1.0f / m_scanFrequency;
    m_timeSinceScan = m_scanPeriod; // first update performs a scan immediately

    m_latestScan.clear();
    m_latestScan.reserve((size_t)m_pointsPerScan);
}

void LidarSensor::update(const glm::vec3& robotPos, float robotYaw, float dt)
{
    if (!m_world) return;
    if (m_pointsPerScan <= 0) return;

    m_timeSinceScan += dt;
    if (m_timeSinceScan < m_scanPeriod) return;
    m_timeSinceScan -= m_scanPeriod;

    std::vector<LidarPoint> scan;
    scan.reserve((size_t)m_pointsPerScan);

    // Rays are horizontal (Y is world up), launched from the mount height.
    btVector3 from(robotPos.x, m_height, robotPos.z);

    // Angle 0 = robot forward (+X body frame). Positive body angle rotates +X
    // toward -Z (same sign as yaw), so a sample at body angle theta points in
    // world direction (cos(yaw+theta), 0, -sin(yaw+theta)).
    std::normal_distribution<float> noise(0.0f, m_rangeNoiseStd);
    std::uniform_real_distribution<float> dropout(0.0f, 1.0f);

    for (int i = 0; i < m_pointsPerScan; i++) {
        float theta = (2.0f * glm::pi<float>()) * (float)i / (float)m_pointsPerScan;
        float a = robotYaw + theta;
        btVector3 dir(std::cos(a), 0.0f, -std::sin(a));
        btVector3 to = from + dir * m_maxRange;

        btCollisionWorld::ClosestRayResultCallback cb(from, to);
        m_world->rayTest(from, to, cb);
        if (!cb.hasHit()) continue;

        btVector3 delta = cb.m_hitPointWorld - from;
        float d = delta.length();
        if (d < m_minRange) continue;

        d += noise(m_rng);
        if (d < m_minRange || d > m_maxRange) continue;

        if (dropout(m_rng) < m_dropoutProbability) continue;

        LidarPoint p;
        p.angle = theta;
        p.distance = d;
        // Synthetic confidence: near = strong, far = weak.
        p.intensity = std::clamp(1.0f - d / m_maxRange, 0.0f, 1.0f);
        scan.push_back(p);
    }

    m_latestScan = std::move(scan);
}
