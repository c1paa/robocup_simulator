#pragma once

#include "common.h"
#include <btBulletDynamicsCommon.h>
#include <random>
#include <vector>

class Config;

// A single LiDAR return, in the robot body frame. angle = 0 is forward (+X)
// and increases the same way yaw does (positive yaw rotates +X toward -Z,
// per AGENTS.md) — which is the robot's LEFT, confirmed empirically via the
// manual-drive keybinding (App::handleKeyboardInput).
struct LidarPoint
{
    float angle = 0.0f;     // radians, body frame, 0 = forward (+X)
    float distance = 0.0f;  // metres
    float intensity = 0.0f; // 0.0-1.0, synthetic confidence
};

// LD06-like 2D scanning lidar: a 360-degree raycast sweep against the Bullet
// world, matched to the real sensor's range / angular density / scan rate /
// noise / dropout behaviour. Internal scan state updates at scan_frequency,
// independent of the render/physics tick; consumers read the latest completed
// scan.
class LidarSensor
{
public:
    void init(Config& cfg, btDiscreteDynamicsWorld* world);

    // Advance the internal scan clock; perform a new sweep when the configured
    // period has elapsed. robotPos is the robot's world position (body origin),
    // robotYaw its world yaw.
    void update(const glm::vec3& robotPos, float robotYaw, float dt);

    const std::vector<LidarPoint>& latestScan() const { return m_latestScan; }

    // Height of the sensor above the ground (metres).
    float height() const { return m_height; }

private:
    btDiscreteDynamicsWorld* m_world = nullptr;

    // ---- Config (converted to metres once at load time) ----
    float m_height = 0.17f;          // mount height above ground
    float m_minRange = 0.02f;
    float m_maxRange = 12.0f;
    int   m_pointsPerScan = 450;
    float m_scanFrequency = 10.0f;
    float m_rangeNoiseStd = 0.015f;
    float m_dropoutProbability = 0.01f;

    // ---- Scan state ----
    float m_scanPeriod = 0.1f;
    float m_timeSinceScan = 0.0f;
    std::vector<LidarPoint> m_latestScan;

    std::mt19937 m_rng{43};
};
