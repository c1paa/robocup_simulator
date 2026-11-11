#pragma once

#include "common.h"
#include "lidar_sensor.h"
#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

// Mutex-guarded bridge between the simulator thread (App::update) and the
// gRPC handler threads. Minimal on purpose: latest camera frame + robot pose
// published by the sim thread, and the pending drive command written by the
// gRPC SendCommand handler and consumed by the sim thread.
struct SharedState
{
    std::mutex mtx;
    std::condition_variable cv;
    bool shutdown = false;

    // Non-empty for robots claimed via Join(): SendCommand/SensorStream must
    // present it. Empty (AddRobot/CLI robots) = unrestricted.
    std::string token;

    // Sensor data (published by the sim thread each frame)
    std::vector<uint8_t> image; // RGB, top-down
    int width = 0;
    int height = 0;
    glm::vec3 position{0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float velocity = 0.0f;
    float angularVelocity = 0.0f;
    double timestamp = 0.0;
    uint64_t frameId = 0;

    // Ball ground-truth position (meters)
    glm::vec3 ballPosition{0.0f, 0.0f, 0.0f};

    // Dribbler motor's actual (lagged, load-sagged) speed (RPM, signed like
    // dribble_speed) and kicker electrical state.
    float dribblerRpm = 0.0f;
    float capacitorCharge = 0.0f;   // 0..1, capacitorVoltage / chargeVoltage
    float busVoltage = 16.0f;       // V
    float capacitorVoltage = 0.0f;  // V

    // IMU (BNO055-modeled), body frame -- see imu_sensor.h.
    float imuRoll = 0.0f, imuPitch = 0.0f, imuYaw = 0.0f;
    glm::vec3 imuAccel{0.0f, 0.0f, 0.0f};
    glm::vec3 imuGyro{0.0f, 0.0f, 0.0f};

    // Dead-reckoning odometry (drifts from ground truth under slip)
    float odomX = 0.0f;
    float odomZ = 0.0f;
    float odomYaw = 0.0f;

    // Latest completed lidar scan (robot body frame)
    std::vector<LidarPoint> lidarPoints;

    // Pending command (written by gRPC handler, consumed by sim thread)
    bool hasCommand = false;
    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
    float dribbleSpeed = 0.0f;
    bool capacitorChargeOpen = false;
    bool kickerOpen = false;
};

// One runtime "add a robot" request, pushed by a SimulatorServiceImpl::
// AddRobot gRPC handler thread and drained once per frame by GrpcServer::
// update() on the sim/main thread -- robot construction touches Bullet and
// GL (camera FBOs), both main-thread-only, so it can't happen directly on
// the gRPC handler thread that received the request. The handler blocks on
// this object's own mtx/cv until the main thread fills in the result and
// sets done, so AddRobot's unary response is only sent once the robot
// actually exists and can be reached by SendCommand/SensorStream. See
// docs/tasks/multi-robot.md's "Dynamic robot creation" section.
struct PendingAddRobot
{
    std::string configPath;
    float spawnX = 0.0f, spawnZ = 0.0f, spawnYawDeg = 0.0f;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    bool success = false;
    std::string message;
    int robotId = -1;
};

// Queue of pending AddRobot requests, shared between SimulatorServiceImpl
// (producer, one or more gRPC handler threads) and GrpcServer (consumer, the
// sim thread). Guarded by its own mutex, separate from each PendingAddRobot's
// own mtx (which only guards that one request's done/result fields).
struct AddRobotQueue
{
    std::mutex mtx;
    std::deque<std::shared_ptr<PendingAddRobot>> pending;
};
