#pragma once

#include "common.h"
#include <mutex>
#include <condition_variable>
#include <cstdint>
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

    // Pending command (written by gRPC handler, consumed by sim thread)
    bool hasCommand = false;
    float leftWheel = 0.0f;
    float rightWheel = 0.0f;
    float kickPower = 0.0f;
    float dribbleSpeed = 0.0f;
};
