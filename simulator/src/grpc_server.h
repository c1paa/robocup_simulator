#pragma once

#include "shared_state.h"
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>

namespace grpc { class Server; }

class Robot;
class Ball;
class Camera;
class LidarSensor;
class Dribbler;
class Kicker;
class ImuSensor;
class SimulatorServiceImpl;

// Bridges N robots (registered via registerRobot, one call per robot_id) to
// the gRPC SendCommand/SensorStream RPCs. Each robot gets its own SharedState
// (own pending command, own publish cadence -- a slower robot's camera
// shouldn't throttle a faster one's stream) but they all share one Ball
// ground truth and one listening port, multiplexed by RobotCommand.robot_id /
// SensorRequest.robot_id.
//
// registerRobot/setBall may now also be called *after* start() -- via
// setAddRobotHandler + the AddRobot RPC, see below -- so m_states (unlike
// m_bindings, which stays main-thread-only) is guarded by m_statesMutex:
// registerRobot takes it exclusively to insert, SimulatorServiceImpl's RPC
// handler threads and update()'s own per-robot loop take it shared to look
// up. See docs/tasks/multi-robot.md's "Dynamic robot creation" section.
class GrpcServer
{
public:
    GrpcServer();
    ~GrpcServer();

    void start();
    void stop();
    void update(float dt);

    // Drains queued AddRobot requests. Called every frame by App::run
    // regardless of pause state (a paused sim must still answer AddRobot
    // instead of leaving the RPC handler blocked), on the main/sim thread.
    void processPending();

    // True once a client called the Shutdown RPC; App::run exits its loop.
    bool shutdownRequested() const { return m_shutdownRequested.load(); }

    void setBall(Ball* ball) { m_ball = ball; }

    // Registers one robot's subsystems under `id` (matches RobotCommand/
    // SensorRequest.robot_id). Any pointer may be null if that robot has no
    // such subsystem.
    void registerRobot(int id, Robot* robot, Camera* camera, LidarSensor* lidar,
                        Dribbler* dribbler, Kicker* kicker, ImuSensor* imu);

    // Lets the AddRobot RPC actually construct robots at runtime: App sets
    // this once (to a lambda wrapping App::addRobot) so update() can call it
    // on the main/sim thread when a PendingAddRobot shows up in
    // m_addRobotQueue -- see that struct's own comment (shared_state.h) for
    // why the construction itself can't happen on the gRPC handler thread
    // that received the request. Returns the new robot's id, or a negative
    // value on failure (e.g. bad config path).
    using AddRobotFn = std::function<int(const std::string& configPath,
                                          float spawnX, float spawnZ, float spawnYawDeg)>;
    void setAddRobotHandler(AddRobotFn fn) { m_addRobotFn = std::move(fn); }

    // Producer side used by SimulatorServiceImpl's constructor -- exposes the
    // same queue drained in update() below.
    AddRobotQueue& addRobotQueue() { return m_addRobotQueue; }

    // Guards m_states (structural changes only -- lookups can run
    // concurrently, insertion cannot run concurrently with anything else).
    // SimulatorServiceImpl takes this shared to look up a robot_id; only
    // registerRobot() takes it exclusively.
    std::shared_mutex& statesMutex() { return m_statesMutex; }

private:
    struct RobotBinding {
        Robot* robot = nullptr;
        Camera* camera = nullptr;
        LidarSensor* lidar = nullptr;
        Dribbler* dribbler = nullptr;
        Kicker* kicker = nullptr;
        ImuSensor* imu = nullptr;
    };

    std::map<int, RobotBinding> m_bindings; // main/sim-thread-only, no lock needed
    std::map<int, SharedState> m_states;    // one publish/command buffer per robot_id -- see m_statesMutex
    std::shared_mutex m_statesMutex;
    Ball* m_ball = nullptr;

    AddRobotFn m_addRobotFn;
    AddRobotQueue m_addRobotQueue;
    std::atomic<bool> m_shutdownRequested{false};

    std::thread m_thread;
    std::unique_ptr<grpc::Server> m_server;
    std::unique_ptr<SimulatorServiceImpl> m_service;
};
