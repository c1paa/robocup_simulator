#include "grpc_server.h"
#include "simulator_service.h"
#include "config.h"
#include "robot.h"
#include "ball.h"
#include "camera.h"
#include "lidar_sensor.h"
#include "dribbler.h"
#include "kicker.h"
#include "imu_sensor.h"

#include <grpcpp/grpcpp.h>
#include <iostream>
#include <chrono>

GrpcServer::GrpcServer() = default;

GrpcServer::~GrpcServer()
{
    stop();
}

void GrpcServer::registerRobot(int id, Robot* robot, Camera* camera, LidarSensor* lidar,
                                Dribbler* dribbler, Kicker* kicker, ImuSensor* imu)
{
    RobotBinding binding;
    binding.robot = robot;
    binding.camera = camera;
    binding.lidar = lidar;
    binding.dribbler = dribbler;
    binding.kicker = kicker;
    binding.imu = imu;
    m_bindings[id] = binding; // main/sim-thread-only, no lock needed

    // Exclusive lock: this may race a gRPC handler thread's shared-locked
    // find() in SendCommand/SensorStream/AddRobot (harmless once the queue
    // above only feeds addRobot() calls from this same thread, but a
    // concurrent std::map insert vs. find from different threads with no
    // synchronization at all is undefined behavior regardless).
    std::unique_lock<std::shared_mutex> lock(m_statesMutex);
    m_states[id]; // default-construct this robot's SharedState in place
}

void GrpcServer::start()
{
    int port = Config::instance().getInt("/network/grpc_port", 50051);
    std::string addr = "0.0.0.0:" + std::to_string(port);

    grpc::ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());

    m_service = std::make_unique<SimulatorServiceImpl>(m_states, m_statesMutex, m_addRobotQueue, m_shutdownRequested);
    builder.RegisterService(m_service.get());

    m_server = builder.BuildAndStart();
    if (!m_server) {
        std::cerr << "[gRPC] Failed to start server on " << addr << std::endl;
        return;
    }

    std::cout << "[gRPC] Listening on " << addr << " (" << m_bindings.size()
               << " robot(s) registered)" << std::endl;
    m_thread = std::thread([this]() { m_server->Wait(); });
}

void GrpcServer::stop()
{
    for (auto& [id, state] : m_states) {
        std::lock_guard<std::mutex> lock(state.mtx);
        state.shutdown = true;
        state.cv.notify_all();
    }

    // Fail (don't just drop) any AddRobot requests still queued -- update()
    // won't run again to drain them, and their handler threads are blocked
    // waiting on req->done, which would otherwise hang m_server->Shutdown()
    // (it waits for in-flight RPCs to finish).
    {
        std::vector<std::shared_ptr<PendingAddRobot>> toFail;
        {
            std::lock_guard<std::mutex> lock(m_addRobotQueue.mtx);
            while (!m_addRobotQueue.pending.empty()) {
                toFail.push_back(m_addRobotQueue.pending.front());
                m_addRobotQueue.pending.pop_front();
            }
        }
        for (auto& req : toFail) {
            {
                std::lock_guard<std::mutex> lock(req->mtx);
                req->success = false;
                req->message = "simulator shutting down";
                req->done = true;
            }
            req->cv.notify_all();
        }
    }

    if (m_server) m_server->Shutdown();
    if (m_thread.joinable()) m_thread.join();
    m_server.reset();
    m_service.reset();
}

void GrpcServer::processPending()
{
    // Drain runtime AddRobot requests, before update()'s per-robot loop runs
    // -- robot construction (Bullet body, camera FBOs) touches Bullet/GL and
    // must happen here on the main/sim thread, not on the gRPC handler
    // thread that received the request (see PendingAddRobot's own comment).
    // Doing this before update(), not during or after, means a robot added
    // this tick is fully registered (its m_bindings/m_states entries exist)
    // before its own command/publish handling below ever runs, instead of
    // depending on std::map's range-for visiting a key inserted mid-iteration.
    {
        std::vector<std::shared_ptr<PendingAddRobot>> toProcess;
        {
            std::lock_guard<std::mutex> lock(m_addRobotQueue.mtx);
            while (!m_addRobotQueue.pending.empty()) {
                toProcess.push_back(m_addRobotQueue.pending.front());
                m_addRobotQueue.pending.pop_front();
            }
        }
        for (auto& req : toProcess) {
            int id = -1;
            std::string message;
            if (m_addRobotFn) {
                id = m_addRobotFn(req->configPath, req->spawnX, req->spawnZ, req->spawnYawDeg);
                message = (id >= 0) ? "ok" : "addRobot failed (bad config path?)";
            } else {
                message = "server not ready to add robots yet";
            }
            {
                std::lock_guard<std::mutex> lock(req->mtx);
                req->success = (id >= 0);
                req->message = message;
                req->robotId = id;
                req->done = true;
            }
            req->cv.notify_all();
        }
    }
}

void GrpcServer::update(float dt)
{
    (void)dt;

    for (auto& [id, binding] : m_bindings) {
        // find(), not operator[] -- the latter is a potentially-modifying
        // operation even on a hit, which isn't safe to run concurrently with
        // a gRPC handler thread's own find() under only a shared lock. id
        // always has a matching m_states entry (registerRobot inserts both
        // together), so this never actually needs the exclusive path.
        SharedState* statePtr;
        {
            std::shared_lock<std::shared_mutex> lock(m_statesMutex);
            auto it = m_states.find(id);
            if (it == m_states.end()) continue; // shouldn't happen, see above
            statePtr = &it->second;
        }
        SharedState& state = *statePtr;

        // Consume any pending command for this robot under its own mutex, so
        // the command application stays race-free with SendCommand's handler
        // thread.
        {
            std::lock_guard<std::mutex> lock(state.mtx);
            if (state.hasCommand) {
                if (binding.robot) {
                    binding.robot->setBodyVelocity(state.vx, state.vy, state.omega);
                }
                if (binding.kicker) {
                    binding.kicker->setCapacitorOpen(state.capacitorChargeOpen);
                    binding.kicker->setKickerOpen(state.kickerOpen);
                }
                if (binding.dribbler) {
                    binding.dribbler->setTargetSpeed(state.dribbleSpeed);
                }
                state.hasCommand = false;
            }
        }

        // Publish this robot's latest sensor state and signal waiting
        // SensorStream RPCs only when its own camera actually rendered a new
        // image this tick -- see the single-robot version of this comment
        // (now per-robot, since each robot's camera has its own stream_fps
        // gating and frameChanged() flag).
        if (binding.camera && binding.camera->frameChanged()) {
            std::lock_guard<std::mutex> lock(state.mtx);
            state.image  = binding.camera->imageData();
            state.width  = binding.camera->imageWidth();
            state.height = binding.camera->imageHeight();
            if (binding.robot) {
                state.position        = binding.robot->position();
                state.yaw             = binding.robot->orientation();
                state.velocity        = binding.robot->velocity();
                state.angularVelocity = binding.robot->angularVelocity();
                state.odomX           = binding.robot->odometryX();
                state.odomZ           = binding.robot->odometryZ();
                state.odomYaw         = binding.robot->odometryYaw();
            }
            if (binding.lidar) {
                state.lidarPoints = binding.lidar->latestScan();
            }
            if (m_ball) {
                state.ballPosition = m_ball->position();
            }
            if (binding.dribbler) {
                state.dribblerRpm = binding.dribbler->rpm();
            }
            if (binding.kicker) {
                state.capacitorCharge = binding.kicker->charge();
                state.busVoltage = binding.kicker->busVoltage();
                state.capacitorVoltage = binding.kicker->capacitorVoltage();
            }
            if (binding.imu) {
                state.imuRoll = binding.imu->roll();
                state.imuPitch = binding.imu->pitch();
                state.imuYaw = binding.imu->yaw();
                state.imuAccel = binding.imu->acceleration();
                state.imuGyro = binding.imu->angularVelocity();
            }
            state.timestamp = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            state.frameId++;
            state.cv.notify_all();
        }
    }
}
