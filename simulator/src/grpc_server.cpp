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

void GrpcServer::start()
{
    int port = Config::instance().getInt("/network/grpc_port", 50051);
    std::string addr = "0.0.0.0:" + std::to_string(port);

    grpc::ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());

    m_service = std::make_unique<SimulatorServiceImpl>(m_state);
    builder.RegisterService(m_service.get());

    m_server = builder.BuildAndStart();
    if (!m_server) {
        std::cerr << "[gRPC] Failed to start server on " << addr << std::endl;
        return;
    }

    std::cout << "[gRPC] Listening on " << addr << std::endl;
    m_thread = std::thread([this]() { m_server->Wait(); });
}

void GrpcServer::stop()
{
    {
        std::lock_guard<std::mutex> lock(m_state.mtx);
        m_state.shutdown = true;
        m_state.cv.notify_all();
    }
    if (m_server) m_server->Shutdown();
    if (m_thread.joinable()) m_thread.join();
    m_server.reset();
    m_service.reset();
}

void GrpcServer::update(float dt)
{
    (void)dt;

    // Consume any pending drive command under the same mutex the gRPC thread
    // uses, so the command application stays race-free.
    {
        std::lock_guard<std::mutex> lock(m_state.mtx);
        if (m_state.hasCommand && m_robot) {
            m_robot->setBodyVelocity(m_state.vx, m_state.vy, m_state.omega);
            if (m_kicker) {
                m_kicker->setCapacitorOpen(m_state.capacitorChargeOpen);
                m_kicker->setKickerOpen(m_state.kickerOpen);
            }
            if (m_dribbler) {
                m_dribbler->setTargetSpeed(m_state.dribbleSpeed);
            }
            m_state.hasCommand = false;
        }
    }

    // Publish the latest sensor state and signal waiting SensorStream RPCs
    // only when the camera actually rendered a new image this tick — camera
    // updates are throttled to /camera/stream_fps, well below the main loop
    // rate, and re-publishing the same bytes under a new frameId would just
    // make SensorStream send duplicate frames.
    if (m_camera && m_camera->frameChanged()) {
        std::lock_guard<std::mutex> lock(m_state.mtx);
        m_state.image  = m_camera->imageData();
        m_state.width  = m_camera->imageWidth();
        m_state.height = m_camera->imageHeight();
        if (m_robot) {
            m_state.position        = m_robot->position();
            m_state.yaw             = m_robot->orientation();
            m_state.velocity        = m_robot->velocity();
            m_state.angularVelocity = m_robot->angularVelocity();
            m_state.odomX           = m_robot->odometryX();
            m_state.odomZ           = m_robot->odometryZ();
            m_state.odomYaw         = m_robot->odometryYaw();
        }
        if (m_lidar) {
            m_state.lidarPoints = m_lidar->latestScan();
        }
        if (m_ball) {
            m_state.ballPosition = m_ball->position();
        }
        if (m_dribbler) {
            m_state.dribblerRpm = m_dribbler->rpm();
        }
        if (m_kicker) {
            m_state.capacitorCharge = m_kicker->charge();
            m_state.busVoltage = m_kicker->busVoltage();
            m_state.capacitorVoltage = m_kicker->capacitorVoltage();
        }
        if (m_imu) {
            m_state.imuRoll = m_imu->roll();
            m_state.imuPitch = m_imu->pitch();
            m_state.imuYaw = m_imu->yaw();
            m_state.imuAccel = m_imu->acceleration();
            m_state.imuGyro = m_imu->angularVelocity();
        }
        m_state.timestamp = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        m_state.frameId++;
        m_state.cv.notify_all();
    }
}
