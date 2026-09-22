#pragma once

#include "shared_state.h"
#include <memory>
#include <thread>

namespace grpc { class Server; }

class Robot;
class Camera;
class SimulatorServiceImpl;

class GrpcServer
{
public:
    GrpcServer();
    ~GrpcServer();

    void start();
    void stop();
    void update(float dt);

    void setRobot(Robot* robot) { m_robot = robot; }
    void setCamera(Camera* camera) { m_camera = camera; }

private:
    SharedState m_state;
    std::thread m_thread;
    std::unique_ptr<grpc::Server> m_server;
    std::unique_ptr<SimulatorServiceImpl> m_service;
    Robot* m_robot = nullptr;
    Camera* m_camera = nullptr;
};
