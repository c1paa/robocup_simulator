#pragma once

#include "shared_state.h"
#include <memory>
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
    void setLidar(LidarSensor* lidar) { m_lidar = lidar; }
    void setBall(Ball* ball) { m_ball = ball; }
    void setDribbler(Dribbler* dribbler) { m_dribbler = dribbler; }
    void setKicker(Kicker* kicker) { m_kicker = kicker; }
    void setImu(ImuSensor* imu) { m_imu = imu; }

private:
    SharedState m_state;
    std::thread m_thread;
    std::unique_ptr<grpc::Server> m_server;
    std::unique_ptr<SimulatorServiceImpl> m_service;
    Robot* m_robot = nullptr;
    Ball* m_ball = nullptr;
    Camera* m_camera = nullptr;
    LidarSensor* m_lidar = nullptr;
    Dribbler* m_dribbler = nullptr;
    Kicker* m_kicker = nullptr;
    ImuSensor* m_imu = nullptr;
};
