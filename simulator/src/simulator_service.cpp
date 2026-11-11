#include "simulator_service.h"
#include <chrono>
#include <memory>

grpc::Status SimulatorServiceImpl::SendCommand(
    grpc::ServerContext* context,
    grpc::ServerReader<robocup::RobotCommand>* reader,
    robocup::CommandResponse* response)
{
    (void)context;
    robocup::RobotCommand cmd;
    while (reader->Read(&cmd)) {
        SharedState* statePtr;
        {
            std::shared_lock<std::shared_mutex> lock(m_statesMutex);
            auto it = m_states.find(cmd.robot_id());
            if (it == m_states.end()) {
                response->set_success(false);
                response->set_message("Unknown robot_id " + std::to_string(cmd.robot_id()));
                return grpc::Status::OK;
            }
            statePtr = &it->second;
        }
        SharedState& state = *statePtr;

        std::lock_guard<std::mutex> lock(state.mtx);
        state.hasCommand = true;
        state.vx = cmd.vx();
        state.vy = cmd.vy();
        state.omega = cmd.omega();
        state.dribbleSpeed = cmd.dribble_speed();
        state.capacitorChargeOpen = cmd.capacitor_charge_open();
        state.kickerOpen = cmd.kicker_open();
    }

    // Deliberately NOT resetting vx/vy/omega/dribble_speed when the client
    // stream ends: a normal workflow here is a script setting persistent
    // state (e.g. dribble(1.0)) and then exiting so the user can drive the
    // robot manually from the keyboard while that state stays in effect --
    // see docs/tasks/dribbler-kicker.md and the discussion around 2026-09-24.
    // Manual arrow-key driving overwrites vx/vy/omega directly on Robot
    // regardless (see App::handleKeyboardInput), so this doesn't strand a
    // driving command, only a dribble/kick one the user may still want.

    response->set_success(true);
    response->set_message("ok");
    return grpc::Status::OK;
}

grpc::Status SimulatorServiceImpl::SensorStream(
    grpc::ServerContext* context,
    const robocup::SensorRequest* request,
    grpc::ServerWriter<robocup::SensorData>* writer)
{
    SharedState* statePtr;
    {
        std::shared_lock<std::shared_mutex> lock(m_statesMutex);
        auto it = m_states.find(request->robot_id());
        if (it == m_states.end()) {
            return grpc::Status(grpc::StatusCode::NOT_FOUND,
                "Unknown robot_id " + std::to_string(request->robot_id()));
        }
        statePtr = &it->second;
    }
    SharedState& state = *statePtr;
    int robotId = request->robot_id();
    uint64_t lastFrame = 0;

    while (!context->IsCancelled()) {
        robocup::SensorData data;
        {
            std::unique_lock<std::mutex> lock(state.mtx);
            state.cv.wait(lock, [&] {
                return state.shutdown || state.frameId != lastFrame;
            });
            if (state.shutdown) break;
            lastFrame = state.frameId;

            data.set_robot_id(robotId);
            data.set_image_width(state.width);
            data.set_image_height(state.height);
            data.set_image_data(state.image.data(), state.image.size());
            data.set_pos_x(state.position.x);
            data.set_pos_y(state.position.y);
            data.set_pos_z(state.position.z);
            data.set_yaw(state.yaw);
            data.set_velocity(state.velocity);
            data.set_angular_velocity(state.angularVelocity);
            data.set_timestamp(state.timestamp);
            data.set_odom_x(state.odomX);
            data.set_odom_z(state.odomZ);
            data.set_odom_yaw(state.odomYaw);
            data.set_ball_pos_x(state.ballPosition.x);
            data.set_ball_pos_y(state.ballPosition.y);
            data.set_ball_pos_z(state.ballPosition.z);
            data.set_dribbler_rpm(state.dribblerRpm);
            data.set_capacitor_charge(state.capacitorCharge);
            data.set_bus_voltage(state.busVoltage);
            data.set_capacitor_voltage(state.capacitorVoltage);
            data.set_imu_roll(state.imuRoll);
            data.set_imu_pitch(state.imuPitch);
            data.set_imu_yaw(state.imuYaw);
            data.set_imu_accel_x(state.imuAccel.x);
            data.set_imu_accel_y(state.imuAccel.y);
            data.set_imu_accel_z(state.imuAccel.z);
            data.set_imu_gyro_x(state.imuGyro.x);
            data.set_imu_gyro_y(state.imuGyro.y);
            data.set_imu_gyro_z(state.imuGyro.z);
            for (const LidarPoint& p : state.lidarPoints) {
                robocup::LidarPoint* lp = data.add_lidar_points();
                lp->set_angle(p.angle);
                lp->set_distance(p.distance);
                lp->set_intensity(p.intensity);
            }
        }
        if (!writer->Write(data)) break;
    }

    return grpc::Status::OK;
}

grpc::Status SimulatorServiceImpl::AddRobot(
    grpc::ServerContext* context,
    const robocup::AddRobotRequest* request,
    robocup::AddRobotResponse* response)
{
    auto pending = std::make_shared<PendingAddRobot>();
    pending->configPath = request->config_path();
    pending->spawnX = request->spawn_x();
    pending->spawnZ = request->spawn_z();
    pending->spawnYawDeg = request->spawn_yaw_deg();

    {
        std::lock_guard<std::mutex> lock(m_addRobotQueue.mtx);
        m_addRobotQueue.pending.push_back(pending);
    }

    // Blocks until GrpcServer::update() (sim/main thread) drains the queue
    // and actually constructs the robot -- see PendingAddRobot's comment for
    // why that can't happen directly on this handler thread.
    std::unique_lock<std::mutex> lock(pending->mtx);
    // Poll so a cancelled/timed-out client doesn't pin this handler thread
    // forever (e.g. if the sim loop is gone).
    while (!pending->cv.wait_for(lock, std::chrono::milliseconds(100),
                                 [&] { return pending->done; })) {
        if (context->IsCancelled()) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "client cancelled AddRobot");
        }
    }

    response->set_success(pending->success);
    response->set_message(pending->message);
    response->set_robot_id(pending->robotId);
    return grpc::Status::OK;
}

grpc::Status SimulatorServiceImpl::Ping(grpc::ServerContext*, const robocup::PingRequest*,
                                        robocup::PingResponse* response)
{
    std::shared_lock<std::shared_mutex> lock(m_statesMutex);
    response->set_robot_count(static_cast<int32_t>(m_states.size()));
    return grpc::Status::OK;
}

grpc::Status SimulatorServiceImpl::Shutdown(grpc::ServerContext*, const robocup::ShutdownRequest*,
                                            robocup::ShutdownResponse*)
{
    m_shutdownFlag.store(true);
    return grpc::Status::OK;
}
