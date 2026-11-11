#include "simulator_service.h"
#include "config.h"
#include <random>
#include <chrono>
#include <memory>

bool SimulatorServiceImpl::authorized(grpc::ServerContext* context, SharedState& state)
{
    std::string required;
    {
        std::lock_guard<std::mutex> lock(state.mtx);
        required = state.token;
    }
    if (required.empty()) return true;
    const auto& md = context->client_metadata();
    auto it = md.find("x-robot-token");
    return it != md.end() && std::string(it->second.data(), it->second.size()) == required;
}

grpc::Status SimulatorServiceImpl::SendCommand(
    grpc::ServerContext* context,
    grpc::ServerReader<robocup::RobotCommand>* reader,
    robocup::CommandResponse* response)
{
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
        if (!authorized(context, state)) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED,
                "Missing or wrong robot token for robot_id " + std::to_string(cmd.robot_id()));
        }

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
    if (!authorized(context, state)) {
        return grpc::Status(grpc::StatusCode::PERMISSION_DENIED,
            "Missing or wrong robot token for robot_id " + std::to_string(request->robot_id()));
    }
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

grpc::Status SimulatorServiceImpl::Join(grpc::ServerContext* context,
                                        const robocup::JoinRequest* request,
                                        robocup::JoinResponse* response)
{
    auto fail = [&](const std::string& msg) {
        response->set_success(false);
        response->set_message(msg);
        return grpc::Status::OK;
    };

    const json& root = Config::instance().root();
    if (!root.contains("match") || !root["match"].contains("slots")) {
        return fail("simulator was not started with --match <config>");
    }
    const json& slots = root["match"]["slots"];
    const std::string& slot = request->slot();
    if (!slots.contains(slot)) {
        std::string known;
        for (auto it = slots.begin(); it != slots.end(); ++it) known += (known.empty() ? "" : ", ") + it.key();
        return fail("unknown slot '" + slot + "' (available: " + known + ")");
    }
    const json& def = slots[slot];
    std::string team = def.value("team", "");

    std::lock_guard<std::mutex> claimLock(m_slotsMutex);
    auto claimed = m_claims.find(slot);
    if (claimed != m_claims.end()) {
        if (!request->token().empty() && request->token() == claimed->second.token) {
            response->set_success(true);
            response->set_message("reconnected");
            response->set_robot_id(claimed->second.robotId);
            response->set_token(claimed->second.token);
            response->set_team(team);
            return grpc::Status::OK;
        }
        return fail("slot '" + slot + "' is already taken");
    }

    std::string robotCfg = def.value("robot_config", "robot.json");
    if (!robotCfg.empty() && robotCfg[0] != '/') robotCfg = Config::instance().configDir() + robotCfg;
    float x = 0, z = 0, yaw = 0;
    if (def.contains("spawn")) {
        x = def["spawn"].value("x", 0.0f);
        z = def["spawn"].value("z", 0.0f);
        yaw = def["spawn"].value("yaw_deg", 0.0f);
    }

    auto pending = std::make_shared<PendingAddRobot>();
    pending->configPath = robotCfg;
    pending->spawnX = x;
    pending->spawnZ = z;
    pending->spawnYawDeg = yaw;
    {
        std::lock_guard<std::mutex> lock(m_addRobotQueue.mtx);
        m_addRobotQueue.pending.push_back(pending);
    }
    {
        std::unique_lock<std::mutex> lock(pending->mtx);
        while (!pending->cv.wait_for(lock, std::chrono::milliseconds(100), [&] { return pending->done; })) {
            if (context->IsCancelled()) return grpc::Status(grpc::StatusCode::CANCELLED, "client cancelled Join");
        }
        if (!pending->success) return fail(pending->message);
    }

    std::random_device rd;
    std::mt19937_64 gen(rd());
    char buf[33];
    snprintf(buf, sizeof buf, "%016llx%016llx", (unsigned long long)gen(), (unsigned long long)gen());
    std::string token = buf;

    {
        std::shared_lock<std::shared_mutex> lock(m_statesMutex);
        auto it = m_states.find(pending->robotId);
        if (it != m_states.end()) {
            std::lock_guard<std::mutex> sl(it->second.mtx);
            it->second.token = token;
        }
    }
    m_claims[slot] = {pending->robotId, token};

    response->set_success(true);
    response->set_message("ok");
    response->set_robot_id(pending->robotId);
    response->set_token(token);
    response->set_team(team);
    return grpc::Status::OK;
}

grpc::Status SimulatorServiceImpl::ListRobots(grpc::ServerContext*, const robocup::ListRobotsRequest*,
                                              robocup::ListRobotsResponse* response)
{
    const json& root = Config::instance().root();
    if (!root.contains("match") || !root["match"].contains("slots")) return grpc::Status::OK;
    std::lock_guard<std::mutex> lock(m_slotsMutex);
    for (auto it = root["match"]["slots"].begin(); it != root["match"]["slots"].end(); ++it) {
        robocup::SlotInfo* info = response->add_slots();
        info->set_slot(it.key());
        info->set_team(it.value().value("team", ""));
        auto c = m_claims.find(it.key());
        info->set_claimed(c != m_claims.end());
        info->set_robot_id(c != m_claims.end() ? c->second.robotId : -1);
    }
    return grpc::Status::OK;
}
