#include "simulator_service.h"

grpc::Status SimulatorServiceImpl::SendCommand(
    grpc::ServerContext* context,
    grpc::ServerReader<robocup::RobotCommand>* reader,
    robocup::CommandResponse* response)
{
    (void)context;
    robocup::RobotCommand cmd;
    while (reader->Read(&cmd)) {
        // Only a single robot (id 0) exists today; reject anything else.
        if (cmd.robot_id() != 0) {
            response->set_success(false);
            response->set_message("Unknown robot_id " + std::to_string(cmd.robot_id()));
            return grpc::Status::OK;
        }

        std::lock_guard<std::mutex> lock(m_state.mtx);
        m_state.hasCommand = true;
        m_state.vx = cmd.vx();
        m_state.vy = cmd.vy();
        m_state.omega = cmd.omega();
        m_state.kickPower = cmd.kick_power();
        m_state.dribbleSpeed = cmd.dribble_speed();
    }

    response->set_success(true);
    response->set_message("ok");
    return grpc::Status::OK;
}

grpc::Status SimulatorServiceImpl::SensorStream(
    grpc::ServerContext* context,
    const robocup::SensorRequest* request,
    grpc::ServerWriter<robocup::SensorData>* writer)
{
    (void)request; // single robot, id always 0
    uint64_t lastFrame = 0;

    while (!context->IsCancelled()) {
        robocup::SensorData data;
        {
            std::unique_lock<std::mutex> lock(m_state.mtx);
            m_state.cv.wait(lock, [&] {
                return m_state.shutdown || m_state.frameId != lastFrame;
            });
            if (m_state.shutdown) break;
            lastFrame = m_state.frameId;

            data.set_robot_id(0);
            data.set_image_width(m_state.width);
            data.set_image_height(m_state.height);
            data.set_image_data(m_state.image.data(), m_state.image.size());
            data.set_pos_x(m_state.position.x);
            data.set_pos_y(m_state.position.y);
            data.set_pos_z(m_state.position.z);
            data.set_yaw(m_state.yaw);
            data.set_velocity(m_state.velocity);
            data.set_angular_velocity(m_state.angularVelocity);
            data.set_timestamp(m_state.timestamp);
            data.set_odom_x(m_state.odomX);
            data.set_odom_z(m_state.odomZ);
            data.set_odom_yaw(m_state.odomYaw);
            for (const LidarPoint& p : m_state.lidarPoints) {
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
