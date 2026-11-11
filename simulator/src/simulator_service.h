#pragma once

#include "shared_state.h"
#include "simulator.grpc.pb.h"
#include "simulator.pb.h"
#include <atomic>
#include <map>
#include <shared_mutex>

// gRPC service implementation for robocup::Simulator. Bridges the gRPC handler
// threads to the simulator via one mutex-guarded SharedState per robot_id
// (see GrpcServer). The map can grow at runtime (AddRobot, handled by
// GrpcServer::update() on the sim thread), so every lookup here takes
// statesMutex shared first -- only each SharedState's own mutex is needed
// afterward, to guard its fields.
class SimulatorServiceImpl final : public robocup::Simulator::Service
{
public:
    SimulatorServiceImpl(std::map<int, SharedState>& states, std::shared_mutex& statesMutex,
                          AddRobotQueue& addRobotQueue, std::atomic<bool>& shutdownFlag)
        : m_states(states), m_statesMutex(statesMutex), m_addRobotQueue(addRobotQueue),
          m_shutdownFlag(shutdownFlag) {}

    // Client-streaming: apply each command to its robot_id's pending-command slot.
    grpc::Status SendCommand(grpc::ServerContext* context,
                             grpc::ServerReader<robocup::RobotCommand>* reader,
                             robocup::CommandResponse* response) override;

    // Server-streaming: send the latest sensor frame for request.robot_id at
    // that robot's own publish rate.
    grpc::Status SensorStream(grpc::ServerContext* context,
                              const robocup::SensorRequest* request,
                              grpc::ServerWriter<robocup::SensorData>* writer) override;

    // Unary: queues the request and blocks until GrpcServer::update() (on
    // the sim/main thread) has actually constructed and registered the
    // robot -- see PendingAddRobot's own comment (shared_state.h).
    grpc::Status AddRobot(grpc::ServerContext* context,
                          const robocup::AddRobotRequest* request,
                          robocup::AddRobotResponse* response) override;

    // Unary: liveness/readiness probe used by the Python Simulator launcher.
    grpc::Status Ping(grpc::ServerContext* context, const robocup::PingRequest* request,
                      robocup::PingResponse* response) override;

    // Unary: asks the simulator to exit its main loop (window closes).
    grpc::Status Shutdown(grpc::ServerContext* context, const robocup::ShutdownRequest* request,
                          robocup::ShutdownResponse* response) override;

private:
    std::map<int, SharedState>& m_states;
    std::shared_mutex& m_statesMutex;
    AddRobotQueue& m_addRobotQueue;
    std::atomic<bool>& m_shutdownFlag;
};
