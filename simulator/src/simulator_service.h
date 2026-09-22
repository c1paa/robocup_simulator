#pragma once

#include "shared_state.h"
#include "simulator.grpc.pb.h"
#include "simulator.pb.h"

// gRPC service implementation for robocup::Simulator. Bridges the gRPC handler
// threads to the simulator via a mutex-guarded SharedState.
class SimulatorServiceImpl final : public robocup::Simulator::Service
{
public:
    explicit SimulatorServiceImpl(SharedState& state) : m_state(state) {}

    // Client-streaming: apply each command to the shared pending-command slot.
    grpc::Status SendCommand(grpc::ServerContext* context,
                             grpc::ServerReader<robocup::RobotCommand>* reader,
                             robocup::CommandResponse* response) override;

    // Server-streaming: send the latest sensor frame at the sim frame rate.
    grpc::Status SensorStream(grpc::ServerContext* context,
                              const robocup::SensorRequest* request,
                              grpc::ServerWriter<robocup::SensorData>* writer) override;

private:
    SharedState& m_state;
};
