#include "grpc_server.h"
#include <iostream>

void GrpcServer::start()
{
    std::cout << "[gRPC] Server placeholder. Will start on localhost:50051" << std::endl;
}

void GrpcServer::stop()
{
    std::cout << "[gRPC] Server stopped." << std::endl;
}

void GrpcServer::update(float dt)
{
    (void)dt;
    // Placeholder: process incoming gRPC commands, send sensor data
}
