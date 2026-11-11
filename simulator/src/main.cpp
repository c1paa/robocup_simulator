#include "app.h"
#include <array>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <iostream>

static void printUsage(const char* prog)
{
    std::cout << "RoboCup Simulator v0.1\n\n"
              << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  --config-dir <path>          Path to configs directory (default: configs/)\n"
              << "  --port <n>                    gRPC listening port (default: /network/grpc_port)\n"
              << "  --match <path>                Match config with named slots; programs claim them\n"
              << "                                 over gRPC (Join). Implies --empty.\n"
              << "  --robot <path>                Add a robot loaded from this config file\n"
              << "                                 (relative to the working directory, not\n"
              << "                                 --config-dir). Repeat for a team --\n"
              << "                                 e.g. --robot configs/robot.json --robot\n"
              << "                                 configs/robot.json. If omitted entirely,\n"
              << "                                 one robot is added from <config-dir>/robot.json\n"
              << "                                 (the previous single-robot default).\n"
              << "  --robot-spawn <x> <z> <yaw>    Overrides the *next* --robot's spawn point\n"
              << "                                 (x/z in mm, yaw in degrees) -- otherwise it\n"
              << "                                 uses whatever its own config file's\n"
              << "                                 /robot/spawn/* says (default: origin), which\n"
              << "                                 means two --robot flags with the same config\n"
              << "                                 file will overlap unless you use this.\n"
              << "  --empty                        Don't add the default robot when no --robot\n"
              << "                                 flags are given either -- start with a truly\n"
              << "                                 empty field. Use this when you're going to add\n"
              << "                                 robots at runtime over gRPC (Simulator.AddRobot\n"
              << "                                 -- see python/robot_hal.py's SimulatorClient)\n"
              << "                                 instead of from the command line.\n"
              << "  --help, -h                     Show this help\n";
}

int main(int argc, char* argv[])
{
    std::string configDir = "configs/";
    std::vector<std::string> robotConfigs;
    std::vector<std::array<float, 3>> robotSpawns; // parallel to robotConfigs; {x, z, yaw_deg}
    bool haveSpawnOverride = false;
    std::array<float, 3> pendingSpawn{0.0f, 0.0f, 0.0f};
    bool startEmpty = false;
    std::string matchPath;
    int grpcPort = 0; // 0 = keep /network/grpc_port from project.json

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--config-dir") == 0 && i + 1 < argc) {
            configDir = argv[++i];
            if (configDir.back() != '/') configDir += '/';
        } else if (std::strcmp(argv[i], "--robot") == 0 && i + 1 < argc) {
            robotConfigs.push_back(argv[++i]);
            robotSpawns.push_back(haveSpawnOverride ? pendingSpawn : std::array<float, 3>{0.0f, 0.0f, 0.0f});
            haveSpawnOverride = false;
        } else if (std::strcmp(argv[i], "--robot-spawn") == 0 && i + 3 < argc) {
            pendingSpawn = { (float)std::atof(argv[i + 1]), (float)std::atof(argv[i + 2]),
                              (float)std::atof(argv[i + 3]) };
            haveSpawnOverride = true;
            i += 3;
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            grpcPort = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--match") == 0 && i + 1 < argc) {
            matchPath = argv[++i];
        } else if (std::strcmp(argv[i], "--empty") == 0) {
            startEmpty = true;
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            printUsage(argv[0]);
            return 0;
        }
    }

    App app;
    if (!app.init(configDir)) return 1;
    if (!matchPath.empty()) {
        try {
            Config::instance().loadMatch(matchPath);
        } catch (const std::exception& e) {
            std::cerr << "[Config] " << e.what() << std::endl;
            return 1;
        }
        startEmpty = true; // match robots join over gRPC
    }
    if (grpcPort > 0) Config::instance().setGrpcPort(grpcPort);

    // "In code" (here, or in any other caller of App::addRobot) is the real
    // way to build up a team -- --robot/--robot-spawn just expose the same
    // call from the command line for quick manual testing. See AGENTS.md's
    // App/RobotEntry entry and docs/tasks/multi-robot.md. Robots can also be
    // added after run() starts, over gRPC (Simulator.AddRobot) -- --empty
    // skips the single-robot fallback below for that workflow.
    if (robotConfigs.empty() && !startEmpty) {
        app.addRobot(configDir + "robot.json");
    } else {
        for (size_t i = 0; i < robotConfigs.size(); i++) {
            app.addRobot(robotConfigs[i], robotSpawns[i][0], robotSpawns[i][1], robotSpawns[i][2]);
        }
    }

    app.run();
    app.shutdown();
    return 0;
}
