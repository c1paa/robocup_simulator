#include "app.h"
#include <string>
#include <cstring>
#include <iostream>

static void printUsage(const char* prog)
{
    std::cout << "RoboCup Sumilator v0.1\n\n"
              << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  --config-dir <path>   Path to configs directory (default: configs/)\n"
              << "  --help, -h            Show this help\n";
}

int main(int argc, char* argv[])
{
    std::string configDir = "configs/";

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--config-dir") == 0 && i + 1 < argc) {
            configDir = argv[++i];
            if (configDir.back() != '/') configDir += '/';
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            printUsage(argv[0]);
            return 0;
        }
    }

    App app;
    if (!app.init(configDir)) return 1;
    app.run();
    app.shutdown();
    return 0;
}
