#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <stdexcept>

using json = nlohmann::json;

class Config
{
public:
    // Publicly constructible (not singleton-only): App::addRobot needs one
    // independent Config per robot (see withRobotOverlay). instance() below
    // remains the one shared/global config (project.json + world/network/
    // viewer settings) that every robot's own Config starts as a copy of.
    Config() = default;

    static Config& instance();

    void loadProject(const std::string& path);
    void loadRobot(const std::string& path);

    float getFloat(const std::string& key, float defaultVal) const;
    int getInt(const std::string& key, int defaultVal) const;
    std::string getString(const std::string& key, const std::string& defaultVal) const;
    const json& root() const { return m_data; }

    // Returns a copy of this config with a robot config file merged on top --
    // used to give each robot in a multi-robot field its own /robot/* and
    // /camera/* tree while still inheriting this config's shared /physics/,
    // /viewer/, /network/, /scene/, /field/ trees. See App::addRobot.
    Config withRobotOverlay(const std::string& path) const;

    // Overrides /robot/spawn/{x,z,yaw_deg} directly (x/z in millimetres,
    // yaw_deg in degrees), bypassing whatever the loaded robot JSON set --
    // lets App::addRobot place several robots spawned from the *same* config
    // file (e.g. a team) without needing a separate file per robot.
    void setRobotSpawn(float xMm, float zMm, float yawDeg);

    // Directory the project/robot JSON were loaded from (trailing '/'), so
    // config values that reference another file on disk (e.g. a mirror
    // profile CSV) can resolve a relative path the same way project.json/
    // robot.json themselves were found — see AGENTS.md's no-hardcoded-paths
    // convention.
    // Overrides /network/grpc_port (the --port CLI flag), so a launcher can
    // pick a free port without editing project.json.
    void setGrpcPort(int port);

    // Loads a match config (slots: team/robot_config/spawn per named slot)
    // under /match -- see SimulatorServiceImpl::Join.
    void loadMatch(const std::string& path);

    void setConfigDir(const std::string& dir) { m_configDir = dir; }
    const std::string& configDir() const { return m_configDir; }

private:
    json m_data;
    std::string m_configDir;

    const json* resolve(const std::string& dotPath) const;
};
