#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <stdexcept>

using json = nlohmann::json;

class Config
{
public:
    static Config& instance();

    void loadProject(const std::string& path);
    void loadRobot(const std::string& path);

    float getFloat(const std::string& key, float defaultVal) const;
    int getInt(const std::string& key, int defaultVal) const;
    std::string getString(const std::string& key, const std::string& defaultVal) const;
    const json& root() const { return m_data; }

    // Directory the project/robot JSON were loaded from (trailing '/'), so
    // config values that reference another file on disk (e.g. a mirror
    // profile CSV) can resolve a relative path the same way project.json/
    // robot.json themselves were found — see AGENTS.md's no-hardcoded-paths
    // convention.
    void setConfigDir(const std::string& dir) { m_configDir = dir; }
    const std::string& configDir() const { return m_configDir; }

private:
    Config() = default;
    json m_data;
    std::string m_configDir;

    const json* resolve(const std::string& dotPath) const;
};
