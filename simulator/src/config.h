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

private:
    Config() = default;
    json m_data;

    const json* resolve(const std::string& dotPath) const;
};
