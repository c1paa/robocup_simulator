#include "config.h"
#include <fstream>
#include <iostream>
#include <sstream>

Config& Config::instance()
{
    static Config c;
    return c;
}

void Config::loadProject(const std::string& path)
{
    std::cout << "[Config] Loading project: " << path << std::endl;
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot open " + path);
    json proj;
    f >> proj;
    m_data.merge_patch(proj);
}

void Config::loadRobot(const std::string& path)
{
    std::cout << "[Config] Loading robot: " << path << std::endl;
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot open " + path);
    json robot;
    f >> robot;
    m_data.merge_patch(robot);
}

const json* Config::resolve(const std::string& dotPath) const
{
    const json* current = &m_data;
    std::istringstream ss(dotPath);
    std::string token;
    while (std::getline(ss, token, '/')) {
        if (token.empty()) continue;
        if (!current->is_object() || !current->contains(token)) return nullptr;
        current = &(*current)[token];
    }
    return current;
}

float Config::getFloat(const std::string& key, float defaultVal) const
{
    const json* node = resolve(key);
    return (node && node->is_number()) ? node->get<float>() : defaultVal;
}

int Config::getInt(const std::string& key, int defaultVal) const
{
    const json* node = resolve(key);
    return (node && node->is_number_integer()) ? node->get<int>() : defaultVal;
}

std::string Config::getString(const std::string& key, const std::string& defaultVal) const
{
    const json* node = resolve(key);
    return (node && node->is_string()) ? node->get<std::string>() : defaultVal;
}
