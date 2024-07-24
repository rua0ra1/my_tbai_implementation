#include "tbai_core/config/YamlConfig.hpp"

#include <ros/ros.h>

namespace tbai {
namespace core {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
YamlConfig::YamlConfig(const std::string &configPath, const char delim, bool performChecks)
    : configPath_(configPath), delim_(delim), performChecks_(performChecks) {}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
YamlConfig YamlConfig::fromRosParam(const std::string &pathParam, const char delim, bool performChecks) {
    std::string configPath;
    ros::param::get(pathParam, configPath);
    return YamlConfig(configPath, delim, performChecks);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
std::vector<std::string> YamlConfig::split(const std::string &s) const {
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim_)) {
        result.push_back(item);
    }
    return result;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/************************************************************`**********************************************************/
void YamlConfig::checkExists(const YAML::Node &node, const std::string &key) const {
    YAML::Node component(node);
    for (auto &k : split(key)) {
        if (!component[k]) {
            throw std::runtime_error("Key '" + key + "' does not exist in config file.");
        }
        component = component[k];
    }
}

}  // namespace core
}  // namespace tbai
