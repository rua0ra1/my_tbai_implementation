#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace tbai {
namespace core {

class YamlConfig {
   public:
    /**
     * @brief Construct a new Yaml Config object
     *
     * @param configPath : Path to yaml config file
     * @param delim  : config path delimiter
     * @param performChecks : Perform checks (whether or not a key exists) when getting a value
     */
    YamlConfig(const std::string &configPath, const char delim = '/', bool performChecks = true);

    /**
     * @brief Construct YamlConfig with config path specified by a ROS parameter
     *
     * @param pathParam : ROS parameter name containing config's path
     * @param delim : config path delimiter
     * @param performChecks : Perform checks (whether or not a key exists) when getting a value
     *
     * @return YamlConfig object
     */
    static YamlConfig fromRosParam(const std::string &pathParam, const char delim = '/', bool performChecks = true);

    /**
     * @brief Get value from config
     *
     * @tparam T : Type of value to get
     * @param path : Path to value in config
     * @return T : Value from config
     */
    template <typename T>
    T get(const std::string &path) const;

   private:
    /** Split config path: a.b.c -> {'a', 'b', 'c'} if delim_ == '.' */
    std::vector<std::string> split(const std::string &s) const;

    /** Check if key exists */
    void checkExists(const YAML::Node &node, const std::string &key) const;

    /** Get value from a YAML node */
    template <typename T>
    T parseNode(const YAML::Node &node) const;

    /** Get YAML node specified by its path in config*/
    template <typename T>
    T traverse(const YAML::Node &node, const std::string &key) const;

    std::string configPath_;
    char delim_;
    bool performChecks_;
};

/** Retrieve parameter from tbai yaml config file
 *
 * Note: 'tbai_config_path' parameter must be set in ROS parameter server
 *
 * @tparam T : Type of value to get
 * @param path : Path to value in config
 * @param delim : config path delimiter
 * @param configParam : ROS parameter name containing config's path
 *
 */
template <typename T>
T fromRosConfig(const std::string &path, const char delim = '/', const std::string &configParam = "tbai_config_path");

}  // namespace core
}  // namespace tbai

#include "tbai_core/config/implementation/YamlConfig.hpp"
