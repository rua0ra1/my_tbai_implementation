#pragma once

#include <iostream>
#include <string>

#include "tbai_core/Types.hpp"
#include <ros/package.h>

namespace tbai {

namespace core {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
template <typename T>
T YamlConfig::traverse(const YAML::Node &node, const std::string &key) const {
    YAML::Node component(node);
    for (auto &k : split(key)) {
        component = component[k];
    }
    return parseNode<T>(component);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
template <typename T>
T YamlConfig::parseNode(const YAML::Node &node) const {
    return node.as<T>();
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
/// \cond  // TODO(lnotspotl): This specialization breaks doxygen
template <>
inline vector_t YamlConfig::parseNode(const YAML::Node &node) const {
    const size_t len = node.size();
    vector_t output(len);
    for (size_t i = 0; i < len; ++i) {
        output(i) = node[i].as<scalar_t>();
    }
    return output;
}
/// \endcond

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
/// \cond  // TODO(lnotspotl): This specialization breaks doxygen
template <>
inline matrix_t YamlConfig::parseNode(const YAML::Node &node) const {
    const size_t rows = node.size();
    const size_t cols = node[0].size();
    matrix_t output(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            output(i, j) = node[i][j].as<scalar_t>();
        }
    }
    return output;
}
/// \endcond

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
/// \cond  // TODO(lnotspotl): This specialization breaks doxygen
template <>
inline std::string YamlConfig::parseNode(const YAML::Node &node) const {
    auto out = node.as<std::string>();

    const std::string prefix = "package://";
    if (out.substr(0, prefix.size()) == prefix) {
        // find package name
        const std::string package_name = out.substr(prefix.size(), out.find('/', prefix.size()) - prefix.size());

        // update path
        out = ros::package::getPath(package_name) + out.substr(prefix.size() + package_name.size());
    }

    return out;
}
/// \endcond

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
template <typename T>
T YamlConfig::get(const std::string &path) const {
    YAML::Node config = YAML::LoadFile(configPath_);
    if (performChecks_) {
        YAML::Node configCheck = YAML::Clone(config);
        checkExists(configCheck, path);
    }
    return traverse<T>(config, path);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
template <typename T>
T fromRosConfig(const std::string &path, const char delim, const std::string &configParam) {
    auto config = YamlConfig::fromRosParam(configParam, delim);
    return config.get<T>(path);
}

}  // namespace core
}  // namespace tbai
