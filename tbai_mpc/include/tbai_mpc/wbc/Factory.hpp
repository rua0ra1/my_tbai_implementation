#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tbai_mpc/wbc/HqpWbc.hpp"
#include "tbai_mpc/wbc/SqpWbc.hpp"
#include <ocs2_anymal_mpc/AnymalInterface.h>

namespace switched_model {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const switched_model::ComModelBase<scalar_t> &comModel,
                                      const switched_model::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames);

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const switched_model::ComModelBase<scalar_t> &comModel,
                                      const switched_model::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames);

}  // namespace switched_model
