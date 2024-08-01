#include "tbai_mpc/wbc/Factory.hpp"

#include <tbai_core/config/YamlConfig.hpp>

namespace switched_model {

std::unique_ptr<WbcBase> getWbcUnique(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const switched_model::ComModelBase<scalar_t> &comModel,
                                      const switched_model::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames) {
    auto wbcType = tbai::core::fromRosConfig<std::string>("mpc_controller/wbc_type");

    if (wbcType == "hqp") {
        return std::make_unique<HqpWbc>(controllerConfigFile, urdfString, comModel, kinematics, jointNames);
    }

    if (wbcType == "sqp") {
        return std::make_unique<SqpWbc>(controllerConfigFile, urdfString, comModel, kinematics, jointNames);
    }

    throw std::runtime_error("Unknown WBC type: " + wbcType);
}

std::shared_ptr<WbcBase> getWbcShared(const std::string &controllerConfigFile, const std::string &urdfString,
                                      const switched_model::ComModelBase<scalar_t> &comModel,
                                      const switched_model::KinematicsModelBase<scalar_t> &kinematics,
                                      const std::vector<std::string> &jointNames) {
    return std::shared_ptr<WbcBase>(
        getWbcUnique(controllerConfigFile, urdfString, comModel, kinematics, jointNames).release());
}

}  // namespace switched_model
