#pragma once

#include <ros/ros.h>
#include <tbai_core/Types.hpp>

namespace tbai {
namespace core {

inline void setEpochStart() {
    ros::NodeHandle().setParam("epoch_start", ros::Time::now().toSec());
}

inline bool isEpochStartSet() {
    return ros::NodeHandle().hasParam("epoch_start");
}

inline scalar_t getEpochStart() {
    if (!isEpochStartSet()) {
        ROS_ERROR("Epoch start time not set. Use setEpochStart() to set the epoch start time.");
        throw std::runtime_error("Epoch start time not set");
    }
    double epochStart;
    ros::param::get("epoch_start", epochStart);
    return static_cast<scalar_t>(epochStart);
}

}  // namespace core
}  // namespace tbai
