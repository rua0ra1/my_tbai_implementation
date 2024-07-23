#ifndef JOINTCONTROLLER_HPP
#define JOINTCONTROLLER_HPP
// System
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Config
#include <tbai_core/config/YamlConfig.hpp>

// Gazebo
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <urdf/model.h>

// ROS
#include <ros/ros.h>
#include <my_tbai_msgs/JointCommand.h>
#include <my_tbai_msgs/JointCommandArray.h>

namespace tbai {
namespace gazebo {

class JointController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
   public:
    /** Initialize joint controller plugin */
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);

    /** Update joint controllers - called after every simulation tick */
    void update(const ros::Time &time, const ros::Duration &period);

   private:
    /** JointCommandArray message callback */
    void jointCommandCallback(const my_tbai_msgs::JointCommandArrayConstPtr &commandArrayPtr);

    /** JointCommandArray subscriber */
    ros::Subscriber commandSubscriber_;

    /** Joint handle map */
    std::vector<hardware_interface::JointHandle> jointHandles_;

    /** Joint position limit map */
    std::vector<std::pair<tbai::scalar_t, tbai::scalar_t>> jointLimits_;

    /** Joint effort limit map */
    std::vector<tbai::scalar_t> effortLimits_;

    /** Joint index map */
    std::unordered_map<std::string, size_t> jointIndexMap_;

    /** Real time joint command buffer */
    realtime_tools::RealtimeBuffer<my_tbai_msgs::JointCommandArray> commandBuffer_;

    /** Joint positions from the previous simulation tick - used to calculate joint velocities */
    std::vector<tbai::scalar_t> lastJointPositions_;

    /** Number of joints */
    size_t numJoints_;

    /** First update flag */
    bool firstUpdate_ = true;
};

}  // namespace gazebo
}  // namespace tbai

#endif
