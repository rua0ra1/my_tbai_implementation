#include "my_tbai_gazebo/JointController.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace tbai {
namespace gazebo {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool JointController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    // Setup yaml config
    const std::string configParam = "tbai_config_path";
    auto config = tbai::core::YamlConfig::fromRosParam(configParam);

    // Load joint names
    auto jointNames = config.get<std::vector<std::string>>("joint_names");
    ROS_INFO_STREAM("[JointController] Loading " << jointNames.size() << " joint controllers.");
    for (size_t i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[JointController] Joint " << i + 1 << ": " << jointNames[i]);
    }

    // Load model URDF
    urdf::Model urdf;
    if (!urdf.initParam("robot_description")) {  // TODO(lnotspotl): Parametrize this
        ROS_ERROR("Could not parse urdf file! Failed to initialize.");
        return false;
    }

    for (size_t i = 0; i < jointNames.size(); ++i) {
        std::string &jointName = jointNames[i];

        // Get joint handle
        try {
            jointHandles_.push_back(hw->getHandle(jointName));
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM("Could not find joint '" << jointName << "' in hardware interface");
            return false;
        }

        // Get joint position limits
        std::pair<tbai::scalar_t, tbai::scalar_t> jointLimit;
        urdf::JointConstSharedPtr jointUrdf = urdf.getJoint(jointName);
        if (!jointUrdf) {
            ROS_ERROR_STREAM("Could not find joint '" << jointName << "' in urdf");
            return false;
        }
        jointLimit.first = jointUrdf->limits->lower;
        jointLimit.second = jointUrdf->limits->upper;
        jointLimits_.push_back(jointLimit);

        // Get joint effort limits
        effortLimits_.push_back(jointUrdf->limits->effort);

        // Get joint index
        jointIndexMap_[jointName] = i;
    }

    // Load command topic
    auto commandTopic = config.get<std::string>("command_topic");
    ROS_INFO_STREAM("[JointController] Subscribing to " << commandTopic);

    // Subscribe to command topic
    ros::NodeHandle nh;
    commandSubscriber_ =
        nh.subscribe<my_tbai_msgs::JointCommandArray>(commandTopic, 1, &JointController::jointCommandCallback, this);

    // Initialize command buffer
    commandBuffer_.writeFromNonRT(my_tbai_msgs::JointCommandArray());

    // Store number of joints
    numJoints_ = jointNames.size();

    return true;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void JointController::update(const ros::Time &time, const ros::Duration &period) {
    // Read command buffer
    my_tbai_msgs::JointCommandArray &commandArray = *commandBuffer_.readFromRT();

    // Update last joint positions if this is the first update
    if (firstUpdate_) {
        lastJointPositions_.resize(numJoints_);
        for (size_t i = 0; i < numJoints_; ++i) {
            lastJointPositions_[i] = jointHandles_[i].getPosition();
        }
        firstUpdate_ = false;
    }

    const scalar_t dt = period.toSec();
    const size_t nCommands = commandArray.joint_commands.size();
    for (int i = 0; i < nCommands; ++i) {
        // Unpack JointCommand message
        my_tbai_msgs::JointCommand &command = commandArray.joint_commands[i];
        std::string &jointName = command.joint_name;
        const scalar_t desiredPosition = command.desired_position;
        const scalar_t desiredVelocity = command.desired_velocity;
        const scalar_t kp = command.kp;
        const scalar_t kd = command.kd;
        const scalar_t torque_ff = command.torque_ff;

        // Get joint handle
        const size_t jointIdx = jointIndexMap_[jointName];
        hardware_interface::JointHandle &joint = jointHandles_[jointIdx];

        // Get current joint position
        const scalar_t currentPosition = joint.getPosition();

        // Get current joint velocity
        const scalar_t currentVelocity = (currentPosition - lastJointPositions_[jointIdx]) / dt;

        // Update last joint position
        lastJointPositions_[jointIdx] = currentPosition;

        // Compute torque
        const scalar_t positionError = desiredPosition - currentPosition;
        const scalar_t velocityError = desiredVelocity - currentVelocity;
        const scalar_t torque = torque_ff + kp * positionError + kd * velocityError;

        // Clip torque
        const scalar_t effortLimit = effortLimits_[jointIdx];
        const scalar_t torque_clipped = std::max(-effortLimit, std::min(effortLimit, torque));

        // Set torque
        joint.setCommand(torque_clipped);
    }
}

void JointController::jointCommandCallback(const my_tbai_msgs::JointCommandArrayConstPtr &commandArrayPtr) {
    commandBuffer_.writeFromNonRT(*commandArrayPtr);
}

}  // namespace gazebo
}  // namespace tbai

PLUGINLIB_EXPORT_CLASS(tbai::gazebo::JointController, controller_interface::ControllerBase);
