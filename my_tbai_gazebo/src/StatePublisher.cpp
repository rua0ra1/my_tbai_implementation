// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "my_tbai_gazebo/StatePublisher.hpp"

#include <functional>
#include <string>

#include <Eigen/Geometry>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/YamlConfig.hpp>
#include <my_tbai_msgs/RbdState.h>

namespace gazebo {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StatePublisher::Load(physics::ModelPtr robot, sdf::ElementPtr sdf) {
    ROS_INFO_STREAM("[StatePublisher] Loading StatePublisher plugin");
    // set Gazebo callback function
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&StatePublisher::OnUpdate, this));

    this->robot_ = robot;
    auto config = tbai::core::YamlConfig::fromRosParam("/tbai_config_path");

    ros::NodeHandle nh;
    auto stateTopic = config.get<std::string>("state_topic");
    statePublisher_ = nh.advertise<my_tbai_msgs::RbdState>(stateTopic, 2);

    auto base = config.get<std::string>("base_name");
    baseLinkPtr_ = robot->GetChildLink(base);

    // get joints; ignore 'universe' and 'root_joint'
    auto jointNames = config.get<std::vector<std::string>>("joint_names");
    for (int i = 0; i < jointNames.size(); ++i) {
        joints_.push_back(robot->GetJoint(jointNames[i]));
    }

    // initialize last publish time
    lastSimTime_ = robot->GetWorld()->SimTime();

    rate_ = config.get<double>("state_publisher/update_rate");
    period_ = 1.0 / rate_;

    // Setup contact flags - TODO(lnotspotl): This is a bit hacky, remove hardcoding!
    std::vector<std::string> contactTopics = {"/lf_foot_contact", "/rf_foot_contact", "/lh_foot_contact",
                                              "/rh_foot_contact"};
    for (int i = 0; i < contactTopics.size(); ++i) {
        contactFlags_[i] = false;
        auto callback = [this, i](const std_msgs::Bool::ConstPtr &msg) { contactFlags_[i] = msg->data; };
        contactSubscribers_[i] = nh.subscribe<std_msgs::Bool>(contactTopics[i], 1, callback);
    }

    ROS_INFO_STREAM("[StatePublisher] Loaded StatePublisher plugin");
}  // namespace gazebo

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StatePublisher::OnUpdate() {
    // Get current time
    const common::Time currentTime = robot_->GetWorld()->SimTime();
    const double dt = (currentTime - lastSimTime_).Double();

    // Check if update is needed
    if (dt < period_) {
        return;
    }

    // Unpack base pose
    const ignition::math::Pose3d &basePoseIgn = baseLinkPtr_->WorldPose();
    const auto &basePositionIgn = basePoseIgn.Pos();
    const auto &baseOrientationIgn = basePoseIgn.Rot();

    // Base orientation - Euler zyx
    const Eigen::Quaternion<double> baseQuaternion(baseOrientationIgn.W(), baseOrientationIgn.X(),
                                                   baseOrientationIgn.Y(), baseOrientationIgn.Z());
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::core::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    // Base position in world frame
    const Eigen::Vector3d basePosition(basePositionIgn.X(), basePositionIgn.Y(), basePositionIgn.Z());

    if (firstUpdate_) {
        lastOrientationBase2World_ = R_base_world;
        lastPositionBase_ = basePosition;
        firstUpdate_ = false;
    }

    // Base angular velocity in base frame
    const Eigen::Vector3d angularVelocityWorld = tbai::core::mat2aa(R_world_base * lastOrientationBase2World_) / dt;
    const Eigen::Vector3d angularVelocityBase = R_base_world * angularVelocityWorld;

    // Base linear velocity in base frame
    const Eigen::Vector3d linearVelocityWorld = (basePosition - lastPositionBase_) / dt;
    const Eigen::Vector3d linearVelocityBase = R_base_world * linearVelocityWorld;

    // Joint angles
    std::vector<double> jointAngles(joints_.size());
    for (int i = 0; i < joints_.size(); ++i) {
        jointAngles[i] = joints_[i]->Position(0);
    }

    // Joint velocities
    if (lastJointAngles_.size() != joints_.size()) {
        lastJointAngles_.resize(joints_.size());
        for (size_t i = 0; i < joints_.size(); ++i) {
            lastJointAngles_[i] = jointAngles[i];
        }
    }

    // Get joint velocities
    std::vector<double> jointVelocities(joints_.size());
    for (int i = 0; i < joints_.size(); ++i) {
        jointVelocities[i] = (jointAngles[i] - lastJointAngles_[i]) / dt;
        lastJointAngles_[i] = jointAngles[i];
    }

    // Put everything into an RbdState message
    my_tbai_msgs::RbdState message;  // TODO(lnotspotl): Room for optimization here

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    message.rbd_state[0] = rpy[0];
    message.rbd_state[1] = rpy[1];
    message.rbd_state[2] = rpy[2];

    // Base position
    message.rbd_state[3] = basePosition[0];
    message.rbd_state[4] = basePosition[1];
    message.rbd_state[5] = basePosition[2];

    // Base angular velocity
    message.rbd_state[6] = angularVelocityBase[0];
    message.rbd_state[7] = angularVelocityBase[1];
    message.rbd_state[8] = angularVelocityBase[2];

    // Base linear velocity
    message.rbd_state[9] = linearVelocityBase[0];
    message.rbd_state[10] = linearVelocityBase[1];
    message.rbd_state[11] = linearVelocityBase[2];

    // Joint positions
    for (int i = 0; i < jointAngles.size(); ++i) {
        message.rbd_state[12 + i] = jointAngles[i];
    }

    // Joint velocities
    for (int i = 0; i < jointVelocities.size(); ++i) {
        message.rbd_state[12 + 12 + i] = jointVelocities[i];
    }

    // Observation time
    message.stamp = ros::Time::now();

    // Contact flags
    std::copy(contactFlags_.begin(), contactFlags_.end(), message.contact_flags.begin());

    lastOrientationBase2World_ = R_base_world;
    lastPositionBase_ = basePosition;

    // Publish message
    statePublisher_.publish(message);

    lastSimTime_ = currentTime;
}

GZ_REGISTER_MODEL_PLUGIN(StatePublisher);

}  // namespace gazebo
