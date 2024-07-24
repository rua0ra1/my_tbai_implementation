#ifndef STATICCONTROLLER_HPP
#define STATICCONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>

#include "tbai_core/control/Controller.hpp"
#include "tbai_core/control/StateSubscriber.hpp"
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

namespace tbai {
namespace static_ {

class StaticController : public tbai::core::Controller {
   public:
    /**
     * @brief Construct a new StaticController object
     *
     * @param configRosParam : ROS parameter name for controller configuration file
     */
    StaticController(const std::string &configRosParam,
                     std::shared_ptr<tbai::core::StateSubscriber> stateSubscriberPtr);

    my_tbai_msgs::JointCommandArray getCommandMessage(scalar_t currentTime, scalar_t dt) override;

    void visualize() override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override {}

    scalar_t getRate() const override;

    bool checkStability() const override { return true; }

   private:
    /** Load settings from config file specified by ROS param*/
    void loadSettings(const std::string &configRosParam);

    /** Publish odom->base transforms */
    void publishOdomBaseTransforms(const vector_t &currentState, const ros::Time &currentTime);

    /** Publish joint angles */
    void publishJointAngles(const vector_t &currentState, const ros::Time &currentTime);

    /** Get command message during interpolation phase */
    my_tbai_msgs::JointCommandArray getInterpCommandMessage(scalar_t dt);

    /** Get command message when standing */
    my_tbai_msgs::JointCommandArray getStandCommandMessage();

    /** Get command message when sitting */
    my_tbai_msgs::JointCommandArray getSitCommandMessage();

    /** Pack desired joint angles into a command message */
    my_tbai_msgs::JointCommandArray packCommandMessage(const vector_t &jointAngles);

    /** State subscriber */
    std::shared_ptr<tbai::core::StateSubscriber> stateSubscriberPtr_;

    /** Visualization */
    tf::TransformBroadcaster tfBroadcaster_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

    /** Time since last visualization step */
    scalar_t timeSinceLastVisualizationUpdate_;

    /** PD constants */
    scalar_t kp_;
    scalar_t kd_;

    /** Stand joint angles */
    vector_t standJointAngles_;

    /** Sit joint angles */
    vector_t sitJointAngles_;

    /** How long should interpolation take */
    scalar_t interpolationTime_;

    /** Interp from and to */
    vector_t interpFrom_;
    vector_t interpTo_;

    /** Interpolation phase: -1 means interpolation has finished */
    scalar_t alpha_;

    /** Rate at which the controller should be running */
    scalar_t rate_;

    /** Joint names */
    std::vector<std::string> jointNames_;

    /** Current controller type */
    std::string currentControllerType_;
};

}  // namespace static_
}  // namespace tbai

#endif