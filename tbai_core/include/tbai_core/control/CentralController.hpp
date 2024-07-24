#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tbai_core/Types.hpp"
#include "tbai_core/config/YamlConfig.hpp"
#include "tbai_core/control/Controller.hpp"
#include "tbai_core/control/StateSubscriber.hpp"
#include <std_msgs/String.h>

namespace tbai {
namespace core {

class CentralController {
   public:
    /**
     * @brief Construct a new Central Controller object
     *
     * @param nh : ROS node handle
     * @param stateTopic : topic to subscribe to for state messages
     * @param commandTopic : topic to publish command messages to
     * @param changeControllerTopic : topic to subscribe to for changing controller
     */
    CentralController(ros::NodeHandle &nh, const std::string &stateTopic, const std::string &commandTopic,
                      const std::string &changeControllerTopic);

    /**
     * @brief Construct a new Central Controller object, load parameters from a config file
     *
     * Note that the reired config keys are:
     *   - state_topic
     *   - command_topic,
     *   - change_controller_topic
     *
     * @param nh : ROS node handle
     * @param configParam : ROS parameter containing config file path
     *
     */
    CentralController(ros::NodeHandle &nh, const std::string &configParam);

    /**
     * @brief Start main control loop
     *
     */
    void start();

    /**
     * @brief Add controller to central controller
     *
     * @param controller : controller to add
     * @param makeActive : whether to make this controller active
     */
    void addController(std::unique_ptr<Controller> controller, bool makeActive = false);

    /**
     * @brief Get state subscriber pointer
     *
     * @return std::shared_ptr<StateSubscriber> : state subscriber pointer
     */
    const std::shared_ptr<StateSubscriber> &getStateSubscriberPtr();

    /**
     * @brief Get current time in seconds
     *
     * @return scalar_t : current time in seconds
     */
    inline scalar_t getCurrentTime() const { return (ros::Time::now().toSec() - initTime_); }

   private:
    /** Perform controller step */
    void step(scalar_t currentTime, scalar_t dt);

    /** Perform visualization step */
    inline void visualize() { activeController_->visualize(); }

    /** Callback for changing controller */
    void changeControllerCallback(const std_msgs::String::ConstPtr &msg);

    /** Controllers */
    std::vector<std::unique_ptr<Controller>> controllers_;

    /** Currently used controller */
    Controller *activeController_;

    /** State subscriber */
    std::shared_ptr<StateSubscriber> stateSubscriberPtr_;

    /** ROS stuff */
    ros::Rate loopRate_;
    ros::Publisher commandPublisher_;
    ros::Subscriber changeControllerSubscriber_;

    /** Whether or not the SIT controller is present */
    bool containsFallbackController_ = false;

    bool checkForFallbackController();
    void switchToFallbackController();
    std::string fallbackControllerType_;

    scalar_t initTime_;
};

}  // namespace core
}  // namespace tbai
