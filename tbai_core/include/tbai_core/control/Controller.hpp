#pragma once

#include <string>

#include "tbai_core/Types.hpp"
#include <my_tbai_msgs/JointCommandArray.h>

namespace tbai {
namespace core {

class Controller {
   public:
    /** Desctructor */
    virtual ~Controller() = default;

    /**
     * @brief Produce command message for robot
     *
     * @param currentTime : Current time in seconds
     * @param dt : Time since last call in seconds, should be roughly 1/rate
     * @return my_tbai_msgs::JointCommandArray : ROS message with joint commands
     */
    virtual my_tbai_msgs::JointCommandArray getCommandMessage(scalar_t currentTime, scalar_t dt) = 0;

    /**
     * @brief Visualize robot (typically in RViz)
     *
     */
    virtual void visualize() = 0;

    /**
     * @brief Change controller to specified controller type. Note that this controllers isSupported() should evaluate
     * to true
     *
     * @param controllerType : controller type to change to
     * @param currentTime : Current time in seconds
     */
    virtual void changeController(const std::string &controllerType, scalar_t currentTime) = 0;

    /**
     * @brief Stop controller
     *
     */
    virtual void stopController() = 0;

    /**
     * @brief Check if specified controller is supported by this controller
     *
     * @param controllerType : controller type to check
     */
    virtual bool isSupported(const std::string &controllerType) = 0;

    /**
     * @brief Get rate at which the controller should be running
     *
     * @return scalar_t : rate in Hz
     */
    virtual scalar_t getRate() const = 0;

    /**
     * @brief Check if the controller is stable
     *
     * @return true : if controller is stable
     * @return false : if controller is not stable
     */
    virtual bool checkStability() const = 0;
};

}  // namespace core
}  // namespace tbai
