#pragma once

#include <algorithm>
#include <string>

#include "tbai_core/Types.hpp"
#include "my_tbai_msgs/RbdState.h"
#include <ros/ros.h>

namespace tbai {
namespace core {

class StateSubscriber {
   public:
    /**
     * @brief Construct a new State Subscriber object
     *
     * @param nh : ROS node handle
     * @param stateTopic : ROS topic name for state messages
     */
    StateSubscriber(ros::NodeHandle &nh, const std::string &stateTopic);

    /**
     * @brief Wait until the first state message is received
     *
     */
    void waitTillInitialized();

    /**
     * @brief Get latest Rbd state
     *
     * @return const vector_t& : latest rbd state
     */
    const vector_t &getLatestRbdState();

    /**
     * @brief Get the Latest Rbd Stamp
     *
     * @return const ros::Time& : latest rbd stamp
     */
    inline const ros::Time &getLatestRbdStamp() { return stateMessage_->stamp; }

    /**
     * @brief Get the Contact Flags
     *
     * @return std::arrat<bool, 4> : contact flags
     */
    inline std::array<bool, 4> getContactFlags() {
        std::array<bool, 4> contactFlags;
        std::copy(stateMessage_->contact_flags.begin(), stateMessage_->contact_flags.end(), contactFlags.begin());
        return contactFlags;
    }

   private:
    /** State message callback */
    void stateMessageCallback(const my_tbai_msgs::RbdState::Ptr &msg);

    /** Convert state message to vector_t */
    void updateLatestRbdState();

    /** Shared pointer to the latest state message */
    my_tbai_msgs::RbdState::Ptr stateMessage_;

    /** State message subscriber */
    ros::Subscriber stateSubscriber_;

    /** Whether or not the latest message has been converted to vector_t */
    bool stateReady_ = false;

    /** Latest Rbd state */
    vector_t latestRbdState_;
};

}  // namespace core
}  // namespace tbai
