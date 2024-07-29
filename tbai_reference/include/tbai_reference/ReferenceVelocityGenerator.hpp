#pragma once

#include <memory>
#include <string>

#include "tbai_reference/ReferenceVelocity.hpp"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tbai_core/Types.hpp>

namespace tbai {
namespace reference {

class ReferenceVelocityGenerator {
   public:
    /** Destructor*/
    virtual ~ReferenceVelocityGenerator() = default;

    /**
     * @brief Generate reference velocity
     * @param time Current time
     * @param dt Time since the last call
     * @return Reference velocity
     */
    virtual ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) = 0;
};

template <typename T>
class RosVelocityReferenceGenerator : public ReferenceVelocityGenerator {
   public:
    RosVelocityReferenceGenerator(ros::NodeHandle &nh, const std::string &topic) {
        subscriber_ = nh.subscribe(topic, 1, &RosVelocityReferenceGenerator::callback, this);
    }
    ~RosVelocityReferenceGenerator() override = default;

    ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) override { return reference_; }

   protected:
    ReferenceVelocity reference_;

   private:
    /** ROS message callback */
    virtual void callback(const T &msg) = 0;

    /** ROS subscriber to topic */
    ros::Subscriber subscriber_;
};

class JoystickReferenceVelocityGenerator final : public RosVelocityReferenceGenerator<sensor_msgs::Joy> {
   public:
    JoystickReferenceVelocityGenerator(ros::NodeHandle &nh, const std::string &topic, scalar_t rampedVelocity,
                                       size_t xIndex = 0, size_t yIndex = 1, size_t yawIndex = 2, scalar_t xScale = 1.0,
                                       scalar_t yScale = 1.0, scalar_t yawScale = 1.0)
        : RosVelocityReferenceGenerator<sensor_msgs::Joy>(nh, topic),
          rampedVelocity_(rampedVelocity),
          xIndex_(xIndex),
          yIndex_(yIndex),
          yawIndex_(yawIndex),
          xScale_(xScale),
          yScale_(yScale),
          yawScale_(yawScale) {}

    ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) override;

   private:
    scalar_t rampedVelocity_;
    ReferenceVelocity referenceDesired_;

    size_t xIndex_;
    size_t yIndex_;
    size_t yawIndex_;

    scalar_t xScale_;
    scalar_t yScale_;
    scalar_t yawScale_;

    void callback(const sensor_msgs::Joy &msg) override;
};

class TwistReferenceVelocityGenerator final : public RosVelocityReferenceGenerator<geometry_msgs::Twist> {
   public:
    TwistReferenceVelocityGenerator(ros::NodeHandle &nh, const std::string &topic)
        : RosVelocityReferenceGenerator<geometry_msgs::Twist>(nh, topic) {}

   private:
    void callback(const geometry_msgs::Twist &msg) override;
};

/**
 * @brief Get the ReferenceVelocityGenerator unique pointer, initialized with the parameters from a config file
 *
 * @param nh : ROS node handle
 * @return std::unique_ptr<ReferenceVelocityGenerator> : unique pointer to ReferenceVelocityGenerator
 */
std::unique_ptr<ReferenceVelocityGenerator> getReferenceVelocityGeneratorUnique(ros::NodeHandle &nh);

/**
 * @brief Get the ReferenceVelocityGenerator shared pointer, initialized with the parameters from a config file
 *
 * @param nh : ROS node handle
 * @return std::shared_ptr<ReferenceVelocityGenerator> : shared pointer to ReferenceVelocityGenerator
 */
std::shared_ptr<ReferenceVelocityGenerator> getReferenceVelocityGeneratorShared(ros::NodeHandle &nh);

}  // namespace reference
}  // namespace tbai
