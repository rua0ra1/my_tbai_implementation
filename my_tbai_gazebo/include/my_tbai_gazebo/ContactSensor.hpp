#ifndef CONTACTSENSOR_HPP
#define CONTACTSENSOR_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace gazebo {
class ContactSensor : public SensorPlugin {
   public:
    ContactSensor() : SensorPlugin() {}
    void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);
    void OnUpdate();

   private:
    sensors::ContactSensorPtr parentSensor;
    event::ConnectionPtr updateConnection;
    ros::Publisher contactPublisher_;
    std_msgs::Bool contactMsg_;
    bool lastState_;
};
}  // namespace gazebo



#endif