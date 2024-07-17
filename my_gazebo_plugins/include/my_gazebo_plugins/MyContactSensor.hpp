#ifndef MYCONTACTSENSOR_HPP
#define MYCONTACTSENSOR_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace gazebo {
class MyContactSensor : public SensorPlugin {
public:
  MyContactSensor() : SensorPlugin(){};
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  void OnUpdate();

private:

};
} // namespace gazebo

#endif
