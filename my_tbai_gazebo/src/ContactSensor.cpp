#include "my_tbai_gazebo/ContactSensor.hpp"

#include <tbai_core/Types.hpp>
#include <tbai_core/config/YamlConfig.hpp>

namespace gazebo {
GZ_REGISTER_SENSOR_PLUGIN(ContactSensor)

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ContactSensor::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

    if (!this->parentSensor) {
        ROS_ERROR_STREAM("[ContactSensor] Could not load contact sensor plugin.");
        return;
    }

    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&ContactSensor::OnUpdate, this));
    this->parentSensor->SetActive(true);

    // create ROS publisher
    const std::string topicName = this->parentSensor->Name();
    contactPublisher_ = ros::NodeHandle().advertise<std_msgs::Bool>(topicName, 1);

    // Set initial state
    lastState_ = false;

    ROS_INFO_STREAM("[ContactSensor] Loading ContactSensor plugin. Publishing on topic /" << topicName);

    // Setup update rate
    auto updateRate = tbai::core::fromRosConfig<tbai::scalar_t>("contact_sensor/update_rate");
    this->parentSensor->SetUpdateRate(updateRate);

    ROS_INFO_STREAM("[ContactSensor] Loaded ContactSensor plugin. Update rate: " << updateRate << " Hz");
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ContactSensor::OnUpdate() {
    bool state = parentSensor->Contacts().contact_size() != 0;
    if (state == lastState_) {
        return;
    }

    // Contact flag changed, publish message
    contactMsg_.data = state;
    contactPublisher_.publish(contactMsg_);
    lastState_ = state;
}

};  // namespace gazebo
