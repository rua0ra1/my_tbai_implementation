#include "my_gazebo_plugins/MyContactSensor.hpp"
namespace gazebo{
GZ_REGISTER_SENSOR_PLUGIN(MyContactSensor)


/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/

void MyContactSensor::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

    if (!this->parentSensor) {
        ROS_ERROR_STREAM("[ContactSensor] Could not load contact sensor plugin.");
        return;
    }

    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&MyContactSensor::OnUpdate, this));
    this->parentSensor->SetActive(true);

    // create ROS publisher
    const std::string topicName = this->parentSensor->Name();
    contactPublisher_ = ros::NodeHandle().advertise<std_msgs::Bool>(topicName, 1);

    // Set initial state
    lastState_ = false;

    ROS_INFO_STREAM("[ContactSensor] Loading ContactSensor plugin. Publishing on topic /" << topicName);

    // Setup update rate
    auto updateRate = 100;
    this->parentSensor->SetUpdateRate(updateRate);

    ROS_INFO_STREAM("[ContactSensor] Loaded ContactSensor plugin. Update rate: " << updateRate << " Hz");
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/

void MyContactSensor::OnUpdate() {
    bool state = parentSensor->Contacts().contact_size()!=0;
    // if state didn't change use
    if(state==lastState_){
        return;
    }

    //contact flag changed,publish message
    contactMsg_.data=state;
    contactPublisher_.publish(contactMsg_);
    lastState_=state;
}


}


