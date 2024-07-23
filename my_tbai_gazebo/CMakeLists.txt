cmake_minimum_required(VERSION 3.0.2)
project(my_tbai_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#add_compile_options(-std=c++17)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  ocs2_robotic_tools
  pluginlib
  roscpp
  rospy
  std_msgs
  tbai_core
  my_tbai_msgs
  realtime_tools
)

find_package(gazebo REQUIRED)




catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES my_tbai_gazebo
  CATKIN_DEPENDS  tbai_core my_tbai_msgs
  DEPENDS pinocchio
)


include_directories(
   include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

## Joint controller plugin
add_library(JointController src/JointController.cpp)
target_link_libraries(JointController ${catkin_LIBRARIES}) 
target_compile_options(JointController PRIVATE ${OCS2_CXX_FLAGS})

## RBD state plugin
# Anymal state estimator plugin
add_library(StatePublisher src/StatePublisher.cpp)
target_link_libraries(StatePublisher ${gazebo_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(StatePublisher PRIVATE ${OCS2_CXX_FLAGS})

# Anymal contact sensor
add_library(ContactSensor SHARED src/ContactSensor.cpp)
target_link_libraries(ContactSensor ${catkin_LIBRARIES} ${gazebo_LIBRARIES})
target_compile_options(ContactSensor PRIVATE ${OCS2_CXX_FLAGS})


