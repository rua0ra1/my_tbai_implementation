cmake_minimum_required(VERSION 3.0.2)
project(tbai_core)

## C++17 is a requirement
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  tf
  my_tbai_msgs
  std_msgs
  roscpp
  roslib
  ocs2_robotic_tools
)
find_package(yaml-cpp REQUIRED)

include(cmake/tbai_cm.cmake)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  ${PROJECT_NAME} ${PROJECT_NAME}_config 
  CATKIN_DEPENDS roscpp roslib my_tbai_msgs ocs2_robotic_tools
  DEPENDS yaml-cpp
  CFG_EXTRAS tbai_cm.cmake
)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/control/CentralController.cpp
  src/control/StateSubscriber.cpp
)

add_library(${PROJECT_NAME}_config
  src/config/YamlConfig.cpp
)
target_link_libraries(${PROJECT_NAME}_config
  ${YAML_CPP_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_config
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Testing
catkin_add_gtest(test_config
  test/testConfig.cpp
)
target_link_libraries(test_config
  ${PROJECT_NAME}
  ${PROJECT_NAME}_config
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
