cmake_minimum_required(VERSION 3.0.2)
project(tbai_reference)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    tbai_core
)

find_package(yaml-cpp REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tbai_reference 
 CATKIN_DEPENDS  
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(tbai_reference
    src/ReferenceVelocityGenerator.cpp
)
target_link_libraries(tbai_reference ${catkin_LIBRARIES})

install(TARGETS tbai_reference
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)