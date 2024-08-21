cmake_minimum_required(VERSION 3.0.2)
project(tbai_mpc_perceptive)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tbai_core
    tbai_mpc
    tbai_static
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES  
 CATKIN_DEPENDS  
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(simple
    src/simple.cpp
)
target_link_libraries(simple
    ${catkin_LIBRARIES} yaml-cpp
)