cmake_minimum_required(VERSION 3.0.2)
project(tbai_msgs)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
    FILES
    JointCommand.msg
    JointCommandArray.msg
    RbdState.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    DEPENDS message_runtime
)

include_directories(
)