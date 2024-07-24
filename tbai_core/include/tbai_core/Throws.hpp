#pragma once

#define TBAI_STD_THROW(message)                                                                                    \
    do {                                                                                                           \
        std::cerr << "\n"                                                                                          \
                  << "Exception thrown in file " << __FILE__ << " at line " << __LINE__ << ": " << message << "\n" \
                  << std::endl;                                                                                    \
        throw std::runtime_error(message);                                                                         \
    } while (0)

#define TBAI_ROS_THROW(message)                                                                                  \
    do {                                                                                                         \
        ROS_ERROR_STREAM("Exception thrown in file " << __FILE__ << " at line " << __LINE__ << ": " << message); \
        throw std::runtime_error(message);                                                                       \
    } while (0)

#define TBAI_STD_THROW_IF(condition, message) \
    if (condition) {                          \
        TBAI_STD_THROW(message);              \
    }

#define TBAI_ROS_THROW_IF(condition, message) \
    if (condition) {                          \
        TBAI_ROS_THROW(message);              \
    }

#define TBAI_STD_THROW_UNLESS(condition, message) \
    if (!(condition)) {                           \
        TBAI_STD_THROW(message);                  \
    }

#define TBAI_ROS_THROW_UNLESS(condition, message) \
    if (!(condition)) {                           \
        TBAI_ROS_THROW(message);                  \
    }
