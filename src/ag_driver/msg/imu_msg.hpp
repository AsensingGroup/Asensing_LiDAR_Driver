
#pragma once

#ifdef ROS2_FOUND
#include <sensor_msgs/msg/imu.hpp>
typedef sensor_msgs::msg::Imu::SharedPtr AGImuMsgPtr;
#endif

#ifdef ROS1_FOUND
#include <sensor_msgs/Imu.h>
typedef sensor_msgs::ImuConstPtr AGImuMsgPtr;
#endif

