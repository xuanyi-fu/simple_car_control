//
// Created by xuanyi on 4/28/19.
//
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "car_control/path.h"
#include "car_control/car_controller.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_control_node");
    auto controller = car_controller(path());
    ros::spin();
}

