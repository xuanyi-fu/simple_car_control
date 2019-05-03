//
// Created by xuanyi on 4/28/19.
//
#include <car_control/polynomial.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "car_control/path.h"
#include "car_control/car_controller.h"
#include "car_control/polynomial.h"


int main(int argc, char** argv)
{
//    ros::init(argc, argv, "car_control_node");
//    auto controller = car_controller(path());
//    ros::spin();

auto poly = util::polynomial<3>(Eigen::Vector4d(2,1,1,1).transpose());

std::cout<<poly(1)<<std::endl;
}

