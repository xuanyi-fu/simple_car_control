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

    auto multiPath = path<util::polynomial<3> >();
    Eigen::Matrix<double, 1, 4> A1 = Eigen::Vector4d(0.0100, -0.1500, 1.0000, 0).transpose();
    Eigen::Matrix<double, 1, 4> A2 = Eigen::Vector4d(0.0050, -0.0750, 0, 5.0000).transpose();
//    auto poly = util::polynomial<3>();
//    poly.set(A1);
//    std::cout<<poly(0.0);
    multiPath.addPath(10.0,A1,A2);
    ros::init(argc, argv, "car_control_node");
    auto controller = car_controller<path<util::polynomial<3> > >(multiPath);
    ros::spin();

//auto poly = util::polynomial<3>();
//poly.set(Eigen::Vector4d(2,1,1,1).transpose());
//
//std::cout<<poly(1)<<std::endl;


}

