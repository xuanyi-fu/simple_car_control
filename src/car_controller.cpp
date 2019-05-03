//
// Created by xuanyi on 5/1/19.
//
#include "car_control/car_controller.h"
#include "tf/transform_datatypes.h"
#include "sophus/so2.hpp"


car_controller::car_controller(const path& path)
        :mIsControlStart(false)
        ,mIsVelocityStable(false)
        ,mIfSetVelocityStableStartTime(false)
        ,mPath(path)
        ,mNodeHandle()
        ,mControlPub(mNodeHandle.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd",10))
        ,mStateSub(mNodeHandle.subscribe<nav_msgs::Odometry>("/odom", 1, &car_controller::controlCallback,this))
        ,mControlStartTime()
        ,mStableVelocityStartTime()
{}

inline double car_controller::getSecFromStart(const ros::Time& ros_time) noexcept {
    return (ros_time - mControlStartTime).toSec();
}

//xd for x desired
//x  for x true
void car_controller::controlCallback(const nav_msgs::OdometryConstPtr& nav_msg) {

    auto k0 = 0.1;
    auto k  = 0.1;
    auto controlMessage = ackermann_msgs::AckermannDriveStamped();
    auto x = Eigen::Vector4d(nav_msg->pose.pose.position.x
                            ,nav_msg->pose.pose.position.y
                            ,static_cast<double>(tf::getYaw(nav_msg->pose.pose.orientation))
                            ,nav_msg->twist.twist.linear.x);
    // [dx1;dx2]
    auto dx1x2 = Eigen::Vector2d(x(3)*cos(x(2)),x(3)*sin(x(2)));
    auto R = Sophus::SO2d(x(2));
    double t;


    if(!mIsVelocityStable){
        if(!mIfSetVelocityStableStartTime){
            mStableVelocityStartTime = nav_msg->header.stamp;
            mIfSetVelocityStableStartTime = true;
        }

        controlMessage.drive.speed = 1.;
        mControlPub.publish(controlMessage);

        if((nav_msg->header.stamp - mStableVelocityStartTime).toSec() > 1.0){
            mIsVelocityStable = true;
        }
        return;
    }

    if(!mIsControlStart){
        mControlStartTime = nav_msg->header.stamp;
        t = 0.0;
        mIsControlStart = true;
    }else{
        t = getSecFromStart(nav_msg->header.stamp);
    }

    //Stop the car immediately after the execution of the trajectory.

    if(t > 10.0){
        controlMessage.header.stamp = nav_msg->header.stamp;
        controlMessage.drive.speed = 0.;
        controlMessage.drive.acceleration = 0.;
        controlMessage.drive.steering_angle=0.;
        mControlPub.publish(controlMessage);
        return;
    }

    auto refStates = mPath(t);

    auto xd     = Eigen::Vector4d(refStates.block<4,1>(0,0));
    auto dxd    = Eigen::Vector4d(refStates.block<4,1>(0,1));
    auto d2xd   = Eigen::Vector4d(refStates.block<4,1>(0,2));


    auto e      = x.head<2>() - xd.head<2>();

    auto de     = dx1x2       - dxd.head<2>();

    auto z      = -dxd.head<2>() + k0*e + dx1x2;

    auto m      = Eigen::Matrix2d();
    m(0,0)      = 1.0;
    m(0,1)      = 0.0;
    m(1,0)      = 0.0;
    m(1,1)      = 1.0/(x(3)*x(3));


    Eigen::Vector2d rhs    = R.inverse().matrix()*m*(d2xd.head<2>() - e - k0*de - k*z);



    auto u1     = atan(rhs(1));
    auto u2     = rhs(0);
    ROS_INFO_STREAM("rhs : "<<rhs);
    ROS_INFO_STREAM("t : "<<t);
    ROS_INFO_STREAM("x(3) : "<<x(3));
    ROS_INFO_STREAM("e : "<<e);
    ROS_INFO_STREAM("-dxd.head<2>() : "<<-dxd.head<2>());
    ROS_INFO_STREAM("k0*e : "<<k0*e);
    ROS_INFO_STREAM("dx1x2 : "<<dx1x2);




    controlMessage.header.stamp = nav_msg->header.stamp;
    controlMessage.drive.acceleration = u2;
    controlMessage.drive.steering_angle = u1;

    mControlPub.publish(controlMessage);

}



