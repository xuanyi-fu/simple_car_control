//
// Created by xuanyi on 5/1/19.
//
#include "car_control/path.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "sophus/so2.hpp"
#include "tf/transform_datatypes.h"


#ifndef CAR_CONTROL_CAR_CONTROLLER_H
#define CAR_CONTROL_CAR_CONTROLLER_H
template<typename PATH_T>
class car_controller{
private:
    bool                    mIsControlStart;
    bool                    mIsVelocityStable;
    bool                    mIfSetVelocityStableStartTime;
    PATH_T                  mPath;
    ros::NodeHandle         mNodeHandle;
    ros::Subscriber         mStateSub;
    ros::Publisher          mControlPub;
    ros::Time               mControlStartTime;
    ros::Time               mStableVelocityStartTime;

    inline double getSecFromStart(const ros::Time& ros_time) noexcept {
        return (ros_time - mControlStartTime).toSec();
    }
    void controlCallback(const nav_msgs::OdometryConstPtr& nav_msg) {

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

        auto refStates = mPath(t).transpose();

        auto xd     = Eigen::Vector2d(refStates.block(0,0,2,1));
        auto dxd    = Eigen::Vector2d(refStates.block(0,1,2,1));
        auto d2xd   = Eigen::Vector2d(refStates.block(0,2,2,1));


        auto e      = x.head<2>() - xd;

        auto de     = dx1x2       - dxd;

        auto z      = -dxd + k0*e + dx1x2;

        auto m      = Eigen::Matrix2d();
        m(0,0)      = 1.0;
        m(0,1)      = 0.0;
        m(1,0)      = 0.0;
        m(1,1)      = 1.0/(x(3)*x(3));

        Eigen::Vector2d rhs    = R.inverse().matrix()*m*(d2xd - e - k0*de - k*z);

        auto u1     = atan(rhs(1));
        auto u2     = rhs(0);

        controlMessage.header.stamp = nav_msg->header.stamp;
        controlMessage.drive.acceleration = u2;
        controlMessage.drive.steering_angle = u1;

        mControlPub.publish(controlMessage);

    }

public:
    explicit car_controller(const PATH_T& path)
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



//xd for x desired
//x  for x true

};
#endif //CAR_CONTROL_CAR_CONTROLLER_H
