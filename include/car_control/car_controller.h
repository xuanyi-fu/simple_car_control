//
// Created by xuanyi on 5/1/19.
//
#include "car_control/path.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"


#ifndef CAR_CONTROL_CAR_CONTROLLER_H
#define CAR_CONTROL_CAR_CONTROLLER_H

class car_controller{
private:
    bool                    mIsControlStart;
    bool                    mIsVelocityStable;
    bool                    mIfSetVelocityStableStartTime;
    path                    mPath;
    ros::NodeHandle         mNodeHandle;
    ros::Subscriber         mStateSub;
    ros::Publisher          mControlPub;
    ros::Time               mControlStartTime;
    ros::Time               mStableVelocityStartTime;

    inline double getSecFromStart(const ros::Time&) noexcept;
    void controlCallback(const nav_msgs::OdometryConstPtr&);
public:
    explicit car_controller(const path&);
};

#endif //CAR_CONTROL_CAR_CONTROLLER_H
