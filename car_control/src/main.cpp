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
#include "car_control/util.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_control_node");
    ros::NodeHandle nh;

    std::string inputFile     = "/home/xuanyi/params2.mat";
    std::string inputX        = "Axs";
    std::string inputY        = "Ays";
    std::string inputTime     = "Ts";
    std::string outputFile1   = "/home/xuanyi/result1.mat";
    std::string outputStates  = "States";

    nh.param("inputMatlabFile"   , inputFile     , inputFile   );
    nh.param("inputXVariableName", inputX        , inputX      );
    nh.param("inputYVariableName", inputY        , inputY      );
    nh.param("outputMatlabFile"  , outputFile1   , outputFile1 );
    nh.param("outputVariableName", outputStates  , outputStates);

    Eigen::MatrixXd Axs = util::Matlab2Eigen(inputFile,inputX);
    Eigen::MatrixXd Ays = util::Matlab2Eigen(inputFile,inputY);
    auto endTimes = util::Matlab2STDVector(inputFile,inputTime);

    auto multiPath = path<util::polynomial<3> >();
    multiPath.addPathFromParamMatrix(endTimes, Axs, Ays);

    auto controller = car_controller<path<util::polynomial<3> > >(nh,multiPath);

    ros::spin();

    util::writeDoubleVector2Mat(outputFile1,outputStates,controller.getTrueStates(),4,controller.getTrueStates().size()/4);
}

