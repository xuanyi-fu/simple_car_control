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

#include "mat.h"

Eigen::MatrixXd Matlab2Eigen(const std::string& file, const std::string& variable){
    std::vector<double> v;

    MATFile* pmat = matOpen(file.c_str(), "r");
    if(pmat == nullptr){
        std::cout<<"Cannot open!";
        Eigen::MatrixXd params;
        params.resize(1,1);
        params << 0;
        return params;
    }

    mxArray *arr = matGetVariable(pmat, variable.c_str());
    if (arr != nullptr && mxIsDouble(arr) && !mxIsEmpty(arr)) {
        // copy data
        mwSize num = mxGetNumberOfElements(arr);
        mwSize numRow = mxGetM(arr);
        mwSize numCol = mxGetN(arr);
        double *pr = mxGetPr(arr);
        if (pr != nullptr) {
            v.reserve(num); //is faster than resize :-)
            v.assign(pr, pr+num);
        }
        Eigen::MatrixXd params = Eigen::Map<Eigen::MatrixXd>(v.data(),numRow,numCol);
        return params;
    }

// cleanup
    mxDestroyArray(arr);
    matClose(pmat);

    Eigen::MatrixXd params;
    params.resize(1,1);
    params << 0;
    return params;
}

std::vector<double> Matlab2STDVector(const std::string& file, const std::string& variable){
    std::vector<double> v;

    MATFile* pmat = matOpen(file.c_str(), "r");
    if(pmat == nullptr){
        std::cout<<"Cannot open!";
        return v;
    }

    mxArray *arr = matGetVariable(pmat, variable.c_str());
    if (arr != nullptr && mxIsDouble(arr) && !mxIsEmpty(arr)) {
        // copy data
        mwSize num = mxGetNumberOfElements(arr);
        double *pr = mxGetPr(arr);
        if (pr != nullptr) {
            v.reserve(num); //is faster than resize :-)
            v.assign(pr, pr+num);
        }
        return v;
    }

// cleanup
    mxDestroyArray(arr);
    matClose(pmat);
    return v;
}


int main(int argc, char** argv)
{

    Eigen::MatrixXd Axs = Matlab2Eigen("/home/xuanyi/params2.mat","Axs");
    Eigen::MatrixXd Ays = Matlab2Eigen("/home/xuanyi/params2.mat","Ays");
    auto endTimes = Matlab2STDVector("/home/xuanyi/params2.mat","Ts");

    auto multiPath = path<util::polynomial<3> >();
    multiPath.addPathFromParamMatrix(endTimes, Axs, Ays);
    ros::init(argc, argv, "car_control_node");
    auto controller = car_controller<path<util::polynomial<3> > >(multiPath);
    ros::spin();

//auto poly = util::polynomial<3>();
//poly.set(Eigen::Vector4d(2,1,1,1).transpose());
//
//std::cout<<poly(1)<<std::endl;



}

