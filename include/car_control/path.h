//
// Created by xuanyi on 5/1/19.
//

#ifndef CAR_CONTROL_PATH_H
#define CAR_CONTROL_PATH_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <map>
template <typename BaseFunction>
class path{
public:
    path()= default;
    Eigen::Matrix<double, 3, 2> operator()(double t){
        Eigen::Matrix<double, 3, 2> dStates = Eigen::Matrix<double, 3, 2>::Zero();
        auto PathIt    = mPath.lower_bound(t);
        if(PathIt != mPath.begin()){
            auto LastPathIt = --PathIt;
            PathIt++;
            t = t - LastPathIt->first;
        }
        dStates.block(0,0,3,1) = PathIt->second.first(t).block(0,0,3,1);
        dStates.block(0,1,3,1) = PathIt->second.second(t).block(0,0,3,1);
        return dStates;
    }
    template <typename T1, typename T2>
    void addPath(double endTime, const T1& param1, const T2& param2){
        auto xPath = BaseFunction();
        xPath.set(param1);
        auto yPath = BaseFunction();
        yPath.set(param2);
        mPath[endTime].first  = xPath;
        mPath[endTime].second = yPath;
    }

    void addPathFromParamMatrix(const std::vector<double>& endTimes, const Eigen::MatrixXd& XparamMat, const Eigen::MatrixXd& YparamMat ){

        if(endTimes.size() != XparamMat.cols() || endTimes.size() != YparamMat.cols() || XparamMat.cols() != YparamMat.cols()){
            ROS_ERROR_STREAM("in addPathFromParamMatrix, Wrong Matrix/Vector Sizes!");
        }

        auto matColIndex = 0;
        auto matRowNum = XparamMat.rows();
        for(auto endTime : endTimes){
            Eigen::Matrix<double, 1, Eigen::Dynamic> XparamVec;
            XparamVec.resize(1,matRowNum);
            XparamVec.block(0,0,1,4) = XparamMat.block(0,matColIndex,matRowNum,1).transpose();


            Eigen::Matrix<double, 1, Eigen::Dynamic> YparamVec;
            YparamVec.resize(1,matRowNum);
            YparamVec.block(0,0,1,4) = YparamMat.block(0,matColIndex,matRowNum,1).transpose();

            addPath(endTime,XparamVec,YparamVec);

            ++matColIndex;
        }
    }

    double getPathFinalEndTime(){
        return (--(mPath.cend()))->first;
    }
private:
    std::map<double, std::pair<BaseFunction, BaseFunction>> mPath;
};
#endif //CAR_CONTROL_PATH_H
