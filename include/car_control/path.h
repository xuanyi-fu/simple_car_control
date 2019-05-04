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
        auto Path    = mPath.lower_bound(t);
        dStates.block(0,0,3,1) = Path->second.first(t).block(0,0,3,1);
        dStates.block(0,1,3,1) = Path->second.second(t).block(0,0,3,1);
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
private:
    std::map<double, std::pair<BaseFunction, BaseFunction>> mPath;
};
#endif //CAR_CONTROL_PATH_H
