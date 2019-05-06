//
// Created by xuanyi on 5/5/19.
//

#ifndef CAR_CONTROL_UTIL_H
#define CAR_CONTROL_UTIL_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace util{
    Eigen::MatrixXd Matlab2Eigen(const std::string& file, const std::string& variable);

    std::vector<double> Matlab2STDVector(const std::string& file, const std::string& variable);

    int writeDoubleVector2Mat( const std::string& file
            , const std::string& variableName
            , const std::vector<double>& data
            , int rows, int cols);

}

#endif //CAR_CONTROL_UTIL_H
