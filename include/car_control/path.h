//
// Created by xuanyi on 5/1/19.
//

#ifndef CAR_CONTROL_PATH_H
#define CAR_CONTROL_PATH_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

class path{
public:
    path()= default;
    Eigen::Matrix<double, 4, 3> operator()(double t);
private:
};

Eigen::Matrix<double, 4, 3> path::operator()(double t) {
    auto raw_path = std::vector<double>(12,0);
    raw_path[0] = t-(t*t)*(3.0/2.0E+1)+(t*t*t)/1.0E+2;
    raw_path[1] = (t*t)*(-3.0/4.0E+1)+(t*t*t)/2.0E+2+5.0;
    raw_path[2] = -atan((t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2))/(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0));
    raw_path[3] = sqrt(pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)+pow(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2),2.0));
    raw_path[4] = t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0;
    raw_path[5] = t*(-3.0/2.0E+1)+(t*t)*(3.0/2.0E+2);
    raw_path[6] = ((t*(3.0/1.0E+2)-3.0/2.0E+1)/(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0)+(t*(3.0/5.0E+1)-3.0/1.0E+1)*(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2))*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0))/(pow(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2),2.0)*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)+1.0);
    raw_path[7] = (((t*(3.0/5.0E+1)-3.0/1.0E+1)*(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0)*2.0-(t*(3.0/1.0E+2)-3.0/2.0E+1)*(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2))*2.0)*1.0/sqrt(pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)+pow(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2),2.0)))/2.0;
    raw_path[8] = t*(3.0/5.0E+1)-3.0/1.0E+1;
    raw_path[9] = t*(3.0/1.0E+2)-3.0/2.0E+1;
    raw_path[10] = ((t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2))*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)*(3.0/5.0E+1)+(3.0/1.0E+2)/(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0)-pow(t*(3.0/5.0E+1)-3.0/1.0E+1,2.0)*(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2))*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,3.0)*2.0-(t*(3.0/5.0E+1)-3.0/1.0E+1)*(t*(3.0/1.0E+2)-3.0/2.0E+1)*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)*2.0)/(pow(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2),2.0)*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)+1.0)+((t*(3.0/5.0E+1)-3.0/1.0E+1)*pow(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2),2.0)*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,3.0)*2.0+(t*(3.0/1.0E+2)-3.0/2.0E+1)*(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2))*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)*2.0)*1.0/pow(pow(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2),2.0)*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)+1.0,2.0)*((t*(3.0/1.0E+2)-3.0/2.0E+1)/(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0)+(t*(3.0/5.0E+1)-3.0/1.0E+1)*(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2))*1.0/pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0));
    raw_path[11] = pow((t*(3.0/5.0E+1)-3.0/1.0E+1)*(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0)*2.0-(t*(3.0/1.0E+2)-3.0/2.0E+1)*(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2))*2.0,2.0)*1.0/pow(pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)+pow(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2),2.0),3.0/2.0)*(-1.0/4.0)+(1.0/sqrt(pow(t*(-3.0/1.0E+1)+(t*t)*(3.0/1.0E+2)+1.0,2.0)+pow(t*(3.0/2.0E+1)-(t*t)*(3.0/2.0E+2),2.0))*(t*(-9.0/2.0E+2)+pow(t*(3.0/5.0E+1)-3.0/1.0E+1,2.0)*2.0+pow(t*(3.0/1.0E+2)-3.0/2.0E+1,2.0)*2.0+(t*t)*4.5E-3+3.0/2.5E+1))/2.0;
    auto path     = Eigen::Map<Eigen::Matrix<double,4,3>>(raw_path.data());
    return path;
}

#endif //CAR_CONTROL_PATH_H
