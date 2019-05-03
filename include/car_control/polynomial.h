//
// Created by xuanyi on 5/3/19.
//
#ifndef CAR_CONTROL_POLYN_H
#define CAR_CONTROL_POLYN_H

#include <Eigen/Core>
#include <Eigen/Dense>
namespace util {
    template<int order>
    Eigen::Matrix<double, 1, order> getDCoefficient(const Eigen::Matrix<double, 1, order + 1>& coeff){

        auto Dcoeff  = Eigen::Matrix<double, 1, order>();
        int f = order;

        for(std::int32_t i = 0; i < order; i++ ){
            Dcoeff(0,i) = (f--)*coeff(0,i);
        }

        return Dcoeff;

    }

    template<int order>
    class polynomial {
    public:
        explicit polynomial(const Eigen::Matrix<double, 1, order + 1> &coefficient)
                : mCoefficient(Eigen::Matrix<double, 3, order + 1>::Zero())
        {
            mCoefficient.block(0,0,1,order + 1)     = coefficient.block(0,0,1,order + 1);
            mCoefficient.block(1,0,1,order    )     = getDCoefficient<order>(coefficient).block(0,0,1,order);
            mCoefficient.block(2,0,1,order - 1)     = getDCoefficient<order - 1>(getDCoefficient<order>(coefficient)).block(0,0,1,order - 1);
        }

        Eigen::Matrix<double, 3, 1> operator()(double t) {
            auto baseVector = std::vector<double>(order + 1, 0.);
            int power = order;
            for (auto &num:baseVector) {
                num = std::pow(t, power--);
            }

            auto base = Eigen::Map<Eigen::Matrix<double, order + 1, 1> >(baseVector.data());
            return mCoefficient * base;
        }

    private:
        Eigen::Matrix<double, 3, order + 1> mCoefficient;
    };


}

#endif //CAR_CONTROL_POLYN_H