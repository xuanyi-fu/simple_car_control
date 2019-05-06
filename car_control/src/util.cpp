//
// Created by xuanyi on 5/5/19.
//

#include "car_control/util.h"
#include "mat.h"
#include "iostream"
namespace util {
    Eigen::MatrixXd Matlab2Eigen(const std::string &file, const std::string &variable) {
        std::vector<double> v;

        MATFile *pmat = matOpen(file.c_str(), "r");
        if (pmat == nullptr) {
            std::cout << "Cannot open!";
            Eigen::MatrixXd params;
            params.resize(1, 1);
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
                v.assign(pr, pr + num);
            }
            Eigen::MatrixXd params = Eigen::Map<Eigen::MatrixXd>(v.data(), numRow, numCol);
            return params;
        }

// cleanup
        mxDestroyArray(arr);
        matClose(pmat);

        Eigen::MatrixXd params;
        params.resize(1, 1);
        params << 0;
        return params;
    }

    std::vector<double> Matlab2STDVector(const std::string &file, const std::string &variable) {
        std::vector<double> v;

        MATFile *pmat = matOpen(file.c_str(), "r");
        if (pmat == nullptr) {
            std::cout << "Cannot open!";
            return v;
        }

        mxArray *arr = matGetVariable(pmat, variable.c_str());
        if (arr != nullptr && mxIsDouble(arr) && !mxIsEmpty(arr)) {
            // copy data
            mwSize num = mxGetNumberOfElements(arr);
            double *pr = mxGetPr(arr);
            if (pr != nullptr) {
                v.reserve(num); //is faster than resize :-)
                v.assign(pr, pr + num);
            }
            return v;
        }

// cleanup
        mxDestroyArray(arr);
        matClose(pmat);
        return v;
    }

    int writeDoubleVector2Mat(const std::string &file, const std::string &variableName, const std::vector<double> &data,
                              int rows, int cols) {
        MATFile *pmat;
        mxArray *pa1;
        pmat = matOpen(file.c_str(), "w");

        if (pmat == nullptr) {
            printf("Error creating file %s\n", file.c_str());
            printf("(Do you have write permission in this directory?)\n");
            return (EXIT_FAILURE);
        }

        pa1 = mxCreateDoubleMatrix(rows, cols, mxREAL);
        if (pa1 == nullptr) {
            printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
            printf("Unable to create mxArray.\n");
            return (EXIT_FAILURE);
        }
        memcpy((void *) (mxGetPr(pa1)), (void *) data.data(), data.size() * sizeof(double));

        auto status = matPutVariable(pmat, variableName.c_str(), pa1);
        if (status != 0) {
            printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
            return (EXIT_FAILURE);
        }

        // cleanup
        mxDestroyArray(pa1);
        matClose(pmat);
    }
}