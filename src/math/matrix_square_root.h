#ifndef MATH_MATRIX_SQUARE_ROOT_H
#define MATH_MATRIX_SQUARE_ROOT_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Eigenvalues"
#include "glog/logging.h"

namespace GraphSfM {

Eigen::MatrixXd MatrixSquareRoot(const Eigen::MatrixXd& mat);

Eigen::MatrixXd MatrixSquareRootForSemidefinitePositiveMat(const Eigen::MatrixXd& mat);

} // namespace GraphSfM

#endif