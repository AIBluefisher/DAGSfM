#ifndef GRAPHSFM_MATH_MATRIX_SPECTRA_LINEAR_OPERATOR_H_
#define GRAPHSFM_MATH_MATRIX_SPECTRA_LINEAR_OPERATOR_H_

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <glog/logging.h>

#include "math/matrix/sparse_cholesky_llt.h"

namespace GraphSfM {

// A sparse method for computing the shift and inverse linear operator. This
// method is intended for use with the Spectra library.
class SparseSymShiftSolveLLT {
 public:
  explicit SparseSymShiftSolveLLT(const Eigen::SparseMatrix<double>& mat)
      : mat_(mat) {
    CHECK_EQ(mat_.rows(), mat_.cols());
    linear_solver_.Compute(mat_);
    if (linear_solver_.Info() != Eigen::Success) {
      LOG(FATAL)
          << "Could not perform Cholesky decomposition on the matrix. Are "
             "you sure it is positive semi-definite?";
    }
  }

  int rows() { return mat_.rows(); }
  int cols() { return mat_.cols(); }
  void set_shift(double sigma) { sigma_ = sigma; }

  // Use LDLT to perform matrix inversion on the positive semidefinite matrix.
  void perform_op(double* x_in, double* y_out) {
    Eigen::Map<Eigen::VectorXd> x(x_in, mat_.rows());
    Eigen::Map<Eigen::VectorXd> y(y_out, mat_.cols());
    y = linear_solver_.Solve(x);
    if (linear_solver_.Info() != Eigen::Success) {
      LOG(FATAL)
          << "Could not perform Cholesky decomposition on the matrix. Are "
             "you sure it is positive semi-definite?";
    }
  }

  const Eigen::SparseMatrix<double>& mat_;
  SparseCholeskyLLt linear_solver_;
  double sigma_;
};

}  // namespace GraphSfM

#endif  // GRAPHSFM_MATH_MATRIX_SPECTRA_LINEAR_OPERATOR_H_
