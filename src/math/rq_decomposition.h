#ifndef GRAPHSFM_MATH_MATRIX_RQ_DECOMPOSITION_H_
#define GRAPHSFM_MATH_MATRIX_RQ_DECOMPOSITION_H_

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <glog/logging.h>

namespace GraphSfM {

// Implements the RQ decomposition that recovers matrices R and Q such that A =
// R * Q where R is an mxn upper-triangular matrix and Q is an nxn unitary
// matrix.
template <typename MatrixType>
class RQDecomposition {
 public:
  // Shorthand for transpose type.
  typedef Eigen::Matrix<typename MatrixType::Scalar,
                        MatrixType::ColsAtCompileTime,
                        MatrixType::RowsAtCompileTime,
                        (MatrixType::Flags & Eigen::RowMajorBit) ?
                        Eigen::RowMajor : Eigen::ColMajor,
                        MatrixType::MaxColsAtCompileTime,
                        MatrixType::MaxRowsAtCompileTime> MatrixTransposeType;

  typedef typename
  Eigen::HouseholderQR<MatrixTransposeType>::MatrixQType MatrixQType;

  // The matlab version of RQ decomposition is as follows:
  //
  // function [R Q] = rq(M)
  //   [Q,R] = qr(flipud(M)')
  //   R = flipud(R');
  //   R = fliplr(R);
  //   Q = Q';
  //   Q = flipud(Q);
  //
  // where flipup flips the matrix upside-down and fliplr flips the matrix from
  // left to right.
  explicit RQDecomposition(const MatrixType& matrix) {
    // flipud(M)' = fliplr(M').
    const MatrixTransposeType matrix_flipud_transpose =
        matrix.transpose().rowwise().reverse();

    Eigen::HouseholderQR<MatrixTransposeType> qr(matrix_flipud_transpose);
    const MatrixQType& q0 = qr.householderQ();
    const MatrixTransposeType& r0 = qr.matrixQR();

    // Flip upside down.
    matrix_r_ = r0.transpose();
    matrix_r_ = matrix_r_.colwise().reverse().eval();

    // Flip left right.
    matrix_r_ = matrix_r_.rowwise().reverse().eval();

    // When R is an mxn matrix and m <= n then it is upper triangular. If m > n
    // then all elements below the subdiagonal are 0.
    for (int i = 0; i < matrix_r_.rows(); ++i) {
      for (int j = 0;
           matrix_r_.cols() - j > matrix_r_.rows() - i && j < matrix_r_.cols();
           ++j) {
        matrix_r_(i, j) = 0;
      }
    }

    // Flip upside down.
    matrix_q_ = q0.transpose();
    matrix_q_ = matrix_q_.colwise().reverse().eval();

    // Since the RQ decomposition is not unique, we will simply require that
    // det(Q) = 1. This makes using RQ decomposition for decomposing the
    // projection matrix very simple.
    if (matrix_q_.determinant() < 0) {
      matrix_q_.row(1) *= -1.0;
      matrix_r_.col(1) *= -1.0;
    }
  }

  // Mimic Eigen's QR interface.
  const MatrixType& matrixR() const {
    return matrix_r_;
  }

  const MatrixQType& matrixQ() const {
    return matrix_q_;
  }

 private:
  MatrixType matrix_r_;
  MatrixQType matrix_q_;
};

}  // namespace GraphSfM

#endif  // GRAPHSFM_MATH_MATRIX_RQ_DECOMPOSITION_H_
