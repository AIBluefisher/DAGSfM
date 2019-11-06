#ifndef GRAPHSFM_MATH_MATRIX_SPARSE_MATRIX_H_
#define GRAPHSFM_MATH_MATRIX_SPARSE_MATRIX_H_

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "math/linear_operator.h"

namespace GraphSfM {

// A wrapper for the Eigen sparse matrix class.
class SparseMatrix : public LinearOperator {
 public:
  explicit SparseMatrix(const Eigen::SparseMatrix<double>& matrix);
  ~SparseMatrix();

  // y = y + Ax;
  void RightMultiply(const Eigen::VectorXd& x,
                     Eigen::VectorXd* y) const override;

  // y = y + A'x;
  void LeftMultiply(const Eigen::VectorXd& x,
                    Eigen::VectorXd* y) const override;

  int num_rows() const override;
  int num_cols() const override;

 private:
  const Eigen::SparseMatrix<double>& matrix_;
};  // namespace GraphSfM

}  // namespace GraphSfM

#endif  // GRAPHSFM_MATH_MATRIX_SPARSE_MATRIX_H_
