#include "math/sparse_matrix.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace GraphSfM {

SparseMatrix::SparseMatrix(const Eigen::SparseMatrix<double>& matrix)
    : matrix_(matrix) {}
SparseMatrix::~SparseMatrix() {}

// y = y + Ax;
void SparseMatrix::RightMultiply(const Eigen::VectorXd& x,
                                 Eigen::VectorXd* y) const {
  y->noalias() += matrix_ * x;
}
// y = y + A'x;
void SparseMatrix::LeftMultiply(const Eigen::VectorXd& x,
                                Eigen::VectorXd* y) const {
  y->noalias() += matrix_.transpose() * x;
}

int SparseMatrix::num_rows() const { return matrix_.rows(); };
int SparseMatrix::num_cols() const { return matrix_.cols(); };

}  // namespace GraphSfM
