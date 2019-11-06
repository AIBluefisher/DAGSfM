#ifndef GRAPHSFM_MATH_MATRIX_SPARSE_CHOLESKY_LLT_H_
#define GRAPHSFM_MATH_MATRIX_SPARSE_CHOLESKY_LLT_H_

#include <cholmod.h>
#include <Eigen/SparseCore>

// UF_long is deprecated but SuiteSparse_long is only available in
// newer versions of SuiteSparse. So for older versions of
// SuiteSparse, we define SuiteSparse_long to be the same as UF_long,
// which is what recent versions of SuiteSparse do anyways.
#ifndef SuiteSparse_long
#define SuiteSparse_long UF_long
#endif

namespace GraphSfM {

// A class for performing the choleksy decomposition of a sparse matrix using
// CHOLMOD from SuiteSparse. This allows us to utilize the supernodal algorithms
// which are not included with Eigen. CHOLMOD automatically determines if the
// simplicial or supernodal algorithm is the best choice. The interface is meant
// to mimic the Eigen linear solver interface except that it is not templated
// and requires sparse matrices.
//
// NOTE: The matrix mat should be a symmetric matrix.
class SparseCholeskyLLt {
 public:
  explicit SparseCholeskyLLt(const Eigen::SparseMatrix<double>& mat);
  SparseCholeskyLLt();
  ~SparseCholeskyLLt();

  // Perform symbolic analysis of the matrix. This is useful for analyzing
  // matrices with the same sparsity pattern when used in conjunction with
  // Factorize().
  void AnalyzePattern(const Eigen::SparseMatrix<double>& mat);

  // Perform numerical decomposition of the current matrix. If the matrix has
  // the same sparsity pattern as the previous decomposition then this method
  // may be used to efficiently decompose the matrix by avoiding symbolic
  // analysis.
  void Factorize(const Eigen::SparseMatrix<double>& mat);

  // Computes the Cholesky decomposition of mat. This is the same as calling
  // AnalyzePattern() followed by Factorize().
  void Compute(const Eigen::SparseMatrix<double>& mat);

  // Returns the current state of the decomposition. After each step users
  // should ensure that Info() returns Eigen::Success.
  Eigen::ComputationInfo Info();

  // Using the cholesky decomposition, solve for x that minimizes
  //    lhs * x = rhs
  // where lhs is the factorized matrix.
  Eigen::VectorXd Solve(const Eigen::VectorXd& rhs);

 private:
  cholmod_common cc_;
  cholmod_factor* cholmod_factor_;
  bool is_factorization_ok_, is_analysis_ok_;
  Eigen::ComputationInfo info_;
};

}  // namespace GraphSfM

#endif  // GRAPHSFM_MATH_MATRIX_SPARSE_CHOLESKY_LLT_H_
