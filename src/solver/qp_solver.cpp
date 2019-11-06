#include "solver/qp_solver.h"

#include <Eigen/Core>
#include <glog/logging.h>

#include <algorithm>
#include <limits>
#include <string>

#include "math/sparse_cholesky_llt.h"
#include "util/stringprintf.h"

namespace GraphSfM {

QPSolver::QPSolver(const Options& options,
                   const Eigen::SparseMatrix<double>& P,
                   const Eigen::VectorXd& q,
                   const double r)
    : options_(options), P_(P), q_(q), r_(r) {
  CHECK_EQ(P_.rows(), P_.cols()) << "P must be a symmetric matrix.";
  CHECK_EQ(P_.cols(), q_.size())
      << "The dimensions of P and q must be consistent.";

  // Set the lower and upper bounds to be negative and positive infinity (i.e
  // no bounds).
  lb_.setConstant(P_.cols(), -std::numeric_limits<double>::infinity());
  ub_.setConstant(P_.cols(), std::numeric_limits<double>::infinity());

  // Set up the linear solver to compute the cholesky decomposition of:
  //     P_ + rho * eye(N)
  Eigen::SparseMatrix<double> spd_mat(P_.rows(), P_.cols());
  spd_mat.setIdentity();
  spd_mat *= options_.rho;
  spd_mat += P_;

  linear_solver_.Compute(spd_mat);
  CHECK_EQ(linear_solver_.Info(), Eigen::Success);
}

void QPSolver::SetMaxIterations(const int max_iterations) {
  options_.max_num_iterations = max_iterations;
}

void QPSolver::SetUpperBound(const Eigen::VectorXd& ub) { ub_ = ub.array(); }

void QPSolver::SetLowerBound(const Eigen::VectorXd& lb) { lb_ = lb.array(); }

// Solve the quadratic program.
bool QPSolver::Solve(Eigen::VectorXd* solution) {
  // Ensure the bounds are valid. If there are any invalid bounds then the
  // difference between the bounds would be a negative value.
  int coeff_index = -1;
  if ((ub_ - lb_).minCoeff(&coeff_index) < 0) {
    LOG(WARNING) << "You specified invalid lower or upper bounds for the "
                    "problem. lower_bound[" << coeff_index
                 << "] = " << lb_[coeff_index] << " but upper_bound["
                 << coeff_index << "] = " << ub_[coeff_index];
    return false;
  }

  CHECK_NOTNULL(solution)->setZero(q_.size());
  Eigen::VectorXd& x = *solution;
  Eigen::VectorXd z(P_.rows()), u(P_.rows());
  z.setZero();
  u.setZero();

  Eigen::VectorXd z_old(z.size()), x_hat(P_.rows());

  // Precompute some convergence terms.
  const double primal_abs_tolerance_eps =
      std::sqrt(P_.rows()) * options_.absolute_tolerance;
  const double dual_abs_tolerance_eps =
      std::sqrt(P_.rows()) * options_.absolute_tolerance;
  VLOG(2) << "Iteration   Residual         R norm          S norm          "
             "Primal eps      Dual eps";
  const std::string row_format =
      "  % 4d      % 4.4e     % 4.4e     % 4.4e     % 4.4e     % 4.4e";

  // Run the iterations.
  for (int i = 0; i < options_.max_num_iterations; i++) {
    // Update x.
    x.noalias() = linear_solver_.Solve(options_.rho * (z - u) - q_);
    if (linear_solver_.Info() != Eigen::Success) {
      return false;
    }

    // Update x_hat.
    std::swap(z, z_old);
    x_hat.noalias() = options_.alpha * x + (1.0 - options_.alpha) * z_old;

    // Update z.
    z.noalias() = ub_.min(lb_.max((x_hat + u).array())).matrix();

    // Update u.
    u.noalias() += x_hat - z;

    // Compute the convergence terms.
    const double objval = 0.5 * x.dot(P_ * x) + q_.dot(x) + r_;
    const double r_norm = (x - z).norm();
    const double s_norm = (-options_.rho * (z - z_old)).norm();
    const double max_norm = std::max({x.norm(), z.norm()});
    const double primal_eps =
        primal_abs_tolerance_eps + options_.relative_tolerance * max_norm;
    const double dual_eps =
        dual_abs_tolerance_eps +
        options_.relative_tolerance * (options_.rho * u).norm();

    // Log the result to the screen.
    VLOG(2) << StringPrintf(row_format.c_str(), objval, i, r_norm, s_norm,
                            primal_eps, dual_eps);
    // Determine if the minimizer has converged.
    if (r_norm < primal_eps && s_norm < dual_eps) {
      break;
    }
  }
  return true;
}

}  // namespace GraphSfM
