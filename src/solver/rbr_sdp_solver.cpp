// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rbr_sdp_solver.h"

#include <ceres/rotation.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

#include "bcm_sdp_solver.h"
#include "eigen3/Eigen/Eigenvalues"
#include "eigen3/Eigen/QR"
#include "math/matrix_square_root.h"

namespace DAGSfM {

RBRSDPSolver::RBRSDPSolver(const int n, const int block_dim)
    : BCMSDPSolver(n, block_dim) {
  X_ = Eigen::MatrixXd::Identity(dim_ * n, dim_ * n);
}

void RBRSDPSolver::Solve(const solver::SolverOption& option,
                         solver::Summary& summary) {
  double prev_func_val = std::numeric_limits<double>::max();
  double cur_func_val = this->EvaluateFuncVal();
  double duration = 0.0;
  double error = 0.0;

  summary.begin_time = std::chrono::high_resolution_clock::now();
  while (summary.total_iterations_num < option.max_iterations) {
    if (option.log_to_terminal) {
      this->LogToStd(summary.total_iterations_num, prev_func_val, cur_func_val,
                     error, duration);
    }

    if (IsConverge(prev_func_val, cur_func_val, option.tolerance, error)) break;

    // convergence rate? Take it for consideration.
    for (int k = 0; k < n_; k++) {
      // Eliminating the k-th row and column from Y to form Bk
      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3 * (n_ - 1), 3 * (n_ - 1));
      this->ReformingB(k, B);

      // Eliminating the k-th column and all but the k-th row from R to form Wk
      Eigen::MatrixXd W = Eigen::MatrixXd::Zero(3 * (n_ - 1), 3);
      this->ReformingW(k, W);

      Eigen::MatrixXd B_multi_W = B * W;
      Eigen::MatrixXd WtBW = W.transpose() * B_multi_W;

      // FIXME: (chenyu) Solving matrix square root with
      // SVD and LDL^T would generate different result
      Eigen::MatrixXd WtBW_sqrt = MatrixSquareRoot(WtBW);
      // Eigen::MatrixXd WtBW_sqrt =
      // MatrixSquareRootForSemidefinitePositiveMat(WtBW);

      // FIXME: (chenyu) Eigen 3.3.0 is required for the use of
      // CompleteOrthogonalDecomposition<>
      // Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(WtBW_sqrt);
      // Eigen::Matrix3d moore_penrose_pseinv = cqr.pseudoInverse();
      Eigen::Matrix3d moore_penrose_pseinv = WtBW_sqrt.inverse();

      // compute S by fixing the error of Equ.(47) in Erikson's paper
      Eigen::MatrixXd S = -B_multi_W * moore_penrose_pseinv;

      // reordering X
      this->ReorderingUnknown(k, B, S);
    }

    summary.total_iterations_num++;
    duration = summary.Duration();

    // Update function value
    prev_func_val = cur_func_val;
    cur_func_val = this->EvaluateFuncVal();
  }

  summary.total_iterations_num++;
  this->LogToStd(summary.total_iterations_num, prev_func_val, cur_func_val,
                 error, duration);
}

double RBRSDPSolver::EvaluateFuncVal() const { return (Q_ * X_).trace(); }

void RBRSDPSolver::ReformingB(const int k, Eigen::MatrixXd& B) {
  int r = 0, c = 0;  // the row and column index of matrix B

  for (int i = 0; i < n_; i++) {
    if (i == k) continue;
    c = 0;
    for (int j = 0; j < n_; j++) {
      if (j == k) continue;

      B.block(3 * r, 3 * c, 3, 3) = X_.block(3 * i, 3 * j, 3, 3);
      c++;
    }
    r++;
  }
}

void RBRSDPSolver::ReformingW(const int k, Eigen::MatrixXd& W) {
  int r = 0;  // row index of matrix W

  for (int i = 0; i < n_; i++) {
    if (i == k) continue;

    W.block(3 * r, 0, 3, 3) = Q_.block(3 * i, 3 * k, 3, 3);
    r++;
  }
}

void RBRSDPSolver::ReorderingUnknown(const int k, const Eigen::MatrixXd& B,
                                     const Eigen::MatrixXd& S) {
  // Reordering X according to [Algorithm 1] in paper:
  // - Z. Wen, D. Goldfarb, S. Ma, and K. Scheinberg.
  //   Row by row methods for semidefinite programming.
  //   Technical report, Columbia University, 2009. 7

  // Update X(k, k)
  X_.block(3 * k, 3 * k, 3, 3) = Eigen::Matrix3d::Identity();

  // Update the k-th column of Y, except Y(k, k)
  int j = 0;  // the j-th block of S
  for (int i = 0; i < n_; i++) {
    if (i == k) continue;
    X_.block(3 * i, 3 * k, 3, 3) = S.block(3 * j, 0, 3, 3);
    // X_.block(3 * i, 3 * k, 3, 3) = S.block(3 * j, 0, 3, 3).transpose();
    j++;
  }

  // Update the k-th row of Y, except Y(k, k)
  int i = 0;  // the i-th block of S
  for (int j = 0; j < n_; j++) {
    if (j == k) continue;
    X_.block(3 * k, 3 * j, 3, 3) = S.block(3 * i, 0, 3, 3).transpose();
    // X_.block(3 * k, 3 * j, 3, 3) = S.block(3 * i, 0, 3, 3);
    i++;
  }
}

}  // namespace DAGSfM