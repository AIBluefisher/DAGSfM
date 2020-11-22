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

#ifndef SOLVER_SDP_SOLVER_H
#define SOLVER_SDP_SOLVER_H

// #define EIGEN_USE_MKL_ALL

#include <glog/logging.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SparseCore>
#include <iostream>
#include <unordered_map>

#include "solver/solver_option.h"
#include "solver/summary.h"

namespace DAGSfM {

class SDPSolver {
 protected:
  // number of unknown blocks
  int n_;

  // the dimension of matrix sub-block
  const int dim_;

  // covariance matrix in SDP problem
  Eigen::SparseMatrix<double> Q_;

  std::unordered_map<size_t, std::vector<size_t>> adj_edges_;

 public:
  // constructor
  SDPSolver(const int n, const int block_dim) : n_(n), dim_(block_dim) {
    Q_ = Eigen::SparseMatrix<double>(dim_ * n, dim_ * n);
  }

  virtual ~SDPSolver() {}

  // covariance setter
  void SetCovariance(const Eigen::SparseMatrix<double>& Q) { Q_ = Q; }

  void SetAdjacentEdges(
      const std::unordered_map<size_t, std::vector<size_t>>& adj_edges) {
    adj_edges_ = adj_edges;
  }

  // Get the final optimal solution
  virtual Eigen::MatrixXd GetSolution() const = 0;

  // Kernel function to solve the SDP problem with BCM
  virtual void Solve(const solver::SolverOption& option,
                     solver::Summary& summary) = 0;
};

}  // namespace DAGSfM

#endif