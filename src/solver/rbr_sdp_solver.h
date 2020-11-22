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

#ifndef SOLVER_RBR_SDP_SOLVER_H
#define SOLVER_RBR_SDP_SOLVER_H

#include <chrono>
#include <iostream>

#include "eigen3/Eigen/Core"
#include "solver/bcm_sdp_solver.h"
#include "solver/sdp_solver.h"

namespace DAGSfM {
// Semidefinite positive solver with Block Coordinate Minimization(BCM)
// This algorithm is a generalization of the RBR approach of the paper:
//   Z. Wen, D. Goldfarb, S. Ma, and K. Scheinberg. Row by row methods for
//   semidefinite programming. Technical report, Columbia University, 2009. 7
// As introduced in Eriksson's paper
//   Eriksson A, Olsson C, Kahl F, et al. Rotation averaging and strong
//   duality[C]//Proceedings of the IEEE Conference on Computer Vision and
//   Pattern Recognition. 2018: 127-135.
// This algorithm actually solves the SDP problem:
// -----------------------------------------------------------------------------------------
//                                  min. -tr(RY)
//                                  s.t. Y_{ii} = I_3, i=1, ..., n,
//                                       Y \succeq 0
// -----------------------------------------------------------------------------------------
// and the optimal solution could be retrieved by reading the first three rows
// of Y^*
//
class RBRSDPSolver : public BCMSDPSolver {
 private:
  Eigen::MatrixXd X_;

 public:
  // constructor
  RBRSDPSolver(const int n, const int block_dim);

  // Kernel function to solve the SDP problem with BCM
  virtual void Solve(const solver::SolverOption& option,
                     solver::Summary& summary) override;

  virtual Eigen::MatrixXd GetSolution() const override { return X_; }

 protected:
  // Compute function value
  virtual double EvaluateFuncVal() const;

 private:
  void ReformingB(const int k, Eigen::MatrixXd& Bk);
  void ReformingW(const int k, Eigen::MatrixXd& Wk);
  void ReorderingUnknown(const int k, const Eigen::MatrixXd& B,
                         const Eigen::MatrixXd& W);
};

}  // namespace DAGSfM

#endif
