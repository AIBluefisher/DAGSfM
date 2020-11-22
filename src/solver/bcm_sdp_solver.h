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

#ifndef SOLVER_BCM_SDP_SOLVER_H
#define SOLVER_BCM_SDP_SOLVER_H

#include <chrono>
#include <iomanip>
#include <iostream>

#include "eigen3/Eigen/Core"
#include "solver/sdp_solver.h"

namespace DAGSfM {

class BCMSDPSolver : public SDPSolver {
 public:
  // constructor
  BCMSDPSolver(const int n, const int block_dim) : SDPSolver(n, block_dim) {}

  // Kernel function to solve the SDP problem with BCM
  virtual void Solve(const solver::SolverOption& option,
                     solver::Summary& summary) = 0;

  virtual Eigen::MatrixXd GetSolution() const = 0;

 protected:
  // Judge if algorithm converges
  virtual bool IsConverge(const double prev_funv, const double cur_funv,
                          const double tolerance, double& error) const {
    error = std::abs(prev_funv - cur_funv) / std::max(std::abs(prev_funv), 1.0);
    return error <= tolerance;
  }

  // Compute function value
  virtual double EvaluateFuncVal() const = 0;

  // Log running information to terminal
  void LogToStd(const size_t& iter, const double& pre_funv,
                const double& cur_funv, const double& error,
                const double& time) {
    if (iter < 1) {
      std::cout << "\n"
                << std::setw(11) << std::setfill(' ') << "Iter "
                << std::setw(16) << std::setfill(' ') << "Prev "
                << std::setw(16) << std::setfill(' ') << "Cur " << std::setw(16)
                << std::setfill(' ') << "Error " << std::setw(16)
                << std::setfill(' ') << "Time";
    } else {
      std::cout << std::setw(8) << std::setfill(' ') << iter << std::setw(5)
                << std::setfill(' ') << " " << std::setw(14)
                << std::setfill(' ') << pre_funv << std::setw(5)
                << std::setfill(' ') << " " << std::setw(12)
                << std::setfill(' ') << cur_funv << std::setw(5)
                << std::setfill(' ') << " " << std::setw(12)
                << std::setfill(' ') << error << std::setw(2)
                << std::setfill(' ') << " " << std::setw(10)
                << std::setfill(' ') << time;
    }
    std::cout << "\n";
  }
};

}  // namespace DAGSfM

#endif
