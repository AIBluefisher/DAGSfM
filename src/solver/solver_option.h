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

#ifndef SOLVER_SOLVER_OPTION_H
#define SOLVER_SOLVER_OPTION_H

#include <iostream>

namespace DAGSfM {
namespace solver {

enum SDPSolverType { RBR_BCM, RANK_DEFICIENT_BCM };

struct SolverOption {
  // maximum iteration number
  size_t max_iterations = 1000;

  // tolerance for convergence
  double tolerance = 1e-8;

  bool log_to_terminal = true;

  // RANK_DEFICIENT_BCM is not public currently.
  SDPSolverType solver_type = RBR_BCM;

  SolverOption(size_t max_iter = 0, double tol = 1e-8, bool log = true) {
    max_iterations = max_iter;
    tolerance = tol;
    log_to_terminal = log;
    solver_type = RBR_BCM;
  }

  SolverOption(const SolverOption& option) {
    max_iterations = option.max_iterations;
    tolerance = option.tolerance;
    log_to_terminal = option.log_to_terminal;
    solver_type = option.solver_type;
  }
};

}  // namespace solver
}  // namespace DAGSfM

#endif