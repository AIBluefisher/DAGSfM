// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

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

#ifndef SOLVER_ADMM_H
#define SOLVER_ADMM_H

#include <glog/logging.h>

#include <iomanip>
#include <iostream>

#include "solver/solver_option.h"
#include "solver/summary.h"

namespace DAGSfM {

// An abstract class for Alternating Direction Method of Multipliers (ADMM)
// Any class which want to implement a solver that based on ADMM should
// inherit this class

class ADMM {
  // public:
  // virtual void Solve(const solver::SolverOption& option, solver::Summary&
  // summary) = 0;
 public:
  virtual ~ADMM() { /* LOG(INFO) << "ADMM destructor"; */
  }

 protected:
  virtual bool IsConverge(const double tolerance, double& prev_delta,
                          double& primal_residual, double& dual_residual) = 0;

  virtual void SetPenaltyParameter() = 0;

  virtual void SetStepSize() = 0;

  virtual double ComputePrimalResidual() const = 0;

  virtual double ComputeDualResidual() const = 0;

  virtual void LogToStd(const size_t& iter, const double& prev_primal_residual,
                        const double& cur_primal_residual,
                        const double& prev_dual_residual,
                        const double& cur_dual_residual,
                        const double& time) const {
    if (iter < 1) {
      std::cout << "\n"
                << std::setw(11) << std::setfill(' ') << "Iter "
                << std::setw(16) << std::setfill(' ') << "PPR  "
                << std::setw(16) << std::setfill(' ') << "CPR  "
                << std::setw(16) << std::setfill(' ') << "PDR  "
                << std::setw(16) << std::setfill(' ') << "CDR  "
                << std::setw(16) << std::setfill(' ') << "Time ";
    } else {
      std::cout << std::setw(8) << std::setfill(' ') << iter << std::setw(5)
                << std::setfill(' ') << " " << std::setw(14)
                << std::setfill(' ') << prev_primal_residual << std::setw(5)
                << std::setfill(' ') << " " << std::setw(12)
                << std::setfill(' ') << cur_primal_residual << std::setw(5)
                << std::setfill(' ') << " " << std::setw(12)
                << std::setfill(' ') << prev_dual_residual << std::setw(5)
                << std::setfill(' ') << " " << std::setw(12)
                << std::setfill(' ') << cur_dual_residual << std::setw(2)
                << std::setfill(' ') << " " << std::setw(10)
                << std::setfill(' ') << time;
    }
    std::cout << "\n";
  }
};
}  // namespace DAGSfM

#endif