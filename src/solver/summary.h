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

#ifndef SOLVER_SUMMARY_H
#define SOLVER_SUMMARY_H

#include <chrono>
#include <iostream>

namespace DAGSfM {
namespace solver {

struct Summary {
  unsigned total_iterations_num;

  std::chrono::high_resolution_clock::time_point begin_time;
  std::chrono::high_resolution_clock::time_point end_time;
  std::chrono::high_resolution_clock::time_point prev_time;

  Summary() { total_iterations_num = 0; }

  Summary(const Summary& summary) {
    total_iterations_num = summary.total_iterations_num;
    begin_time = summary.begin_time;
    end_time = summary.end_time;
  }

  double TotalTime() {
    return std::chrono::duration_cast<std::chrono::duration<int, std::milli>>(
               end_time - begin_time)
        .count();
  }

  double Duration() {
    if (total_iterations_num == 1) {
      prev_time = begin_time;
    } else {
      prev_time = end_time;
    }
    end_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<int, std::milli>>(
               end_time - prev_time)
        .count();
  }

  void Report() {
    std::cout << "\nLagrange Dual Rotation Averaging Report: \n";
    std::cout << "Total iterations: " << total_iterations_num << std::endl;
    std::cout << "Time took: " << TotalTime() << " milliseconds\n";
  }
};
}  // namespace solver
}  // namespace DAGSfM

#endif