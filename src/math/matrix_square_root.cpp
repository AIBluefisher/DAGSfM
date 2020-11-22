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

#include "math/matrix_square_root.h"

#include "eigen3/Eigen/Cholesky"

namespace DAGSfM {
Eigen::MatrixXd MatrixSquareRoot(const Eigen::MatrixXd& mat) {
  // only square matrix can apply matrix square root
  CHECK_EQ(mat.rows(), mat.cols());

  Eigen::EigenSolver<Eigen::MatrixXd> solver(mat);
  Eigen::MatrixXd V = solver.eigenvectors().real();
  Eigen::VectorXd Dv = solver.eigenvalues().real();
  Eigen::MatrixXd sqrt_D = Dv.cwiseSqrt().asDiagonal();

  return V * sqrt_D * V.inverse();
}

Eigen::MatrixXd MatrixSquareRootForSemidefinitePositiveMat(
    const Eigen::MatrixXd& mat) {
  // only square matrix can apply matrix square root
  CHECK_EQ(mat.rows(), mat.cols());

  Eigen::MatrixXd result;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(mat);

  result = ldlt.matrixL();
  result = ldlt.transpositionsP().transpose() * result;
  result *= ldlt.vectorD().array().sqrt().matrix().asDiagonal();

  return result;
}

}  // namespace DAGSfM