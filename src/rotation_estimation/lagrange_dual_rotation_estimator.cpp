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

#include "rotation_estimation/lagrange_dual_rotation_estimator.h"

#include <ceres/rotation.h>
#include <glog/logging.h>
#include <omp.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

#include "Spectra/MatOp/SparseSymMatProd.h"
#include "Spectra/SymEigsSolver.h"
#include "graph/graph.h"
#include "math/matrix_square_root.h"
#include "rotation_estimation/rotation_estimator.h"
#include "solver/bcm_sdp_solver.h"
#include "solver/rbr_sdp_solver.h"

namespace DAGSfM {
LagrangeDualRotationEstimator::LagrangeDualRotationEstimator(const int N,
                                                             const int dim)
    : LagrangeDualRotationEstimator(N, dim, solver::SolverOption(),
                                    solver::Summary()) {}

LagrangeDualRotationEstimator::LagrangeDualRotationEstimator(
    const int N, const int dim, const solver::SolverOption& option,
    const solver::Summary& summary)
    : option_(option), summary_(summary), images_num_(N), dim_(dim) {
  R_ = Eigen::SparseMatrix<double>(dim_ * N, dim_ * N);
  alpha_max_ = 0.0;
}

void LagrangeDualRotationEstimator::SetRAOption(
    const solver::SolverOption& option) {
  option_ = option;
}

void LagrangeDualRotationEstimator::SetRASummary(
    const solver::Summary& summary) {
  summary_ = summary;
}

solver::Summary LagrangeDualRotationEstimator::GetRASummary() const {
  return summary_;
}

double LagrangeDualRotationEstimator::GetErrorBound() const {
  return alpha_max_;
}

bool LagrangeDualRotationEstimator::EstimateRotations(
    const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  const int N = images_num_;

  CHECK_GT(view_pairs.size(), 0);
  CHECK_GT(N, 0);

  // The id of rotations is re-indexed for convenience of matrix manipulation.
  std::vector<image_t> vec_ids;
  for (auto rotation : *global_rotations) {
    vec_ids.push_back(rotation.first);
  }
  std::sort(vec_ids.begin(), vec_ids.end());
  for (uint i = 0; i < vec_ids.size(); i++) {
    view_id_to_index_[vec_ids[i]] = i;
  }

  // Set for R_
  std::unordered_map<size_t, std::vector<size_t>> adj_edges;
  FillinRelativeGraph(view_pairs, R_, adj_edges);

  std::unique_ptr<SDPSolver> solver = this->CreateSDPSolver(N, dim_);
  solver->SetCovariance(-R_);
  solver->SetAdjacentEdges(adj_edges);
  solver->Solve(option_, summary_);
  Y_ = solver->GetSolution();

  RetrieveRotations(Y_, global_rotations);

  LOG(INFO) << "Total time: " << summary_.TotalTime() << " milliseconds";

  return true;
}

void LagrangeDualRotationEstimator::ComputeErrorBound(
    const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs) {
  const int N = images_num_;

  std::vector<Eigen::Triplet<double>> a_triplets;
  // adjacency matrix
  Eigen::SparseMatrix<double> A(N, N);
  // degree matrix (a diagonal matrix)
  Eigen::SparseMatrix<double> D(N, N);
  std::vector<Eigen::Triplet<double>> d_triplets;
  std::vector<double> degrees(N, 0);

  for (auto& view_pair : view_pairs) {
    ImagePair pair = view_pair.first;
    const int i = view_id_to_index_[pair.first];
    const int j = view_id_to_index_[pair.second];
    degrees[i]++;
    degrees[j]++;
  }

  double max_degree = 0;
  for (auto& view_pair : view_pairs) {
    ImagePair pair = view_pair.first;
    const int i = view_id_to_index_[pair.first];
    const int j = view_id_to_index_[pair.second];

    a_triplets.push_back(Eigen::Triplet<double>(i, j, 1.0));
    a_triplets.push_back(Eigen::Triplet<double>(j, i, 1.0));

    d_triplets.push_back(Eigen::Triplet<double>(i, i, degrees[i]));
    d_triplets.push_back(Eigen::Triplet<double>(j, j, degrees[j]));
    max_degree = std::max(std::max(max_degree, degrees[i]), degrees[j]);
  }
  A.setFromTriplets(a_triplets.begin(), a_triplets.end());
  A.makeCompressed();
  D.setFromTriplets(d_triplets.begin(), d_triplets.end());
  D.makeCompressed();

  // laplacian matrix
  Eigen::SparseMatrix<double> L = D - A;

  // compute the bound of residual error
  Spectra::SparseSymMatProd<double> op(L);
  Spectra::SymEigsSolver<double, Spectra::SMALLEST_ALGE,
                         Spectra::SparseSymMatProd<double>>
      eigs(&op, 2, 5);
  eigs.init();
  eigs.compute();

  double lambda2 = 0.0;
  if (eigs.info() == Spectra::SUCCESSFUL) {
    lambda2 = eigs.eigenvalues()[0];
  } else {
    LOG(INFO) << "Computing Eigenvalue fails";
  }

  // get the second smallest eigen value
  alpha_max_ =
      2 * std::asin(std::sqrt(0.25 + lambda2 / (2.0 * max_degree)) - 0.5);
}

void LagrangeDualRotationEstimator::RetrieveRotations(
    const Eigen::MatrixXd& Y,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  for (auto orientation : *global_rotations) {
    image_t view_id = orientation.first;
    const int i = view_id_to_index_[view_id];
    // After fixing Equ.(10)
    Eigen::Matrix3d R = Y.block(0, 3 * i, 3, 3).transpose();
    if (R.determinant() < 0) R = -R;

    // CHECK_GE(R.determinant(), 0);
    // CHECK_NEAR(R.determinant(), 1, 1e-8);

    Eigen::Vector3d angle_axis;
    ceres::RotationMatrixToAngleAxis(R.data(), angle_axis.data());
    // LOG(INFO) << angle_axis.norm();
    (*global_rotations)[view_id] = angle_axis;
  }
}

void LagrangeDualRotationEstimator::FillinRelativeGraph(
    const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
    Eigen::SparseMatrix<double>& R,
    std::unordered_map<size_t, std::vector<size_t>>& adj_edges) {
  std::vector<Eigen::Triplet<double>> triplets;
  for (auto it = view_pairs.begin(); it != view_pairs.end(); ++it) {
    // image_t i = it->first.first, j = it->first.second;
    const int i = view_id_to_index_[it->first.first];
    const int j = view_id_to_index_[it->first.second];
    // CHECK_LT(i, j);
    Eigen::Matrix3d R_ij;
    ceres::AngleAxisToRotationMatrix(it->second.rotation_2.data(), R_ij.data());

    // After fixing Eq.(9)
    // R.block(3 * i, 3 * j, 3, 3) = R_ij.transpose();
    // R.block(3 * j, 3 * i, 3, 3) = R_ij;
    for (int l = 0; l < 3; l++) {
      for (int r = 0; r < 3; r++) {
        triplets.push_back(
            Eigen::Triplet<double>(dim_ * i + l, dim_ * j + r, R_ij(r, l)));
        triplets.push_back(
            Eigen::Triplet<double>(dim_ * j + l, dim_ * i + r, R_ij(l, r)));
      }
    }

    adj_edges[i].push_back(j);
    adj_edges[j].push_back(i);
  }
  R.setFromTriplets(triplets.begin(), triplets.end());
  R.makeCompressed();
}

std::unique_ptr<SDPSolver> LagrangeDualRotationEstimator::CreateSDPSolver(
    const int n, const int dim) {
  switch (option_.solver_type) {
    case solver::RBR_BCM:
      return std::unique_ptr<SDPSolver>(new RBRSDPSolver(n, dim));
      break;
    // case solver::RANK_DEFICIENT_BCM:
    //   return std::unique_ptr<SDPSolver>(new RankRestrictedSDPSolver(n, dim));
    //   break;
    // case solver::ADMM_SDP:
    //     return std::unique_ptr<AdmmSDPSolver>(new AdmmSDPSolver(n));
    //     break;
    // case solver::SDP_RELAXED_ADMM:
    //     return std::unique_ptr<SDPRelaxedAdmmSolver>(new
    //     SDPRelaxedAdmmSolver(n)); break;
    default:
      LOG(WARNING) << "Solve Type is not supported!";
      return nullptr;
      break;
  }
}

}  // namespace DAGSfM