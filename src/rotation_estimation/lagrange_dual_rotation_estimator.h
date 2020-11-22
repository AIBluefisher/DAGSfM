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

#ifndef SRC_ROTATION_ESTIMATION_LAGRANGE_DUAL_ROTATION_AVERAGING_H_
#define SRC_ROTATION_ESTIMATION_LAGRANGE_DUAL_ROTATION_AVERAGING_H_

// #define EIGEN_USE_MKL_ALL

#include <glog/logging.h>

#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "graph/graph.h"
#include "rotation_estimation/rotation_estimator.h"
#include "sfm/twoview_info.h"
#include "solver/sdp_solver.h"
#include "util/hash.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

struct CommunityEdge : graph::Edge {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

  CommunityEdge(size_t i, size_t j, float w, const Eigen::Matrix3d& Rot)
      : Edge(i, j, w) {
    R = Rot;
  }
};

// Implementation of CVPR 2018 && PAMI 2019 paper:
// - Erikson. et.al. Rotation Averaging with strong duality.
// Given the pairwise relative rotations, retrive the globally optimal value
// of absolute orientations.
class LagrangeDualRotationEstimator : public RotationEstimator {
 private:
  solver::SolverOption option_;

  solver::Summary summary_;

  // number of images/frames
  int images_num_;

  int dim_;

  // the compact matrix representation in Equ.(9) of Eriksson's paper
  Eigen::SparseMatrix<double> R_;

  // the optimized variable of (DD) problem
  Eigen::MatrixXd Y_;

  // upper bound for strong duality hold
  double alpha_max_;

  // this hash table is used for non-continuous index, such as
  // unordered internet datasets that composed of many unconnected components
  std::unordered_map<image_t, int> view_id_to_index_;

 public:
  LagrangeDualRotationEstimator(const int N, const int dim);
  LagrangeDualRotationEstimator(const int N, const int dim,
                                const solver::SolverOption& option,
                                const solver::Summary& summary);

  void SetRAOption(const solver::SolverOption& option);

  void SetRASummary(const solver::Summary& summary);
  solver::Summary GetRASummary() const;

  double GetErrorBound() const;

  // Estimate the absolute rotations, given pairs of relative rotations
  bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  // Compute the upper bound of angular error alpha_max_
  // If for all |alpha_{ij}| < alpha_max_, the strong duality hold
  void ComputeErrorBound(
      const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs);

 private:
  // Retrieve optimal solutions from matrix Y_
  void RetrieveRotations(
      const Eigen::MatrixXd& Y,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  void FillinRelativeGraph(
      const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
      Eigen::SparseMatrix<double>& R,
      std::unordered_map<size_t, std::vector<size_t>>& adj_edges);

  std::unique_ptr<SDPSolver> CreateSDPSolver(const int n, const int dim);
};

}  // namespace DAGSfM

#endif