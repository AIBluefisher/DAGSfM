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

#include "clustering/spectral_cluster.h"

#include <Spectra/SymEigsSolver.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>

#include "clustering/kmeans.h"
// #include <Spectra/GenEigsSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>
// #include <Spectra/MatOp/SparseGenMatProd.h>
#include <glog/logging.h>

#include "util/timer.h"

namespace DAGSfM {

std::unordered_map<int, int> SpectralCluster::ComputeCluster(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights, const int num_partitions) {
  if (num_partitions == 1) {
    for (auto node : nodes_) {
      labels_[node] = 0;
    }
    return labels_;
  }

  colmap::Timer timer;
  std::vector<Eigen::Triplet<double>> s_triplets;
  std::unordered_map<int, int> degrees;
  const int k = num_partitions;
  cluster_num_ = num_partitions;

  // 1. Compute similarity graph.
  for (uint i = 0; i < nodes_.size(); i++) {
    node_mapper_[nodes_[i]] = i;
  }

  timer.Start();
  const int N = nodes_.size();
  Eigen::SparseMatrix<double> S(N, N);
  for (uint i = 0; i < edges.size(); i++) {
    int src = node_mapper_[edges[i].first], dst = node_mapper_[edges[i].second];
    s_triplets.push_back(Eigen::Triplet<double>(src, dst, weights[i]));
    s_triplets.push_back(Eigen::Triplet<double>(dst, src, weights[i]));
    degrees[src] += 1;
    degrees[dst] += 1;
  }

  S.setFromTriplets(s_triplets.begin(), s_triplets.end());
  S.makeCompressed();
  timer.Pause();
  LOG(INFO) << "1. Similarity Graph Computation Time: "
            << timer.ElapsedSeconds();

  // 2. Compute Laplacian matrix.
  timer.Start();
  Eigen::SparseMatrix<double> L = ComputeLaplacian(S, degrees);
  L.makeCompressed();
  timer.Pause();
  LOG(INFO) << "2. Laplacian Matrix Computation Time: "
            << timer.ElapsedSeconds();

  // 3. Compute the top-k smallest eigen values and corresponding eigen vectors.
  timer.Start();
  Spectra::SparseSymMatProd<double> op(L);
  Spectra::SymEigsSolver<double, Spectra::SMALLEST_ALGE,
                         Spectra::SparseSymMatProd<double>>
      eigs(&op, k, std::min(2 * k, N));
  eigs.init();
  int nconv = eigs.compute();

  Eigen::VectorXd eigen_values;
  Eigen::MatrixXd eigen_vectors;
  if (eigs.info() == Spectra::SUCCESSFUL) {
    eigen_values = eigs.eigenvalues();
    eigen_vectors = eigs.eigenvectors();
  }

  timer.Pause();
  LOG(INFO) << "3. EigenValue Computation Time: " << timer.ElapsedSeconds();

  // 4. Reverse original eigen vectors(as it stored in descending order)
  timer.Start();
  uint i = 0, j = k - 1;
  while (i < j) {
    const Eigen::VectorXd tmp = eigen_vectors.col(i);
    eigen_vectors.col(i) = eigen_vectors.col(j);
    eigen_vectors.col(j) = tmp;
    i++;
    j--;
  }

  std::vector<Eigen::VectorXd> source_data;
  source_data.reserve(eigen_vectors.rows());
  for (uint i = 0; i < eigen_vectors.rows(); i++) {
    source_data.push_back(eigen_vectors.row(i));
  }
  timer.Pause();
  LOG(INFO) << "4. Eigen Vectors Reverse Time: " << timer.ElapsedSeconds();

  // 5. Invert K-Means for clustering.
  timer.Start();
  std::vector<uint32_t> cluster_assignment;
  std::vector<Eigen::VectorXd> centers;
  KMeans(source_data, cluster_assignment, centers, k);
  timer.Pause();
  LOG(INFO) << "5. KMeans Time: " << timer.ElapsedSeconds();

  for (uint i = 0; i < cluster_assignment.size(); i++) {
    labels_[nodes_[i]] = cluster_assignment[i];
  }
  return labels_;
}

Eigen::SparseMatrix<double> SpectralCluster::ComputeLaplacian(
    const Eigen::SparseMatrix<double>& S,
    const std::unordered_map<int, int>& degrees) const {
  // Compute degree matrix.
  const int N = degrees.size();
  Eigen::SparseMatrix<double> D(N, N);
  // Eigen::SparseMatrix<double> D_inv(N, N);
  // Eigen::SparseMatrix<double> D_sqrt(N, N);
  for (auto it = degrees.begin(); it != degrees.end(); ++it) {
    int id = it->first;
    D.insert(id, id) = it->second;
    // D.insert(node_mapper_.at(id), node_mapper_.at(id)) = it->second;
    // D_inv.insert(node_mapper_.at(id), node_mapper_.at(id)) = 1.0 /
    // it->second; D_sqrt.insert(node_mapper_.at(id), node_mapper_.at(id)) =
    // -1.0 / sqrt(it->second);
  }
  D.makeCompressed();
  // D_inv.makeCompressed();
  // D_sqrt.makeCompressed();

  // Compute Laplacian matrix.
  Eigen::SparseMatrix<double> L = D - S;
  // Eigen::MatrixXd L_random = D_inv * L;
  // Eigen::MatrixXd L_sym = D_sqrt * L * D_sqrt;

  return L;
}

}  // namespace DAGSfM