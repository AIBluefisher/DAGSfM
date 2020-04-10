#include "clustering/spectral_cluster.h"
#include "clustering/kmeans.h"

#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Spectra/SymEigsSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>
#include <glog/logging.h>

#include "util/timer.h"

namespace GraphSfM {

std::unordered_map<int, int> SpectralCluster::ComputeCluster(
        const std::vector<std::pair<int, int>>& edges,
        const std::vector<int>& weights,
        const int num_partitions)
{
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

    for (uint i = 0; i < edges.size(); i++) {
        auto edge = edges[i];
        nodes_.push_back(edge.first);
        nodes_.push_back(edge.second);
    }

    std::sort(nodes_.begin(), nodes_.end());
    nodes_.erase(std::unique(nodes_.begin(), nodes_.end()), nodes_.end());

    // 1. Compute similarity graph.
    for (uint i = 0; i < nodes_.size(); i++) {
        node_mapper_[nodes_[i]] = i;
    }

    timer.Start();
    const int N = nodes_.size();
    Eigen::SparseMatrix<double> S(N, N);
    for (uint i = 0; i < edges.size(); i++) {
        int src = edges[i].first, dst = edges[i].second;
        s_triplets.push_back(Eigen::Triplet<double>(src, dst, weights[i]));
        s_triplets.push_back(Eigen::Triplet<double>(dst, src, weights[i]));
        degrees[src] += 2;
    }

    S.setFromTriplets(s_triplets.begin(), s_triplets.end());
    S.makeCompressed();
    timer.Pause();
    LOG(INFO) << "1. Similarity Graph Computation Time: " << timer.ElapsedSeconds();

    // 2. Compute Laplacian matrix.
    timer.Start();
    Eigen::SparseMatrix<double> L = ComputeLaplacian(S, degrees);
    L.makeCompressed();
    timer.Pause();
    LOG(INFO) << "2. Laplacian Matrix Computation Time: " << timer.ElapsedSeconds();

    // 3. Compute the top-k smallest eigen values and corresponding eigen vectors.
    timer.Start();
    Spectra::SparseSymMatProd<double> op(L);
    Spectra::SymEigsSolver<double, Spectra::SMALLEST_ALGE,
                          Spectra::SparseSymMatProd<double>> eigs(&op, 
                                                                  k,
                                                                  std::min(2 * k, N));
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
        i++; j--;
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
                const std::unordered_map<int, int>& degrees) const
{
    // Compute degree matrix.
    const int N = degrees.size();
    Eigen::SparseMatrix<double> D(N, N);
    // Eigen::SparseMatrix<double> D_inv(N, N);
    // Eigen::SparseMatrix<double> D_sqrt(N, N);
    for (auto it = degrees.begin(); it != degrees.end(); ++it) {
        int id = it->first;
        D.insert(node_mapper_.at(id), node_mapper_.at(id)) = it->second;
        // D_inv.insert(node_mapper_.at(id), node_mapper_.at(id)) = 1.0 / it->second;
        // D_sqrt.insert(node_mapper_.at(id), node_mapper_.at(id)) = -1.0 / sqrt(it->second);
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

} // namespace GraphSfM