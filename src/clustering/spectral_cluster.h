#ifndef SRC_CLUSTERING_SPECTRAL_CLUSTER_H_
#define SRC_CLUSTERING_SPECTRAL_CLUSTER_H_

#include <vector>

#include "clustering/cluster.h"

#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

namespace GraphSfM {

class SpectralCluster : public Cluster
{
public:
    virtual std::unordered_map<int, int> ComputeCluster(
        const std::vector<std::pair<int, int>>& edges,
        const std::vector<int>& weights,
        const int num_partitions) override;

private:
    Eigen::SparseMatrix<double> ComputeLaplacian(
        const Eigen::SparseMatrix<double>& S,
        const std::unordered_map<int, int>& degrees) const;
    
    std::vector<int> nodes_;

    int cluster_num_;
    
    std::unordered_map<int, int> node_mapper_;
};

} // namespace GraphSfM

#endif