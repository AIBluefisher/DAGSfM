#include "clustering/ncut_cluster.h"
#include "base/graph_cut.h"

namespace GraphSfM {

std::unordered_map<int, int> NCutCluster::ComputeCluster(
        const std::vector<std::pair<int, int>>& edges,
        const std::vector<int>& weights,
        const int num_partitions)
{
    if (num_partitions == 1) {
        for (auto edge : edges) {
            labels_[edge.first] = 0;
            labels_[edge.second] = 0;
        }
    } else {
        labels_ = colmap::ComputeNormalizedMinGraphCut(
            edges, weights, num_partitions);
    }
    
    return labels_;
}

} // namespace GraphSfM