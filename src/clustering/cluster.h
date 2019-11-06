#ifndef SRC_CLUSTERING_CLUSTER_H_
#define SRC_CLUSTERING_CLUSTER_H_

#include <unordered_map>
#include <cstdio>
#include <glog/logging.h>

#include "util/types.h"

using namespace colmap;

namespace GraphSfM {

enum ClusterType { NCUT, KMEANS, SPECTRAL, COMMUNITY_DETECTION, HYBRID };

class Cluster
{
protected:
    std::unordered_map<int, int> labels_;

public:
    Cluster() {};
    ~Cluster() {};

    virtual std::unordered_map<int, int> ComputeCluster(
        const std::vector<std::pair<int, int>>& edges,
        const std::vector<int>& weights,
        const int num_partitions) = 0;
};

} // namespace GraphSfM

#endif