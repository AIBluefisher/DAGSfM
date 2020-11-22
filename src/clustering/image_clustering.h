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

#ifndef SRC_CLUSTERING_IMAGE_CLUSTERING_H_
#define SRC_CLUSTERING_IMAGE_CLUSTERING_H_

#include <algorithm>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "clustering/cluster.h"
#include "graph/graph.h"
#include "util/hash.h"
#include "util/timer.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

using Edges = std::unordered_map<ImagePair, int>;
struct ImageCluster {
  int cluster_id;
  std::vector<image_t> image_ids;
  Edges edges;
  bool is_condition_satisfy = false;
  bool completed = false;

  // ImageCluster(const ImageCluster& image_cluster)
  // {
  //     for (auto image_id : image_cluster.image_ids) {
  //         image_ids.push_back(image_id);
  //     }
  //     for (auto it : image_cluster.edges) {
  //         edges[it.first] = it.second;
  //     }
  //     is_condition_satisfy = image_cluster.is_condition_satisfy;
  // }

  void ShowInfo() const {
    LOG(INFO) << image_ids.size() << " nodes";
    for (auto image_id : image_ids) {
      std::cout << image_id << " ";
    }
    std::cout << "\n";

    LOG(INFO) << edges.size() << " edges";
  }
};

class ImageClustering {
 public:
  struct Options {
    // The upper bound of images number
    uint num_images_ub = 100;

    // The number of overlapping images between child clusters.
    uint image_overlap = 50;

    // completeness ratio for selecting expanded clusters
    float completeness_ratio = 0.5;

    float relax_ratio = 1.3;

    // the number of cluster we partition each time
    int branching_factor = 2;

    // Maximum number of edges for building connections between clusters
    uint max_num_cluster_pairs = 0;

    bool is_output_igraph = true;

    std::string cluster_type = "NCUT";

    std::string graph_dir = "";

    bool Check() const;
  };

  struct Summary {
    // The number of grpah cut
    uint total_cutting_num = 0;

    // The number of graph expansion
    uint total_expansion_num = 0;

    // Total time took for graph cut
    double total_cutting_time = 0;

    // Total time took for graph expansion
    double total_expansion_time = 0;

    // Total time time for images clustering
    double total_time = 0;

    // Original number of images
    uint original_images_num = 0;

    // Total number of images after images clustering
    uint clustered_images_num = 0;

    // Original number of images' edges
    uint original_edges_num = 0;

    // Total number of edges after images clustering
    uint clustered_edges_num = 0;

    // Total iteration number
    uint total_iters_num = 0;
  };

  ImageClustering(const Options& options, const ImageCluster& root_cluster);

  virtual void Cut();

  virtual void Expand();

  virtual void ExpandAllEdges();

  virtual void CutAndExpand();

  std::vector<ImageCluster> BiCut(const ImageCluster& cluster);

  std::vector<ImageCluster> GetIntraClusters() const;

  std::vector<ImageCluster> GetInterClusters() const;

  ImageCluster GetRootCluster() const;

  void OutputClusteringSummary() const;

 private:
  Options options_;

  Summary summary_;

  Timer timer_;

  const ImageCluster root_cluster_;

  // clusters that have no connection with each other
  std::vector<ImageCluster> intra_clusters_;

  // clusters that shares common images
  std::vector<ImageCluster> inter_clusters_;

  // The discarded edges after image clustering
  std::priority_queue<graph::Edge> discarded_edges_;
  std::unordered_map<ImagePair, std::vector<graph::Edge>> clusters_lost_edges_;

  double AnalyzeDegree(const std::vector<std::pair<int, int>>& image_pairs,
                       const std::vector<int>& weights) const;

  std::unique_ptr<Cluster> CreateCluster() const;

  std::unique_ptr<Cluster> CreateCluster(
      const std::vector<std::pair<int, int>>& image_pairs,
      const std::vector<int>& weights);

  bool IsSatisfyCompletenessRatio(const ImageCluster& cluster);
  int ClusterSatisfyCompletenessRatio(const graph::Edge& edge);

  uint CommonImagesNum(const ImageCluster& cluster1,
                       const ImageCluster& cluster2) const;

  bool IsRemainingClusters() const;

  void AddLostEdgesBetweenClusters(ImageCluster& cluster1,
                                   ImageCluster& cluster2,
                                   std::vector<graph::Edge>& lost_edges);

  void AnalyzeStatistic();
};

}  // namespace DAGSfM

#endif