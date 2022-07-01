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
#include <mutex>

#include "clustering/cluster.h"
#include "graph/graph.h"
#include "util/hash.h"
#include "util/timer.h"
#include "util/types.h"
#include "util/threading.h"

using namespace colmap;

namespace DAGSfM {

using Edges = std::unordered_map<ImagePair, int>;
class ImageCluster {
 private:
  ImageCluster(const ImageCluster& rhs, const std::lock_guard<std::mutex> &) 
  : cluster_id(rhs.cluster_id), image_ids_(rhs.image_ids_), edges_(rhs.edges_),
    is_condition_satisfy_(rhs.is_condition_satisfy_), completed_(rhs.completed_) {}

 public:
  ImageCluster() = default;
  // Thread safe copy constructor, and copy assignment operator.
  ImageCluster(const ImageCluster& rhs) : ImageCluster(rhs, std::lock_guard<std::mutex>(rhs.mutex_)) {}
  ImageCluster& operator=(const ImageCluster& rhs) {
    if (this != &rhs) {
      std::unique_lock<std::mutex> my_lock(mutex_, std::defer_lock);
      std::unique_lock<std::mutex> rhs_lock(rhs.mutex_, std::defer_lock);
      std::lock(my_lock, rhs_lock);
      cluster_id = rhs.cluster_id;
      image_ids_ = rhs.image_ids_;
      edges_ = rhs.edges_;
      is_condition_satisfy_ = rhs.is_condition_satisfy_;
      completed_ = rhs.completed_;
    }
    return *this;
  }
 public:
  int cluster_id = 0;

  // Setters.
  inline void SetImageIds(const std::unordered_set<image_t>& image_ids);
  inline void SetEdges(const DAGSfM::Edges& edges);
  inline void SetIsConditionSatisfy(const bool is_condition_satisfy);
  inline void SetCompleted(const bool completed);
  
  // Add a new image id or a new edge to this image cluster.
  inline void AddImageId(const image_t image_id);
  inline void AddEdge(const ImagePair& view_pair, const int weight);
  inline void AddEdge(const std::pair<ImagePair, int>& edge);

  // Get const objects.
  inline const std::unordered_set<image_t>& ImageIds() const;
  inline const DAGSfM::Edges& Edges() const;
  inline const bool IsConditionSatisfy() const;
  inline const bool Completed() const;

  // Get size of image_ids_ and edges_
  inline const size_t ImageIdsSize() const;
  inline const size_t EdgesSize() const;

  void ShowInfo() const {
    std::string info = "Cluster " + std::to_string(cluster_id) + ": [";
    info += ("node: " + std::to_string(image_ids_.size()));
    // for (auto image_id : image_ids) {
    //   std::cout << image_id << " ";
    // }
    // std::cout << "\n";

    info += (", edges: " + std::to_string(edges_.size()) + "]");
    LOG(INFO) << info;
  }

 private:
  std::unordered_set<image_t> image_ids_;
  DAGSfM::Edges edges_;
  bool is_condition_satisfy_ = false;
  bool completed_ = false;
  mutable std::mutex mutex_;
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

    bool is_output_igraph = false;

    uint num_threads = -1;

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
  virtual void Expand(const int num_threads, Thread* thread);

  virtual void ExpandAllEdges();

  virtual void CutAndExpand();

  std::vector<ImageCluster> BiCut(const ImageCluster& cluster);

  const std::vector<ImageCluster>& GetIntraClusters() const;
  const std::vector<ImageCluster>& GetInterClusters() const;
  const ImageCluster& GetRootCluster() const;

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
  graph::LargerEdgePriorityQueue<graph::Edge> discarded_edges_;
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

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

void ImageCluster::SetImageIds(const std::unordered_set<image_t>& image_ids) {
  std::lock_guard<std::mutex> l(mutex_);
  image_ids_ = image_ids;
}

void ImageCluster::SetEdges(const DAGSfM::Edges& edges) {
  std::lock_guard<std::mutex> l(mutex_);
  edges_ = edges;
}

void ImageCluster::SetIsConditionSatisfy(const bool is_condition_satisfy) {
  std::lock_guard<std::mutex> l(mutex_);
  is_condition_satisfy_ = is_condition_satisfy;
}

void ImageCluster::SetCompleted(const bool completed) {
  std::lock_guard<std::mutex> l(mutex_);
  completed_ = completed;
}

void ImageCluster::AddImageId(const image_t image_id) {
  std::lock_guard<std::mutex> l(mutex_);
  image_ids_.emplace(image_id);
}

void ImageCluster::AddEdge(const ImagePair& view_pair, const int weight) {
  std::lock_guard<std::mutex> l(mutex_);
  edges_[view_pair] = weight;
}

void ImageCluster::AddEdge(const std::pair<ImagePair, int>& edge) {
  std::lock_guard<std::mutex> l(mutex_);
  edges_.emplace(edge);
}

const std::unordered_set<image_t>& ImageCluster::ImageIds() const {
  std::lock_guard<std::mutex> l(mutex_);
  return image_ids_;
}

const DAGSfM::Edges& ImageCluster::Edges() const {
  std::lock_guard<std::mutex> l(mutex_);
  return edges_;
}

const bool ImageCluster::IsConditionSatisfy() const {
  std::lock_guard<std::mutex> l(mutex_);
  return is_condition_satisfy_;
}

const bool ImageCluster::Completed() const {
  std::lock_guard<std::mutex> l(mutex_);
  return completed_;
}

const size_t ImageCluster::ImageIdsSize() const {
  std::lock_guard<std::mutex> l(mutex_);
  return image_ids_.size();
}

const size_t ImageCluster::EdgesSize() const {
  std::lock_guard<std::mutex> l(mutex_);
  return edges_.size();
}

}  // namespace DAGSfM

#endif