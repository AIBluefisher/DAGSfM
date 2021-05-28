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

#include "clustering/image_clustering.h"

#include <glog/logging.h>

#include "clustering/community_detection_cluster.h"
#include "clustering/hybrid_cluster.h"
#include "clustering/kmeans_cluster.h"
#include "clustering/ncut_cluster.h"
#include "clustering/spectral_cluster.h"
#include "graph/graph.h"
#include "util/map_util.h"
#include "util/misc.h"
#include "util/random.h"

namespace DAGSfM {

bool ImageClustering::Options::Check() const {
  CHECK_GT(image_overlap, 2);
  CHECK_LE(completeness_ratio, 1.0);
  CHECK_GT(num_images_ub, 0);
  CHECK_GT(branching_factor, 0);
  if (is_output_igraph && graph_dir.size() == 0) {
    return false;
  }
  return true;
}

ImageClustering::ImageClustering(const Options& options,
                                 const ImageCluster& root_cluster)
    : options_(options), root_cluster_(root_cluster) {
  CHECK(options_.Check());
  summary_.original_images_num = root_cluster_.ImageIdsSize();
  summary_.original_edges_num = root_cluster_.EdgesSize();
}

void ImageClustering::Cut() {
  timer_.Start();
  const uint num_clusters =
      root_cluster_.ImageIdsSize() / options_.num_images_ub;
  CHECK_GE(num_clusters, 1);

  std::vector<std::pair<int, int>> image_pairs;
  std::vector<int> weights;
  image_pairs.reserve(root_cluster_.EdgesSize());
  weights.reserve(root_cluster_.EdgesSize());

  for (const auto& edge : root_cluster_.Edges()) {
    image_pairs.emplace_back(edge.first.first, edge.first.second);
    weights.emplace_back(edge.second);
  }

  LOG(INFO) << "Images Clustering Started ";

  const std::unique_ptr<Cluster> cluster = this->CreateCluster();
  CHECK_NOTNULL(cluster.get());
  LOG(INFO) << "cluster num: " << num_clusters;
  cluster->InitIGraph(image_pairs, weights);
  std::unordered_map<int, int> labels =
      cluster->ComputeCluster(image_pairs, weights, num_clusters);
  if (options_.is_output_igraph) {
    LOG(INFO) << "Output igraph to " << options_.graph_dir;
    CreateDirIfNotExists(options_.graph_dir);
    cluster->OutputIGraph(options_.graph_dir);
  }
  LOG(INFO) << "Cutting Complete, grouping images...";

  // Collect nodes according to the partition result
  intra_clusters_.resize(cluster->ClusterNum());
  for (const auto label : labels) {
    intra_clusters_[label.second].AddImageId(label.first);
  }
  for (uint i = 0; i < intra_clusters_.size(); i++) {
    intra_clusters_[i].cluster_id = i;
  }

  // Grouping edges
  LOG(INFO) << "Grouping edges...";
  for (size_t k = 0; k < image_pairs.size(); k++) {
    const image_t i = image_pairs[k].first;
    const image_t j = image_pairs[k].second;
    const int cluster_id1 = labels[i];
    const int cluster_id2 = labels[j];

    if (cluster_id1 == cluster_id2) {
      intra_clusters_[cluster_id1].AddEdge(std::make_pair(ImagePair(i, j), weights[k]));
    } else {
      const ImagePair view_pair = cluster_id1 < cluster_id2
                                      ? ImagePair(cluster_id1, cluster_id2)
                                      : ImagePair(cluster_id2, cluster_id1);
      clusters_lost_edges_[view_pair].emplace_back(i, j, weights[k]);
    }
  }

  timer_.Pause();
  summary_.total_cutting_num = 1;
  summary_.total_cutting_time = timer_.ElapsedSeconds();
}

void ImageClustering::Expand() {
  LOG(INFO) << "Expanding Images...";

  const uint num_clusters = intra_clusters_.size();
  inter_clusters_.reserve(num_clusters);
  for (auto cluster : intra_clusters_) {
    inter_clusters_.emplace_back(cluster);
  }

  timer_.Start();
  if (num_clusters > 1) {
    for (auto it : clusters_lost_edges_) {
      const ImagePair cluster_pair = it.first;
      std::vector<graph::Edge> lost_edges = it.second;
      ImageCluster& cluster1 = inter_clusters_[cluster_pair.first];
      ImageCluster& cluster2 = inter_clusters_[cluster_pair.second];
      AddLostEdgesBetweenClusters(cluster1, cluster2, lost_edges);
    }
  }

  timer_.Pause();
  summary_.total_expansion_time = timer_.ElapsedSeconds();
  summary_.total_expansion_num = 1;
  summary_.total_time =
      summary_.total_cutting_time + summary_.total_expansion_time;
  AnalyzeStatistic();
}

void ImageClustering::Expand(const int num_threads, Thread* thread) {
  LOG(INFO) << "Expanding Images...";

  const uint num_clusters = intra_clusters_.size();
  inter_clusters_.reserve(num_clusters);
  for (const auto& cluster : intra_clusters_) {
    inter_clusters_.emplace_back(cluster);
  }

  ThreadPool expand_thread_pool(num_threads);
  std::vector<std::future<void>> futures;

  timer_.Start();
  LOG(INFO) << "num_clusters: " << num_clusters;
  if (num_clusters > 1) {
    for (const auto& it : clusters_lost_edges_) {
      if (thread->IsStopped()) {
        return;
      }
      const ImagePair cluster_pair = it.first;
      std::vector<graph::Edge> lost_edges = it.second;
      futures.push_back(
          expand_thread_pool.AddTask(
              &ImageClustering::AddLostEdgesBetweenClusters, this,
              std::ref(inter_clusters_[cluster_pair.first]),
              std::ref(inter_clusters_[cluster_pair.second]),
              lost_edges));
    }
  }

  for (auto& future : futures) {
    future.get();
  }

  timer_.Pause();
  summary_.total_expansion_time = timer_.ElapsedSeconds();
  summary_.total_expansion_num = 1;
  summary_.total_time =
      summary_.total_cutting_time + summary_.total_expansion_time;
  AnalyzeStatistic();
}

void ImageClustering::ExpandAllEdges() {
  LOG(INFO) << "Expanding All Lost Edges...";

  const uint num_clusters = intra_clusters_.size();
  inter_clusters_.reserve(num_clusters);
  for (const auto& cluster : intra_clusters_) {
    inter_clusters_.emplace_back(cluster);
  }

  timer_.Start();
  if (num_clusters > 1) {
    for (auto it : clusters_lost_edges_) {
      const ImagePair cluster_pair = it.first;
      std::vector<graph::Edge> lost_edges = it.second;

      ImageCluster& cluster1 = inter_clusters_[cluster_pair.first];
      ImageCluster& cluster2 = inter_clusters_[cluster_pair.second];

      for (const graph::Edge& edge : lost_edges) {
        const ImagePair image_pair = edge.src < edge.dst
                                         ? ImagePair(edge.src, edge.dst)
                                         : ImagePair(edge.dst, edge.src);

        ImageCluster& selected_cluster =
            cluster1.EdgesSize() > cluster2.EdgesSize() ? cluster2 : cluster1;

        const std::unordered_set<image_t>& images = selected_cluster.ImageIds();

        if (images.count(edge.src) == 0) {
          selected_cluster.AddImageId(edge.src);
        }
        if (images.count(edge.dst) == 0) {
          selected_cluster.AddImageId(edge.dst);
        }

        selected_cluster.AddEdge(image_pair, edge.weight);
      }
    }
  }

  timer_.Pause();
  summary_.total_expansion_time = timer_.ElapsedSeconds();
  summary_.total_expansion_num = 1;
  summary_.total_time =
      summary_.total_cutting_time + summary_.total_expansion_time;
  AnalyzeStatistic();
}

std::vector<ImageCluster> ImageClustering::BiCut(
    const ImageCluster& image_cluster) {
  timer_.Start();
  std::vector<ImageCluster> image_clusters(options_.branching_factor);
  std::vector<std::pair<int, int>> image_pairs;
  std::vector<int> weights;

  for (const auto& edge : image_cluster.Edges()) {
    image_pairs.push_back(std::make_pair(edge.first.first, edge.first.second));
    weights.push_back(edge.second);
  }

  const std::unique_ptr<Cluster> cluster = this->CreateCluster();
  std::unordered_map<int, int> labels =
      cluster->ComputeCluster(image_pairs, weights, options_.branching_factor);

  // collect nodes according to the partition result
  for (const auto label : labels) {
    image_clusters[label.second].AddImageId(label.first);
  }

  for (size_t k = 0; k < image_pairs.size(); k++) {
    const image_t i = image_pairs[k].first;
    const image_t j = image_pairs[k].second;
    const int cluster_id1 = labels[i];
    const int cluster_id2 = labels[j];

    if (cluster_id1 == cluster_id2) {
      image_clusters[cluster_id1].AddEdge(std::make_pair(ImagePair(i, j), weights[k]));
    } else {
      discarded_edges_.push(graph::Edge(i, j, weights[k]));
    }
  }

  timer_.Pause();
  summary_.total_cutting_time += timer_.ElapsedSeconds();
  summary_.total_cutting_num++;

  return image_clusters;
}

void ImageClustering::CutAndExpand() {
  Timer timer;
  timer.Start();

  std::vector<ImageCluster> init_clusters = BiCut(root_cluster_);
  std::queue<ImageCluster> candidate_clusters;
  for (const auto& cluster : init_clusters) {
    candidate_clusters.emplace(cluster);
  }

  while (!candidate_clusters.empty()) {
    LOG(INFO) << summary_.total_iters_num++ << "-th iterations";

    while (!candidate_clusters.empty()) {
      ImageCluster cluster = candidate_clusters.front();
      candidate_clusters.pop();
      if (cluster.ImageIdsSize() <= options_.num_images_ub) {
        inter_clusters_.emplace_back(cluster);
      } else {
        std::vector<ImageCluster> clusters = BiCut(cluster);
        for (int k = 0; k < options_.branching_factor; k++) {
          candidate_clusters.emplace(clusters[k]);
        }
      }
    }

    // Graph Expansion
    timer_.Start();
    while (!discarded_edges_.empty()) {
      // If there is no cluster satisfies completeness ratio constraint
      // we can jump out of the loop
      if (!IsRemainingClusters()) {
        graph::LargerEdgePriorityQueue<graph::Edge> empty_edges;
        std::swap(empty_edges, discarded_edges_);
        break;
      }
      graph::Edge edge = discarded_edges_.top();
      discarded_edges_.pop();

      const int cluster_id = ClusterSatisfyCompletenessRatio(edge);
      if (cluster_id == -1) {
        LOG(INFO) << "Not find suitable clusters for edge: " << edge.src << ", "
                  << edge.dst;
        continue;
      }

      ImageCluster& image_cluster = inter_clusters_[cluster_id];
      const std::unordered_set<image_t>& image_sets = image_cluster.ImageIds();
      image_t added_image =
          image_sets.find(edge.src) == image_sets.cend() ? edge.src : edge.dst;
      VLOG(2) << "discarded edge: " << edge.src << ", " << edge.dst;
      VLOG(2) << "added image:    " << added_image;
      if (image_sets.find(added_image) == image_sets.cend()) {
        image_cluster.AddImageId(added_image);
      }
      image_cluster.AddEdge(std::make_pair(ImagePair(edge.src, edge.dst), edge.weight));
    }
    timer_.Pause();
    summary_.total_expansion_time += timer_.ElapsedSeconds();
    summary_.total_expansion_num++;

    // After graph expansion, there may be some image graphs that don't
    // satisfy the size constraint, thus we need to re-partition it
    LOG(INFO) << "Re-grouping clusters...";
    for (auto iter = inter_clusters_.begin(); iter != inter_clusters_.end();) {
      if (iter->ImageIdsSize() >
          options_.relax_ratio * options_.num_images_ub) {
        candidate_clusters.push(*iter);
        iter = inter_clusters_.erase(iter);
      } else {
        iter++;
      }
    }
  }

  timer.Pause();
  summary_.total_time += timer.ElapsedSeconds();
  LOG(INFO) << "Image Clustering Complete!";
  AnalyzeStatistic();
}

const std::vector<ImageCluster>& ImageClustering::GetIntraClusters() const {
  return intra_clusters_;
}

const std::vector<ImageCluster>& ImageClustering::GetInterClusters() const {
  return inter_clusters_;
}

const ImageCluster& ImageClustering::GetRootCluster() const {
  return root_cluster_;
}

double ImageClustering::AnalyzeDegree(
    const std::vector<std::pair<int, int>>& image_pairs,
    const std::vector<int>& weights) const {
  using namespace DAGSfM::graph;
  Graph<Node, Edge> graph;
  for (uint i = 0; i < image_pairs.size(); i++) {
    graph.AddEdge(
        Edge(image_pairs[i].first, image_pairs[i].second, weights[i]));
    graph.AddEdge(
        Edge(image_pairs[i].second, image_pairs[i].first, weights[i]));
  }

  graph.CountInDegrees();
  graph.CountOutDegrees();
  graph.CountDegrees();

  // TODO: (chenyu) degrees are depends on graph weights.
  const std::unordered_map<size_t, size_t> degrees = graph.GetDegrees();

  // Analyze the degrees of view graph.
  double ave_degree = 0.0;
  for (auto degree : degrees) {
    ave_degree += degree.second;
  }
  ave_degree /= degrees.size();

  double covariance_degree = 0.0;
  for (auto degree : degrees) {
    covariance_degree +=
        (degree.second - ave_degree) * (degree.second - ave_degree);
  }
  covariance_degree /= degrees.size();

  return covariance_degree / ave_degree;
}

std::unique_ptr<Cluster> ImageClustering::CreateCluster() const {
  if (options_.cluster_type == "NCUT") {
    return std::unique_ptr<Cluster>(new NCutCluster());
  } else if (options_.cluster_type == "KMEANS") {
    return std::unique_ptr<Cluster>(new KMeansCluster());
  } else if (options_.cluster_type == "SPECTRAL") {
    return std::unique_ptr<Cluster>(new SpectralCluster());
  } else if (options_.cluster_type == "HYBRID") {
    return std::unique_ptr<Cluster>(new HybridCluster());
  } else if (options_.cluster_type == "COMMUNITY_DETECTION") {
    return std::unique_ptr<Cluster>(new CommunityDetectionCluster());
  } else {
    return std::unique_ptr<Cluster>(new NCutCluster());
  }
}

std::unique_ptr<Cluster> ImageClustering::CreateCluster(
    const std::vector<std::pair<int, int>>& image_pairs,
    const std::vector<int>& weights)  // const
{
  const double sigma = 4.0;
  const double degree_distribution = AnalyzeDegree(image_pairs, weights);

  if (degree_distribution >= sigma) {
    options_.cluster_type = "HYBRID";
    return std::unique_ptr<Cluster>(new HybridCluster());
  } else {
    options_.cluster_type = "NCUT";
    return std::unique_ptr<Cluster>(new NCutCluster());
  }
}

bool ImageClustering::IsSatisfyCompletenessRatio(const ImageCluster& cluster) {
  if (cluster.IsConditionSatisfy()) return true;

  const uint i = cluster.cluster_id;
  uint repeated_node_num = 0;
  for (uint j = 0; j < inter_clusters_.size(); j++) {
    if (i == j) continue;
    const uint common_images_num =
        CommonImagesNum(inter_clusters_[i], inter_clusters_[j]);
    repeated_node_num += common_images_num;
  }

  // check if satisfy completeness ratio to avoid adding too many edges
  const float repeated_ratio =
      (float)repeated_node_num / (float)(inter_clusters_[i].ImageIdsSize());
  if (repeated_ratio <= options_.completeness_ratio) {
    VLOG(4) << "repeated ratio: " << repeated_ratio;
    return false;
  } else {
    inter_clusters_[i].SetIsConditionSatisfy(true);
    return true;
  }
}

int ImageClustering::ClusterSatisfyCompletenessRatio(const graph::Edge& edge) {
  int cluster_id = -1;

  for (uint i = 0; i < inter_clusters_.size() - 1; i++) {
    const std::unordered_set<image_t>& image_sets = inter_clusters_[i].ImageIds();
    if (image_sets.find(edge.src) == image_sets.cend() &&
        image_sets.find(edge.dst) == image_sets.cend()) {
      continue;
    }

    uint repeated_node_num = 0;
    for (uint j = 0; j < inter_clusters_.size(); j++) {
      if (i == j) continue;
      const uint common_images_num =
          CommonImagesNum(inter_clusters_[i], inter_clusters_[j]);
      // LOG(INFO) << "common images num: " << common_images_num;
      repeated_node_num += common_images_num;
    }

    // check if satisfy completeness ratio to avoid adding too many edges
    const float repeated_ratio =
        (float)repeated_node_num / (float)(inter_clusters_[i].ImageIdsSize());
    if (repeated_ratio <= options_.completeness_ratio) {
      VLOG(4) << "repeated ratio: " << repeated_ratio;
      cluster_id = i;
      break;
    } else {
      // LOG(INFO) << "cluster " << i << " repeated ratio: " << repeated_ratio;
      // LOG(INFO) << "total clusters: " << inter_clusters_.size();
      inter_clusters_[i].SetIsConditionSatisfy(true);
    }
  }

  return cluster_id;
}

uint ImageClustering::CommonImagesNum(const ImageCluster& cluster1,
                                      const ImageCluster& cluster2) const {
  uint common_images_num = 0;
  const std::unordered_set<image_t> images1 = cluster1.ImageIds();
  const std::unordered_set<image_t> images2 = cluster2.ImageIds();

  for (auto it = images1.cbegin(); it != images1.cend(); ++it) {
    if (images2.find(*it) != images2.cend()) {
      common_images_num++;
    }
  }
  // VLOG(2) << "common images num: " << common_images_num;
  return common_images_num;
}

void ImageClustering::OutputClusteringSummary() const {
  LOG(INFO)
      << "#Images Clustering Config:#\n"
      << "\t - image upperbound: " << options_.num_images_ub << "\n"
      << "\t - completeness ratio: " << options_.completeness_ratio << "\n"
      << "\t - cluster type: " << options_.cluster_type << "\n"
      << "#Images Clustering Summary:#\n"
      << "\t - Clusters number: " << inter_clusters_.size() << "\n"
      << "\t - Total graph cutting time: " << summary_.total_cutting_time
      << " seconds\n"
      << "\t - Total graph cutting number: " << summary_.total_cutting_num << "\n"
      << "\t - Total graph expansion time: " << summary_.total_expansion_time
      << " seconds\n"
      << "\t - Total graph expansion number: " << summary_.total_expansion_num
      << "\n"
      << "\t - Total time took: " << summary_.total_time << " seconds\n"
      << "\t - Total iteration number: " << summary_.total_iters_num << "\n"
      << "\t - Images number expanded from " << summary_.original_images_num
      << " to " << summary_.clustered_images_num << "\n"
      << "\t - Repeated Ratio: "
      << (float)(summary_.clustered_images_num - summary_.original_images_num) /
             (float)summary_.original_images_num
      << "\n"
      << "\t - Edges number reduced from " << summary_.original_edges_num << " to "
      << summary_.clustered_edges_num << "\n"
      << "\t - Lost ratio: "
      << (float)(summary_.original_edges_num - summary_.clustered_edges_num) /
             (float)summary_.original_edges_num;
}

bool ImageClustering::IsRemainingClusters() const {
  for (auto cluster : inter_clusters_) {
    if (!cluster.IsConditionSatisfy()) return true;
  }
  return false;
}

void ImageClustering::AddLostEdgesBetweenClusters(
    ImageCluster& cluster1, ImageCluster& cluster2,
    std::vector<graph::Edge>& lost_edges) {
  // The intersection of cluster1 and cluster2 can't surpass the
  // maximum image overlapping.
  if (CommonImagesNum(cluster1, cluster2) > options_.image_overlap) {
    return;
  }

  // Either cluster1 or cluster2 should not achieve the completeness ratio.
  if (IsSatisfyCompletenessRatio(cluster1) &&
      IsSatisfyCompletenessRatio(cluster2)) {
    return;
  }

  const auto cmp = [](const graph::Edge& edge1, const graph::Edge& edge2) {
    return edge1.weight > edge2.weight;
  };
  std::sort(lost_edges.begin(), lost_edges.end(), cmp);

  RandomNumberGenerator rng;
  for (uint k = 0; /*k < options_.image_overlap &&*/ k < lost_edges.size();
       k++) {
    const ImagePair view_pair =
        lost_edges[k].src < lost_edges[k].dst
            ? ImagePair(lost_edges[k].src, lost_edges[k].dst)
            : ImagePair(lost_edges[k].dst, lost_edges[k].src);

    const std::unordered_set<image_t> images1 = cluster1.ImageIds();
    const std::unordered_set<image_t> images2 = cluster2.ImageIds();

    const image_t added_image1 =
        images1.find(lost_edges[k].src) == images1.cend() ? lost_edges[k].src
                                                         : lost_edges[k].dst;
    const image_t added_image2 =
        images2.find(lost_edges[k].src) == images2.cend() ? lost_edges[k].src
                                                         : lost_edges[k].dst;

    // Select a cluster that has a smaller size, such that
    // larger clusters can avoid become too large.
    int selected_image =
        cluster1.ImageIdsSize() > cluster2.ImageIdsSize() ? 2 : 1;
    if (selected_image == 1) {
      if (!IsSatisfyCompletenessRatio(cluster1) &&
          images1.find(added_image1) == images1.cend()) {
        cluster1.AddImageId(added_image1);
        cluster1.AddEdge(view_pair, lost_edges[k].weight);
      }
    } else {
      if (!IsSatisfyCompletenessRatio(cluster2) &&
          images2.find(added_image2) == images2.cend()) {
        cluster2.AddImageId(added_image2);
        cluster2.AddEdge(view_pair, lost_edges[k].weight);
      }
    }

    // If both completeness ratios of cluster1 and cluster2 
    // are satisfied, return earlier.
    if (IsSatisfyCompletenessRatio(cluster1) &&
        IsSatisfyCompletenessRatio(cluster2)) {
      return;
    }
  }
}

void ImageClustering::AnalyzeStatistic() {
  LOG(INFO) << "Analysing Statistics...";
  for (auto& cluster : inter_clusters_) {
    summary_.clustered_images_num += cluster.ImageIdsSize();
    summary_.clustered_edges_num += cluster.EdgesSize();
  }
}

}  // namespace DAGSfM