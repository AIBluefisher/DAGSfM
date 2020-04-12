#include "clustering/image_clustering.h"
#include "clustering/ncut_cluster.h"
#include "clustering/kmeans_cluster.h"
#include "clustering/spectral_cluster.h"
#include "util/misc.h"
#include "util/random.h"
#include "util/map_util.h"
#include "graph/graph.h"

#include <glog/logging.h>

namespace GraphSfM {

bool ImageClustering::Options::Check() const
{
    CHECK_GT(image_overlap, 2);
    CHECK_LE(completeness_ratio, 1.0);
    CHECK_GT(num_images_ub, 0);
    CHECK_GT(branching_factor, 0);
    CHECK_GT(graph_dir.size(), 0);
    return true;
}

ImageClustering::ImageClustering(const Options& options,
                                 const ImageCluster& root_cluster)
    : options_(options),
      root_cluster_(root_cluster)
{
    CHECK(options_.Check());
    summary_.original_images_num = root_cluster_.image_ids.size();
    summary_.original_edges_num = root_cluster_.edges.size();
}

void ImageClustering::Cut()
{
    timer_.Start();
    const uint num_clusters = root_cluster_.image_ids.size() / options_.num_images_ub;
    CHECK_GE(num_clusters, 1);

    std::vector<std::pair<int, int>> image_pairs;
    std::vector<int> weights;
    image_pairs.reserve(root_cluster_.edges.size());
    weights.reserve(root_cluster_.edges.size());
    
    for (auto edge : root_cluster_.edges) {
        image_pairs.push_back(std::make_pair(edge.first.first, edge.first.second));
        weights.push_back(edge.second);
    }

    LOG(INFO) << "Images Clustering Started ";
    const std::unique_ptr<Cluster> cluster = this->CreateCluster();
    CHECK_NOTNULL(cluster.get());
    LOG(INFO) << "cluster num: " << num_clusters;

    std::unordered_map<int, int> labels = 
        cluster->ComputeCluster(image_pairs, weights, num_clusters);

    LOG(INFO) << "Cutting Complete, grouping images...";

    // Collect nodes according to the partition result
    intra_clusters_.resize(num_clusters);
    for (const auto label : labels) {
        intra_clusters_[label.second].image_ids.push_back(label.first);
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
            intra_clusters_[cluster_id1].edges.insert(
                std::make_pair(ViewIdPair(i, j), weights[k]));
        } else {
            const ViewIdPair view_pair = cluster_id1 < cluster_id2 ? 
                        ViewIdPair(cluster_id1, cluster_id2) : 
                        ViewIdPair(cluster_id2, cluster_id1);
            clusters_lost_edges_[view_pair].push_back(graph::Edge(i, j, weights[k]));
        }
    }

    timer_.Pause();
    summary_.total_cutting_num = 1;
    summary_.total_cutting_time = timer_.ElapsedSeconds();
}

void ImageClustering::Expand()
{
    LOG(INFO) << "Expanding Images...";

    const uint num_clusters = intra_clusters_.size();
    inter_clusters_.reserve(num_clusters);
    for (auto cluster : intra_clusters_) {
         inter_clusters_.emplace_back(cluster);
    }

    timer_.Start();
    if (num_clusters > 1) {
        for (auto it : clusters_lost_edges_) {
            const ViewIdPair cluster_pair = it.first;
            std::vector<graph::Edge> lost_edges = it.second;
            ImageCluster& cluster1 = inter_clusters_[cluster_pair.first];
            ImageCluster& cluster2 = inter_clusters_[cluster_pair.second];
            AddLostEdgesBetweenClusters(cluster1, cluster2, lost_edges);
        }
    }

    timer_.Pause();
    summary_.total_expansion_time = timer_.ElapsedSeconds();
    summary_.total_expansion_num = 1;
    summary_.total_time = summary_.total_cutting_time + summary_.total_expansion_time;
    AnalyzeStatistic();
}

std::vector<ImageCluster> ImageClustering::BiCut(const ImageCluster& image_cluster)
{
    timer_.Start();
    std::vector<ImageCluster> image_clusters(options_.branching_factor);
    std::vector<std::pair<int, int>> image_pairs;
    std::vector<int> weights;
    
    for (auto edge : image_cluster.edges) {
        image_pairs.push_back(std::make_pair(edge.first.first, edge.first.second));
        weights.push_back(edge.second);
    }

    const std::unique_ptr<Cluster> cluster = this->CreateCluster();
    std::unordered_map<int, int> labels = 
        cluster->ComputeCluster(image_pairs, weights, options_.branching_factor);

    // collect nodes according to the partition result
    for (const auto label : labels) {
        image_clusters[label.second].image_ids.push_back(label.first);
    }

    for (size_t k = 0; k < image_pairs.size(); k++) {
        const image_t i = image_pairs[k].first;
        const image_t j = image_pairs[k].second;
        const int cluster_id1 = labels[i];
        const int cluster_id2 = labels[j];

        if (cluster_id1 == cluster_id2) {
            image_clusters[cluster_id1].edges.insert(
                std::make_pair(ViewIdPair(i, j), weights[k]));
        } else {
            discarded_edges_.push(graph::Edge(i, j, weights[k]));
        }
    }

    timer_.Pause();
    summary_.total_cutting_time += timer_.ElapsedSeconds();
    summary_.total_cutting_num++;

    return image_clusters;
}

void ImageClustering::CutAndExpand()
{
    Timer timer;
    timer.Start();

    std::vector<ImageCluster> init_clusters = BiCut(root_cluster_);
    std::queue<ImageCluster> candidate_clusters;
    for (auto cluster : init_clusters) {
        candidate_clusters.push(cluster);
    }

    while (!candidate_clusters.empty()) {
        LOG(INFO) << summary_.total_iters_num++ << "-th iterations"; 

        while (!candidate_clusters.empty()) {
            ImageCluster cluster = candidate_clusters.front();
            candidate_clusters.pop();
            if (cluster.image_ids.size() <= options_.num_images_ub) {
                inter_clusters_.push_back(cluster);
            } else {
                std::vector<ImageCluster> clusters = BiCut(cluster);
                for (int k = 0; k < options_.branching_factor; k++) {
                    candidate_clusters.push(clusters[k]);
                }
            }
        }

        // Graph Expansion
        timer_.Start();
        while (!discarded_edges_.empty()) {
            // If there is no cluster satisfies completeness ratio constraint
            // we can jump out of the loop
            if (!IsRemainingClusters()) {
                std::priority_queue<graph::Edge> empty_edges;
                std::swap(empty_edges, discarded_edges_);
                break;
            }
            graph::Edge edge = discarded_edges_.top();
            discarded_edges_.pop();

            const int cluster_id = ClusterSatisfyCompletenessRatio(edge);
            if (cluster_id == -1) {
                LOG(INFO) << "Not find suitable clusters for edge: " 
                          << edge.src << ", " << edge.dst;
                continue;
            }

            ImageCluster& image_cluster = inter_clusters_[cluster_id];
            std::unordered_set<image_t> image_sets(image_cluster.image_ids.begin(), 
                                                   image_cluster.image_ids.end());
            image_t added_image = image_sets.find(edge.src) == image_sets.end()
                                  ? edge.src : edge.dst;
            VLOG(2) << "discarded edge: " << edge.src << ", " << edge.dst;
            VLOG(2) << "added image:    " << added_image;
            if (image_sets.find(added_image) == image_sets.end()) {
                image_cluster.image_ids.push_back(added_image);
            }
            image_cluster.edges.insert(std::make_pair(ViewIdPair(edge.src, edge.dst), edge.weight)); 
        }
        timer_.Pause();
        summary_.total_expansion_time += timer_.ElapsedSeconds();
        summary_.total_expansion_num++;

        // After graph expansion, there may be some image graphs that don't
        // satisfy the size constraint, thus we need to re-partition it
        LOG(INFO) << "Re-grouping clusters...";
        for (auto iter = inter_clusters_.begin(); iter != inter_clusters_.end(); ) {
            if (iter->image_ids.size() > options_.relax_ratio * options_.num_images_ub) {
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

std::vector<ImageCluster> ImageClustering::GetIntraClusters() const 
{
    return intra_clusters_;
}

std::vector<ImageCluster> ImageClustering::GetInterClusters() const
{
    return inter_clusters_;
}

ImageCluster ImageClustering::GetRootCluster() const
{
    return root_cluster_;
}

std::unique_ptr<Cluster> ImageClustering::CreateCluster() const
{
    if (options_.cluster_type == "NCUT") {
        return std::unique_ptr<Cluster>(new NCutCluster());
    } else if (options_.cluster_type == "KMEANS") {
        return std::unique_ptr<Cluster>(new KMeansCluster());
    } else if (options_.cluster_type == "SPECTRAL") {
        return std::unique_ptr<Cluster>(new SpectralCluster());
    }else {
        return std::unique_ptr<Cluster>(new NCutCluster());
    }
}

bool ImageClustering::IsSatisfyCompletenessRatio(const ImageCluster& cluster)
{
    if (cluster.is_condition_satisfy) return true;

    const uint i = cluster.cluster_id;
    std::unordered_set<image_t> image_sets(cluster.image_ids.begin(), 
                                           cluster.image_ids.end());

    uint repeated_node_num = 0;
    for (uint j = 0; j < inter_clusters_.size(); j++) {
        if (i == j) continue;
        const uint common_images_num = CommonImagesNum(inter_clusters_[i], inter_clusters_[j]);
        repeated_node_num += common_images_num;
    }

    // check if satisfy completeness ratio to avoid adding too many edges
    const float repeated_ratio = 
        (float)repeated_node_num / (float)(inter_clusters_[i].image_ids.size());
    if (repeated_ratio <= options_.completeness_ratio) {
        VLOG(2) << "repeated ratio: " << repeated_ratio;
        return false;
    } else {
        inter_clusters_[i].is_condition_satisfy = true;
        return true;
    }
}

int ImageClustering::ClusterSatisfyCompletenessRatio(const graph::Edge& edge)
{
    int cluster_id = -1;

    for (uint i = 0; i < inter_clusters_.size() - 1; i++) {
        std::unordered_set<image_t> image_sets(inter_clusters_[i].image_ids.begin(), 
                                               inter_clusters_[i].image_ids.end());
        if (image_sets.find(edge.src) == image_sets.end() &&
            image_sets.find(edge.dst) == image_sets.end()) {
                continue;
        }

        uint repeated_node_num = 0;
        for (uint j = 0; j < inter_clusters_.size(); j++) {
            if (i == j) continue;
            const uint common_images_num = CommonImagesNum(inter_clusters_[i], inter_clusters_[j]);
            // LOG(INFO) << "common images num: " << common_images_num;
            repeated_node_num += common_images_num;
        }

        // check if satisfy completeness ratio to avoid adding too many edges
        const float repeated_ratio = 
            (float)repeated_node_num / (float)(inter_clusters_[i].image_ids.size());
        if (repeated_ratio <= options_.completeness_ratio) {
            VLOG(2) << "repeated ratio: " << repeated_ratio;
            cluster_id = i;
            break;
        } else {
            // LOG(INFO) << "cluster " << i << " repeated ratio: " << repeated_ratio;
            // LOG(INFO) << "total clusters: " << inter_clusters_.size();
            inter_clusters_[i].is_condition_satisfy = true;
        }
    }

    return cluster_id;
}

uint ImageClustering::CommonImagesNum(const ImageCluster& cluster1, 
                                      const ImageCluster& cluster2) const
{
    uint common_images_num = 0;
    const std::unordered_set<image_t> images1(cluster1.image_ids.begin(),
                                              cluster1.image_ids.end());
    const std::unordered_set<image_t> images2(cluster2.image_ids.begin(),
                                              cluster2.image_ids.end());
    
    for (auto it = images1.begin(); it != images1.end(); ++it) {
        if (images2.find(*it) != images2.end()) {
            common_images_num++;
        }
    }
    // VLOG(2) << "common images num: " << common_images_num;
    return common_images_num;
}

void ImageClustering::OutputClusteringSummary() const
{
    LOG(INFO) << "Images Clustering Config:\n"
              << "- image upperbound: " << options_.num_images_ub << "\n"
              << "- completeness ratio: " << options_.completeness_ratio << "\n"
              << "- cluster type: " << options_.cluster_type << "\n"
              << "Images Clutering Summary:\n"
              << "Clusters number: " << inter_clusters_.size() << "\n"
              << "Total graph cutting time: "
              << summary_.total_cutting_time << " seconds\n"
              << "Total graph cutting number: "
              << summary_.total_cutting_num << "\n"
              << "Total graph expansion time: "
              << summary_.total_expansion_time << " seconds\n"
              << "Total graph expansion number: "
              << summary_.total_expansion_num << "\n"
              << "Total time took: "
              << summary_.total_time << " seconds\n"
              << "Total iteration number: "
              << summary_.total_iters_num << "\n"
              << "Images number expanded from " << summary_.original_images_num
              << " to " << summary_.clustered_images_num << "\n"
              << "Repeated Ratio: " 
              << (float)(summary_.clustered_images_num - summary_.original_images_num)
                 / (float)summary_.original_images_num << "\n"
              << "Edges number reduced from " << summary_.original_edges_num
              << " to " << summary_.clustered_edges_num << "\n"
              << "Lost ratio: " 
              << (float)(summary_.original_edges_num - summary_.clustered_edges_num)
                  / (float)summary_.original_edges_num;
}

bool ImageClustering::IsRemainingClusters() const
{
    for (auto cluster : inter_clusters_) {
        if (!cluster.is_condition_satisfy) return true;
    }
    return false;
}

void ImageClustering::AddLostEdgesBetweenClusters(ImageCluster& cluster1,
                                 ImageCluster& cluster2,
                                 std::vector<graph::Edge>& lost_edges)
{
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
    for (uint k = 0; k < lost_edges.size(); k++) {
        const ViewIdPair view_pair = lost_edges[k].src < lost_edges[k].dst
                                     ? ViewIdPair(lost_edges[k].src, lost_edges[k].dst)
                                     : ViewIdPair(lost_edges[k].dst, lost_edges[k].src);

        const std::unordered_set<image_t> images1(cluster1.image_ids.begin(), 
                                                  cluster1.image_ids.end());
        const std::unordered_set<image_t> images2(cluster2.image_ids.begin(),
                                                  cluster2.image_ids.end());
        
        const image_t added_image1 = images1.find(lost_edges[k].src) == images1.end()
                                     ? lost_edges[k].src : lost_edges[k].dst;
        const image_t added_image2 = images2.find(lost_edges[k].src) == images2.end()
                                     ? lost_edges[k].src : lost_edges[k].dst;

        int selected_image = cluster1.image_ids.size() > cluster2.image_ids.size() ? 2 : 1;
        if (selected_image == 1) {
            if (!IsSatisfyCompletenessRatio(cluster1) && 
                images1.find(added_image1) == images1.end()) {
                cluster1.image_ids.push_back(added_image1);
                cluster1.edges[view_pair] = lost_edges[k].weight;
            }
        } else {
            if (!IsSatisfyCompletenessRatio(cluster2)  && 
                images2.find(added_image2) == images2.end()) {
                cluster2.image_ids.push_back(added_image2);
                cluster2.edges[view_pair] = lost_edges[k].weight;
            }
        }
    }
}

void ImageClustering::AnalyzeStatistic()
{
    LOG(INFO) << "Analysing Statistics...";
    for (auto& cluster : inter_clusters_) {
        std::sort(cluster.image_ids.begin(), cluster.image_ids.end());
        summary_.clustered_images_num += cluster.image_ids.size();
        summary_.clustered_edges_num += cluster.edges.size();
    }
}

} // namespace GraphSfM