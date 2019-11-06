#ifndef SRC_CLUSTERING_IMAGE_CLUSTERING_H_
#define SRC_CLUSTERING_IMAGE_CLUSTERING_H_

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <queue>
#include <memory>

#include "sfm/types.h"
#include "graph/graph.h"
#include "clustering/cluster.h"
#include "util/types.h"
#include "util/hash.h"
#include "util/timer.h"

using namespace colmap;

namespace GraphSfM {

using Edges = std::unordered_map<ViewIdPair, int>;
struct ImageCluster
{
    int cluster_id;
    std::vector<image_t> image_ids;
    Edges edges;
    bool is_condition_satisfy = false;

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

    void ShowInfo() const
    {
        LOG(INFO) << image_ids.size() << " nodes";
        for (auto image_id : image_ids) {
            std::cout << image_id << " ";
        }
        std::cout << "\n";

        LOG(INFO) << edges.size() << " edges";
    }
};

class ImageClustering
{
public:
    struct Options
    {
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

        std::string cluster_type = "NCUT";

        std::string graph_dir = "";

        bool Check() const;
    };

    struct Summary
    {
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

    ImageClustering(const Options& options,
                    const ImageCluster& root_cluster);

    virtual void Cut();

    virtual void Expand();

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
    std::unordered_map<ViewIdPair, std::vector<graph::Edge>> clusters_lost_edges_;

    std::unique_ptr<Cluster> CreateCluster() const;


    bool IsSatisfyCompletenessRatio(const ImageCluster& cluster);
    int ClusterSatisfyCompletenessRatio(const graph::Edge& edge);

    uint CommonImagesNum(const ImageCluster& cluster1, const ImageCluster& cluster2) const;

    bool IsRemainingClusters() const;

    void AddLostEdgesBetweenClusters(ImageCluster& cluster1,
                                     ImageCluster& cluster2,
                                     std::vector<graph::Edge>& lost_edges);

    void AnalyzeStatistic();
};

} // namespace GraphSfM

#endif