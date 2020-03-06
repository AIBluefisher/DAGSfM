#ifndef SRC_CONTROLLERS_DISTRIBUTED_MAPPER_CONTROLLER_H_
#define SRC_CONTROLLERS_DISTRIBUTED_MAPPER_CONTROLLER_H_

#include <string>
#include <memory>

#include "util/threading.h"
#include "base/reconstruction_manager.h"
#include "clustering/image_clustering.h"
#include "clustering/scene_clustering.h"
#include "controllers/incremental_mapper_controller.h"
#include "clustering/image_clustering.h"
#include "sfm/types.h"
#include "graph/graph.h"
#include "map_reduce/master.h"
#include "map_reduce/mapper.h"

using namespace colmap;

namespace GraphSfM {

// Distributed mapping for very large scale structure from motion.
// This mapper first partition images into the given number of clusters, 
// then reconstruct each cluster with incremental/global/hybrid SfM approaches.
// At last, all clusters are aligned together using a robust graph-based aligner.
// A very large scale bundle adjustment should be performed after the alignment.
class DistributedMapperController : public Thread, public SfMMaster, public Mapper
{
public:
    struct Options
    {
        // the path to the image folder which are used as input
        std::string image_path;

        // The path to store reconstructions.
        std::string output_path;

        // The path to the database file which is used as input
        std::string database_path;

        double max_relative_rotation_difference_degrees = 5.0;

        // minimum number of feature matches in an epipolar edge
        uint32_t minimum_edge_inlier = 15;

        // minimum ransac
        uint32_t minimum_ransac_times = 256;

        // The maximum number of trials to initialize a cluster.
        int init_num_trials = 10;

        // The number of workers used to reconstruct clusters in parallel
        int num_workers = -1;

        // Determine if repartition after merging for latter large scale mvs
        bool is_repartition_for_mvs = false;

        // Determin if run in distributed mode or sequential mode.
        bool distributed = false;

        // if assign cluster id for each image.
        bool assign_cluster_id = false;

        // output format.
        bool write_binary = true;

        bool Check() const;
    };
    
    DistributedMapperController(
        const Options& options,
        const ImageClustering::Options& clustering_options,
        const IncrementalMapperOptions& mapper_options,
        ReconstructionManager* reconstruction_manager);

    BundleAdjustmentOptions GlobalBundleAdjustment() const;

private:
    void Run() override;

    virtual void Map(const void* input) override;

    virtual bool RunSequential() override;

    virtual bool RunDistributed() override;

    bool IsPartialReconsExist(std::vector<Reconstruction*>& recons) const;

    void LoadData(std::vector<std::pair<image_t, image_t>>& image_pairs,
                  std::vector<int>& num_inliers,
                  std::unordered_map<image_t, std::string>& image_id_to_name,
                  std::vector<image_t>& image_ids);

    std::vector<ImageCluster> ClusteringScenes(
        const std::vector<std::pair<image_t, image_t>>& image_pairs,
        const std::vector<int>& num_inliers,
        const std::vector<image_t>& image_ids);

    void ReconstructPartitions(
        const std::unordered_map<image_t, std::string>& image_id_to_name,
        std::vector<ImageCluster>& inter_clusters,
        std::unordered_map<const ImageCluster*, ReconstructionManager>& reconstruction_managers,
        std::vector<Reconstruction*>& reconstructions);

    void MergeClusters(std::vector<Reconstruction*>& reconstructions,
                       const int num_eff_threads,
                       graph::Node& anchor_node);

    void MergeClusters(
        const std::vector<ImageCluster>& inter_clusters,
        std::vector<Reconstruction*>& reconstructions,
        std::unordered_map<const ImageCluster*, ReconstructionManager>& reconstruction_managers,
        const int num_eff_threads);

    void RepartitionScenesForMVS();

    void ExportUntransformedLocalRecons(
        const std::vector<Reconstruction*>& reconstructions) const;

    const Options options_;

    ImageClustering::Options clustering_options_;

    std::unique_ptr<ImageClustering> image_clustering_;

    const IncrementalMapperOptions mapper_options_;

    Database database_;

    ReconstructionManager* reconstruction_manager_;

    std::unordered_map<image_t, Image> images_;

    std::unordered_map<camera_t, Camera> cameras_;

    // Global rotations.
    // std::unordered_map<ViewId, Eigen::Vector3d> rotations_;

    std::unordered_map<ViewIdPair, TwoViewGeometry> view_graph_;

    // Required data for images clustering and distributed/parallel SfM.
    std::vector<std::pair<image_t, image_t>> image_pairs_;
    std::vector<int> num_inliers_;
    std::unordered_map<image_t, std::string> image_id_to_name_;
    std::vector<image_t> image_ids_;
    // std::unordered_map<ViewIdPair, TwoViewInfo> view_pairs_;
    std::vector<ImageCluster> inter_clusters_;
};

} // namespace GraphSfM

#endif