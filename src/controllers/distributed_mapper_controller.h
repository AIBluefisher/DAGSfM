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

using namespace colmap;

namespace GraphSfM {

// Distributed mapping for very large scale structure from motion.
// This mapper first partition images into the given number of clusters, 
// then reconstruct each cluster with incremental/global/hybrid SfM approaches.
// At last, all clusters are aligned together using a robust graph-based aligner.
// A very large scale bundle adjustment should be performed after the alignment.
class DistributedMapperController : public Thread
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

        // The maximum number of trials to initialize a cluster.
        int init_num_trials = 10;

        // The number of workers used to reconstruct clusters in parallel
        int num_workers = -1;

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

    void MergeClusters(
        const std::vector<ImageCluster>& inter_clusters,
        std::vector<Reconstruction*>& reconstructions,
        std::unordered_map<const ImageCluster*, ReconstructionManager>& reconstruction_managers,
        const int num_eff_threads);

    const Options options_;

    ImageClustering::Options clustering_options_;

    std::unique_ptr<ImageClustering> image_clustering_;

    const IncrementalMapperOptions mapper_options_;

    ReconstructionManager* reconstruction_manager_;
};

} // namespace GraphSfM

#endif