#include "controllers/distributed_mapper_controller.h"
#include "controllers/sfm_aligner.h"
#include "base/database.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/reconstruction_io.h"

#include <iostream>
#include <utility>
#include <vector>
#include <fstream>
#include <ceres/rotation.h>
#include <boost/filesystem.hpp>

using namespace colmap;

namespace GraphSfM {

bool DistributedMapperController::Options::Check() const
{
    CHECK_OPTION_GT(num_workers, -1);
    return true;
}

DistributedMapperController::DistributedMapperController(
    const Options& options,
    const ImageClustering::Options& clustering_options,
    const IncrementalMapperOptions& mapper_options,
    ReconstructionManager* reconstruction_manager)
    : options_(options),
      clustering_options_(clustering_options),
      mapper_options_(mapper_options),
      reconstruction_manager_(reconstruction_manager)
{
    CHECK(options.Check());
}

void DistributedMapperController::Run()
{
    //////////////////////////////////////////////////////////////////
    // 1. Partitioning the images into the given number of clusters //
    //////////////////////////////////////////////////////////////////
    PrintHeading1("Partitioning the Scene...");

    std::vector<std::pair<image_t, image_t>> image_pairs;
    std::vector<int> num_inliers;
    std::unordered_map<image_t, std::string> image_id_to_name;
    std::vector<image_t> image_ids;
    LoadData(image_pairs, num_inliers, image_id_to_name, image_ids);

    std::unique_ptr<ImageClustering> image_clustering_;
    std::vector<ImageCluster> inter_clusters = 
        ClusteringScenes(image_pairs, num_inliers, image_ids);

    // ////////////////////////////////////////////////////////////////
    // // 2. Reconstruct all clusters in parallel/distributed manner //
    // ////////////////////////////////////////////////////////////////
    PrintHeading1("Reconstucting Clusters...");

    std::unordered_map<const ImageCluster*, ReconstructionManager> reconstruction_managers;
    std::vector<Reconstruction*> reconstructions;
    ReconstructPartitions(image_id_to_name, inter_clusters, reconstruction_managers, reconstructions);    

    // ////////////////////////////////////////////////
    // // 3. Merge clusters ///////////////////////////
    // ////////////////////////////////////////////////
    PrintHeading1("Merging Clusters...");

    // Determine the number of workers and threads per worker
    const int kMaxNumThreads = -1;
    const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
    MergeClusters(inter_clusters, 
                  reconstructions, 
                  reconstruction_managers,
                  num_eff_threads);

    std::cout << std::endl;
    GetTimer().PrintMinutes();
}

BundleAdjustmentOptions DistributedMapperController::GlobalBundleAdjustment() const 
{
    BundleAdjustmentOptions options;
    options.solver_options.function_tolerance = 0.0;
    options.solver_options.gradient_tolerance = 1.0;
    options.solver_options.parameter_tolerance = 0.0;
    options.solver_options.max_num_iterations = 50;
    options.solver_options.max_linear_solver_iterations = 100;
    options.solver_options.minimizer_progress_to_stdout = true;
    options.solver_options.num_threads = -1;
#if CERES_VERSION_MAJOR < 2
    options.solver_options.num_linear_solver_threads = GetEffectiveNumThreads(-1);
#endif  // CERES_VERSION_MAJOR
    options.print_summary = true;
    options.refine_focal_length = true;
    options.refine_principal_point = false;
    options.refine_extra_params = true;
    options.loss_function_type =
        BundleAdjustmentOptions::LossFunctionType::TRIVIAL;
    return options;
}

bool DistributedMapperController::IsPartialReconsExist(std::vector<Reconstruction*>& recons) const
{
    const std::vector<std::string> dirs = colmap::GetRecursiveDirList(options_.output_path);
    recons.reserve(dirs.size());
    for (auto path : dirs) {
        Reconstruction* recon = new Reconstruction();
        if (ExistsFile(JoinPaths(path, "cameras.bin")) &&
            ExistsFile(JoinPaths(path, "images.bin")) &&
            ExistsFile(JoinPaths(path, "points3D.bin"))) {
            recon->ReadBinary(path);
        } else if (ExistsFile(JoinPaths(path, "cameras.txt")) &&
                   ExistsFile(JoinPaths(path, "images.txt")) &&
                   ExistsFile(JoinPaths(path, "points3D.txt"))) {
            recon->ReadText(path);
        } else {
            LOG(WARNING) << "cameras, images, points3D files do not exist at " << path;
            continue;
        }
        recons.push_back(recon);
    }

    if (recons.empty()) return false;
    return true;
}

void DistributedMapperController::LoadData(std::vector<std::pair<image_t, image_t>>& image_pairs,
              std::vector<int>& num_inliers,
              std::unordered_map<image_t, std::string>& image_id_to_name,
              std::vector<image_t>& image_ids)
{ 
    // Loading database
    Database database(options_.database_path);

    // Reading all images
    LOG(INFO) << "Reading images...";
    const auto images = database.ReadAllImages();
    for (const auto& image : images) {
        image_id_to_name.emplace(image.ImageId(), image.Name());
        image_ids.push_back(image.ImageId());
    }

    // Reading scene graph
    LOG(INFO) << "Reading scene graph...";
    CHECK_EQ(image_pairs.size(), num_inliers.size());
    database.ReadTwoViewGeometryNumInliers(&image_pairs, &num_inliers);
}

std::vector<ImageCluster> DistributedMapperController::ClusteringScenes(
    const std::vector<std::pair<image_t, image_t>>& image_pairs,
    const std::vector<int>& num_inliers,
    const std::vector<image_t>& image_ids)
{
    // Clustering images
    ImageCluster image_cluster;
    image_cluster.image_ids = image_ids;
    for (uint i = 0; i < image_pairs.size(); i++) {
        const ViewIdPair view_pair(image_pairs[i].first, image_pairs[i].second);
        image_cluster.edges[view_pair] = num_inliers[i];
    }

    image_clustering_ = std::unique_ptr<ImageClustering>(
                        new ImageClustering(clustering_options_, image_cluster));
    image_clustering_->Cut();
    image_clustering_->Expand();
    // image_clustering_.CutAndExpand();
    image_clustering_->OutputClusteringSummary();
    
    std::vector<ImageCluster> inter_clusters = image_clustering_->GetInterClusters();
    for (auto cluster : inter_clusters) {
        cluster.ShowInfo();
    }

    return inter_clusters;
}

void DistributedMapperController::ReconstructPartitions(
    const std::unordered_map<image_t, std::string>& image_id_to_name,
    std::vector<ImageCluster>& inter_clusters,
    std::unordered_map<const ImageCluster*, ReconstructionManager>& reconstruction_managers,
    std::vector<Reconstruction*>& reconstructions)
{
    // Determine the number of workers and threads per worker
    const int kMaxNumThreads = -1;
    const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
    const int kDefaultNumWorkers = 8;
    const int num_eff_workers = 
        options_.num_workers < 1
        ? std::min(static_cast<int>(inter_clusters.size()),
                   std::min(kDefaultNumWorkers, num_eff_threads))
        : options_.num_workers;
    const int num_threads_per_worker = 
        std::max(1, num_eff_threads / num_eff_workers);
    
    // Function to reconstruct one cluster using incremental mapping.
    // TODO: using different kind of mappers to reconstruct, such as global, hybrid
    auto ReconstructCluster = [&, this](
                            const ImageCluster& cluster,
                            ReconstructionManager* reconstruction_manager) {

        IncrementalMapperOptions custom_options = mapper_options_;
        custom_options.max_model_overlap = 3;
        custom_options.init_num_trials = options_.init_num_trials;
        custom_options.num_threads = num_threads_per_worker;

        for (const auto image_id : cluster.image_ids) {
            custom_options.image_names.insert(image_id_to_name.at(image_id));
        }

        IncrementalMapperController mapper(&custom_options, options_.image_path,
                                   options_.database_path,
                                   reconstruction_manager);
        mapper.Start();
        mapper.Wait();
    };

    // Start reconstructing the bigger clusters first for resource usage.
    const auto cmp = [](const ImageCluster& cluster1, const ImageCluster& cluster2) {
                  return cluster1.image_ids.size() > cluster2.image_ids.size(); };
    std::sort(inter_clusters.begin(), inter_clusters.end(), cmp);

    // Start the reconstruction workers.
    reconstruction_managers.reserve(inter_clusters.size());

    bool is_recons_exist = IsPartialReconsExist(reconstructions);
    if (is_recons_exist) {
        LOG(INFO) << "Loaded from previous reconstruction partitions.";
    } else {
        ThreadPool thread_pool(num_eff_workers);
        for (const auto& cluster : inter_clusters) {
            thread_pool.AddTask(ReconstructCluster, cluster, &reconstruction_managers[&cluster]);
        }
        thread_pool.Wait();

        for (const auto& cluster : inter_clusters) {
            auto& recon_manager = reconstruction_managers.at(&cluster);
            for (size_t i = 0; i < recon_manager.Size(); i++) {
                reconstructions.push_back(&recon_manager.Get(i));
            }
        }

        // Export un-transformed partial reconstructions for debugging.
        for (size_t i = 0; i < reconstructions.size(); ++i) {
            const std::string reconstruction_path = JoinPaths(options_.output_path, 
                                                              "partition_" + std::to_string(i));
            CreateDirIfNotExists(reconstruction_path);
            reconstructions[i]->Write(reconstruction_path);
        }
    }
}

void DistributedMapperController::MergeClusters(
    const std::vector<ImageCluster>& inter_clusters,
    std::vector<Reconstruction*>& reconstructions,
    std::unordered_map<const ImageCluster*, ReconstructionManager>& reconstruction_managers,
    const int num_eff_threads)
{
    LOG(INFO) << "Sub-reconstructions size: " << reconstructions.size();

    BundleAdjustmentOptions ba_options = this->GlobalBundleAdjustment();
    ba_options.solver_options.num_threads = num_eff_threads;

    SfMAligner sfm_aligner(reconstructions, ba_options);
    Node anchor_node;
    if (sfm_aligner.Align()) {
        anchor_node = sfm_aligner.GetAnchorNode();
    }
    CHECK_NE(anchor_node.id, -1);
    CHECK_NOTNULL(reconstructions[anchor_node.id]);

    LOG(INFO) << "Adding the final cluster...";
    LOG(INFO) << "Registered images number: " 
              << reconstructions[anchor_node.id]->RegImageIds().size();
    LOG(INFO) << "Reconstructed 3D points: "
              << reconstructions[anchor_node.id]->NumPoints3D();

    // Reading un-transformed reconstruction partitions.
    std::vector<Reconstruction*> trans_recons;
    CHECK_EQ(IsPartialReconsExist(trans_recons), true);

    std::vector<Sim3> sim3_to_anchor = sfm_aligner.GetSim3ToAnchor();
    for (uint i = 0; i < reconstructions.size(); i++) {
        if (i == anchor_node.id) continue;

        Sim3 sim3 = sim3_to_anchor[i];
        Eigen::Vector4d qvec;
        ceres::RotationMatrixToQuaternion(sim3.R.data(), qvec.data());
        SimilarityTransform3 tform(sim3.s, qvec, sim3.t);
        trans_recons[i]->Transform(tform);
    }

    // Insert a new reconstruction manager for merged cluster.
    const ImageCluster root_cluster = image_clustering_->GetRootCluster();
    auto& reconstruction_manager = reconstruction_managers[&root_cluster];
    reconstruction_manager.Add();
    reconstruction_manager.Get(reconstruction_manager.Size() - 1) = 
        *reconstructions[anchor_node.id];

    LOG(INFO) << "Erasing clusters...";
    for (const ImageCluster& inter_cluster : inter_clusters) {
        reconstruction_managers.erase(&inter_cluster); 
    }
    CHECK_EQ(reconstruction_managers.size(), 1);
    *reconstruction_manager_ = std::move(reconstruction_managers.begin()->second);
}

} // namespace GraphSfM