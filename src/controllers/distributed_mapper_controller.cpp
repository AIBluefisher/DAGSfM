#include "controllers/distributed_mapper_controller.h"

#include <iostream>
#include <utility>
#include <vector>
#include <fstream>
#include <memory>
#include <queue>
#include <mutex>
#include <unordered_set>
#include <unordered_map>

#include <omp.h>
#include <ceres/rotation.h>
#include <boost/filesystem.hpp>

#include "controllers/sfm_aligner.h"
#include "base/database.h"
#include "base/track_selection.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/timer.h"
#include "math/util.h"
#include "util/reconstruction_io.h"
#include "sfm/incremental_triangulator.h"

using namespace colmap;

#define __DEBUG__

namespace GraphSfM {
namespace {
void ExtractColors(const std::string& image_path, const image_t image_id,
                   Reconstruction* reconstruction)
{
    if (!reconstruction->ExtractColorsForImage(image_id, image_path)) {
        std::cout << StringPrintf("WARNING: Could not read image %s at path %s.",
                                  reconstruction->Image(image_id).Name().c_str(),
                                  image_path.c_str())
                  << std::endl;
    }
}


void ExtractColors(const std::string& image_path,
                   Reconstruction* reconstruction)
{
    const auto image_ids = reconstruction->RegImageIds();
    for (const auto image_id : image_ids) {
        if (!reconstruction->ExtractColorsForImage(image_id, image_path)) {
            std::cout << StringPrintf("WARNING: Could not read image %s at path %s.",
                                      reconstruction->Image(image_id).Name().c_str(),
                                      image_path.c_str())
                      << std::endl;
        }
    }
}
}

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
    // // 1. Loading data from database //////////////////////////////
    //////////////////////////////////////////////////////////////////
    PrintHeading1("Loading Data...");
    std::vector<Image> images;
    LoadData(image_pairs_, num_inliers_, image_id_to_name_, image_ids_);

    //////////////////////////////////////////////////////////////////
    // // 2. Extracting the largest connected component //////////////
    //////////////////////////////////////////////////////////////////
    if (options_.reconstruct_largest_cc) {
        PrintHeading1("Extracting Largest Connected Component...");
        LOG(INFO) << "Total image number: " << image_ids_.size();
        ExtractLargestCC();
        LOG(INFO) << "image number in largest cc: " << image_ids_.size();
    }

    //////////////////////////////////////////////////////////////////
    // 3. Partitioning the images into the given number of clusters //
    //////////////////////////////////////////////////////////////////
    PrintHeading1("Partitioning the Scene...");
    inter_clusters_ = ClusteringScenes(image_pairs_, num_inliers_, image_ids_);

    //////////////////////////////////////////////////////////////////
    // // 4. Distributed/Parallel Reconstruction /////////////////////
    //////////////////////////////////////////////////////////////////
    if (options_.distributed) {
        RunDistributed();
    } else {
        RunSequential();
    }

    //////////////////////////////////////////////////////////////////
    // // 5. Re-triangulate scene structures /////////////////////////
    //////////////////////////////////////////////////////////////////
    if (options_.retriangulate) {
        LOG(INFO) << "Re-triangulating...";
        Reconstruction& global_recon = reconstruction_manager_->Get(0);
        std::unique_ptr<IncrementalTriangulator> triangulator;
        triangulator.reset(new IncrementalTriangulator(
                global_recon.GetCorrespondenceGraph(), 
                &global_recon));
        IncrementalTriangulator::Options triangulator_options;
        triangulator->Retriangulate(triangulator_options);
    }

    //////////////////////////////////////////////////////////////////
    // // 6. Final global bundle adjustment //////////////////////////
    //////////////////////////////////////////////////////////////////
    if (options_.final_ba && (image_clustering_->GetInterClusters().size() > 1)) {
        LOG(INFO) << "Final Global Bundle Adjustment";
        Timer timer;
        timer.Start();
        this->AdjustGlobalBundle();
        timer.Pause();
        LOG(INFO) << "Final bundle adjustment took: " << timer.ElapsedSeconds() << " seconds.";
    }

    //////////////////////////////////////////////////
    //// 7. Extract colors ///////////////////////////
    //////////////////////////////////////////////////
    Reconstruction& reconstruction = 
        reconstruction_manager_->Get(reconstruction_manager_->Size() - 1);
    std::vector<image_t> reg_image_ids = reconstruction.RegImageIds();
    LOG(INFO) << "Extracting colors";
    for (auto image_id : reg_image_ids) {
        ExtractColors(options_.image_path, image_id, &reconstruction);
    }

    //////////////////////////////////////////////////
    //// 8. Optional: Partition Scenes for MVS ///////
    //////////////////////////////////////////////////
    if (options_.is_repartition_for_mvs) {
        LOG(INFO) << "Repartioning scenes for MVS";
        RepartitionScenesForMVS();
    }
}

void DistributedMapperController::Map(const void* input)
{

}

bool DistributedMapperController::RunSequential()
{
    ////////////////////////////////////////////////////////
    //// 1. Reconstruct all clusters in sequential manner //
    ////////////////////////////////////////////////////////
    PrintHeading1("Reconstucting Clusters...");
    std::unordered_map<const ImageCluster*, ReconstructionManager> reconstruction_managers;
    std::vector<Reconstruction*> reconstructions;
    ReconstructPartitions(image_id_to_name_, inter_clusters_, reconstruction_managers, reconstructions);    

    //////////////////////////////////////////////////
    //// 2. Merge clusters ///////////////////////////
    //////////////////////////////////////////////////
    PrintHeading1("Merging Clusters...");

    // Determine the number of workers and threads per worker
    const int kMaxNumThreads = -1;
    const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
    MergeClusters(inter_clusters_, 
                  reconstructions, 
                  reconstruction_managers,
                  num_eff_threads);

    return true;
}

bool DistributedMapperController::RunDistributed()
{
    /////////////////////////////////////////////////////////
    //// 1. Data preparation for local clusters /////////////
    /////////////////////////////////////////////////////////
    PrintHeading1("Reconstructing Clusters...");
    std::unordered_map<const ImageCluster*, std::unordered_set<std::string>> 
        cluster_reconstructed_imgs;
    std::unordered_map<const ImageCluster*, DatabaseCache> cluster_database_caches;

    cluster_reconstructed_imgs.reserve(inter_clusters_.size());
    cluster_database_caches.reserve(inter_clusters_.size());

    IncrementalMapperOptions local_mapper_options;
    for (size_t k = 0; k < inter_clusters_.size(); k++) {
        std::unordered_set<std::string> image_names;
        image_names.reserve(inter_clusters_[k].image_ids.size());

        for (const auto image_id : inter_clusters_[k].image_ids) {
            image_names.insert(image_id_to_name_.at(image_id));
        }
        cluster_database_caches[&inter_clusters_[k]].Load(database_, 
                                                         static_cast<size_t>(
                                                             local_mapper_options.min_num_matches), 
                                                         local_mapper_options.ignore_watermarks, 
                                                         image_names);
    }

    /////////////////////////////////////////////////////////
    //// 2. Reconstruct all clusters in distributed manner //
    /////////////////////////////////////////////////////////
    CHECK_GT(map_reduce_config_.server_ips.size(), 0);

    std::vector<Reconstruction*> reconstructions;
    std::queue<size_t> job_queue; // TODO: (chenyu) Consider to use a priority_queue.
    std::unordered_map<size_t, int> working_jobs; // { (job_id, worker_id) }.
    std::unordered_map<size_t, double> timings;
    size_t finished_jobs = 0;

    for (size_t i = 0; i < inter_clusters_.size(); i++) {
        job_queue.push(i);
    }

    auto IsTasksCompleteFunc = [&, this]() {
        while (finished_jobs != inter_clusters_.size()) {
            for (auto working_job : working_jobs) {
                const size_t job_id = working_job.first;
                const int worker_id = working_job.second;

                if (worker_infos_[worker_id].completed) {
                    inter_clusters_[job_id].completed = true;
                    finished_jobs++;
                    working_jobs.erase(job_id);

                    // Reset timers.
                    timers_[worker_id].Pause();
                    timings[job_id] = worker_infos_[worker_id].running_time;
                    
                    // Retrieve SfM data.
                    rpc::client c(map_reduce_config_.server_ips[worker_id],
                                  map_reduce_config_.server_ports[worker_id]);

                    auto local_recons_obj = c.call("GetLocalRecons");
                    Reconstruction recon = 
                        local_recons_obj.get().as<Reconstruction>();

                    Reconstruction* local_recon = new Reconstruction();         
                    local_recon->SetCameras(recon.Cameras());
                    local_recon->SetImages(recon.Images());
                    local_recon->SetPoints3D(recon.Points3D());
                    local_recon->SetRegImagesId(recon.RegImageIds());
                    local_recon->SetNumAddedPoints3D(recon.NumAddedPoints3D());
                    reconstructions.push_back(local_recon);

                    // Reset worker infomation.
                    worker_infos_[worker_id].Reset();
                    c.async_call("ResetWorkerInfo");
                }
            }
        }
    };

    // The task distribution algorithm is simply based on
    // some isomorphic servers.
    auto DistributeTasksFunc = [&, this]() {
        while (!job_queue.empty() && (finished_jobs != inter_clusters_.size())) {
            int idle_server_id = GetIdleWorker();
            if (idle_server_id == -1) {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                continue;
            }

            rpc::client c(map_reduce_config_.server_ips[idle_server_id],
                          map_reduce_config_.server_ports[idle_server_id]);
            c.call("SetNonIdle");

            // Remote async calling for local Structure-from-Motion.
            size_t job_id = job_queue.front(); job_queue.pop();

            // Transfer images to worker server.
            std::unordered_set<std::string> image_names;
            image_names.reserve(inter_clusters_[job_id].image_ids.size());
            for (const auto image_id : inter_clusters_[job_id].image_ids) {
                image_names.insert(image_id_to_name_.at(image_id));
            }
            
            if (options_.transfer_images_to_server) {
                LOG(INFO) << "Transferring images to cluster #" << idle_server_id << "#.";
                CallSaveImages(
                    idle_server_id, 
                    options_.image_path,
                    colmap::JoinPaths(map_reduce_config_.image_paths[idle_server_id],
                                      "cluster" + std::to_string(job_id)), 
                    image_names);
            }

            c.async_call("RunSfM", cluster_database_caches[&inter_clusters_[job_id]]);
            timers_[idle_server_id].Start();
            working_jobs.insert({job_id, idle_server_id});

            CallGetRunningInfo(idle_server_id, true);
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    };

    auto MonitorFunc = [&](){
        // Monitoring the running status of each cluster.
        do {
            ShowProgress();
            UpdateRunningInfo(true);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (finished_jobs != inter_clusters_.size());
    };

    std::thread distribute_task_thread{DistributeTasksFunc};
    std::thread check_complete_thread{IsTasksCompleteFunc};
    std::thread monitor_thread{MonitorFunc};

    distribute_task_thread.join();
    check_complete_thread.join();
    monitor_thread.join();

    // Stop all workers.
    for (size_t i = 0; i < map_reduce_config_.server_ips.size(); i++) {
        rpc::client c(map_reduce_config_.server_ips[i],
                      map_reduce_config_.server_ports[i]);
        c.async_call("StopServer");
    }

    //////////////////////////////////////////////////
    //// 3. Merge clusters ///////////////////////////
    //////////////////////////////////////////////////
    
    // Determine the number of workers and threads per worker
    LOG(INFO) << "Sub-reconstructions size: " << reconstructions.size();
    #ifdef __DEBUG__
        LOG(INFO) << "Extracting colors for local maps.";
        for (Reconstruction* reconstruction : reconstructions) {
            ExtractColors(options_.image_path, reconstruction);
        }
        ExportUntransformedLocalRecons(reconstructions);
        LOG(INFO) << "Local reconstructions are exported to "
                       << options_.output_path;
    #endif

    const int kMaxNumThreads = -1;
    const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);

    PrintHeading1("Merging Clusters...");
    Node anchor_node;
    MergeClusters(reconstructions, num_eff_threads, anchor_node);

    reconstructions[anchor_node.id]->ShowReconInfo();
    reconstruction_manager_->Add();
    reconstruction_manager_->Get(0) = *reconstructions[anchor_node.id];

    for (size_t i = 0; i < reconstructions.size(); i++) {
        if (static_cast<int>(i) == anchor_node.id) continue;
        delete reconstructions[i];
        reconstructions[i] = nullptr;
    }

    return true;
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
    // Loading database.
    database_.Open(options_.database_path);

    // Reading all images
    LOG(INFO) << "Reading images...";
    const auto images = database_.ReadAllImages();
    for (const auto& image : images) {
        image_id_to_name.emplace(image.ImageId(), image.Name());
        image_ids.push_back(image.ImageId());
    }

    // Reading scene graph
    LOG(INFO) << "Reading scene graph...";
    CHECK_EQ(image_pairs.size(), num_inliers.size());
    database_.ReadTwoViewGeometryNumInliers(&image_pairs, &num_inliers);
}

void DistributedMapperController::ExtractLargestCC()
{
    graph::UnionFind uf(image_ids_.size());
    std::vector<size_t> tmp_nodes(image_ids_.begin(), image_ids_.end());
    uf.InitWithNodes(tmp_nodes);

    for (auto image_pair : image_pairs_) {
        uf.Union(image_pair.first, image_pair.second);
    }

    std::unordered_map<size_t, std::vector<image_t>> components;
    for (auto image_id : image_ids_) {
        const size_t parent_id = uf.FindRoot(image_id);
        components[parent_id].push_back(image_id);
    }

    size_t num_largest_component = 0;
    size_t largest_component_id;
    for (const auto& it : components) {
        if (num_largest_component < it.second.size()) {
            num_largest_component = it.second.size();
            largest_component_id = it.first;
        }
    }

    image_ids_.clear();
    image_ids_.assign(components[largest_component_id].begin(),
                      components[largest_component_id].end());
    image_ids_.shrink_to_fit();
    std::sort(image_ids_.begin(), image_ids_.end());

    LOG(INFO) << "There are " << components.size() << " connected components.";
    int num_small_connected_components = 0;
    for (auto component : components) {
        if (component.second.size() < 100) { 
            num_small_connected_components++; 
        } else {
            LOG(INFO) << "Component #" << component.first << "# has " 
                  << component.second.size() << " images.";
        }
    }
    LOG(INFO) << "There are " << num_small_connected_components
              << "small connected components are discarded.";
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
    
    // Function to reconstruct one cluster using incremental mapping.
    // TODO: using different kind of mappers to reconstruct, such as global, hybrid
    auto ReconstructCluster = [&, this](
                            const ImageCluster& cluster,
                            ReconstructionManager* reconstruction_manager) {

        IncrementalMapperOptions custom_options = mapper_options_;
        custom_options.max_model_overlap = 3;
        custom_options.init_num_trials = options_.init_num_trials;
        custom_options.num_threads = num_eff_threads;

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
    std::sort(inter_clusters_.begin(), inter_clusters_.end(), cmp);

    // Start the reconstruction workers.
    reconstruction_managers.reserve(inter_clusters_.size());

    bool is_recons_exist = IsPartialReconsExist(reconstructions);
    if (is_recons_exist) {
        LOG(INFO) << "Loaded from previous reconstruction partitions.";
    } else {
        // #pragma omp parallel for
        for (size_t k = 0; k < inter_clusters_.size(); k++) {
            const auto& cluster = inter_clusters[k];
            std::thread local_sfm_thread(ReconstructCluster, 
                             cluster, 
                             &reconstruction_managers[&cluster]);
            local_sfm_thread.join();
        }

        for (const auto& cluster : inter_clusters) {
            auto& recon_manager = reconstruction_managers.at(&cluster);
            for (size_t i = 0; i < recon_manager.Size(); i++) {
                reconstructions.push_back(&recon_manager.Get(i));
            }
        }
    #ifdef __DEBUG__
        LOG(INFO) << "Extracting colors for local maps.";
        for (Reconstruction* reconstruction : reconstructions) {
            ExtractColors(options_.image_path, reconstruction);
        }
        ExportUntransformedLocalRecons(reconstructions);
        LOG(INFO) << "Local reconstructions are exported to "
               << options_.output_path;
    #endif
    }
}

void DistributedMapperController::MergeClusters(std::vector<Reconstruction*>& reconstructions,
                                                const int num_eff_threads,
                                                Node& anchor_node)
{
    if (reconstructions.size() > 1) {
        SfMAligner::AlignOptions align_options;
        SfMAligner sfm_aligner(reconstructions, align_options);

        if (sfm_aligner.Align()) {
            anchor_node = sfm_aligner.GetAnchorNode();
        }

        if (anchor_node.id == -1) {
            LOG(INFO) << "Extracting colors for local maps.";
            for (auto& reconstruction : reconstructions) {
                ExtractColors(options_.image_path, reconstruction);
            }
            LOG(ERROR) << "Align failed, local reconstructions are exported to "
                       << options_.output_path;
            return;
        }

        CHECK_NOTNULL(reconstructions[anchor_node.id]);

        #ifdef __DEBUG__
            // Reading un-transformed reconstruction partitions.
            std::vector<Reconstruction*> trans_recons;
            CHECK_EQ(IsPartialReconsExist(trans_recons), true);

            std::vector<Sim3> sim3_to_anchor = sfm_aligner.GetSim3ToAnchor();
            for (uint i = 0; i < reconstructions.size(); i++) {
                if (static_cast<int>(i) == anchor_node.id) continue;

                Sim3 sim3 = sim3_to_anchor[i];
                Eigen::Vector4d qvec;
                ceres::RotationMatrixToQuaternion(sim3.R.data(), qvec.data());
                SimilarityTransform3 tform(sim3.s, qvec, sim3.t);
                trans_recons[i]->Transform(tform);
            }
        #endif
    } else { // Only one cluster.
        anchor_node.id = 0;
    }

    // Assign cluster id for each image.
    const std::vector<ImageCluster> intra_clusters = image_clustering_->GetIntraClusters();
    for (size_t i = 0; i < intra_clusters.size(); i++) {
        const ImageCluster intra_cluster = intra_clusters[i];
        const std::vector<image_t> image_ids = intra_cluster.image_ids;
        
        for (auto image_id : image_ids) {
            if (reconstructions[anchor_node.id]->ExistsImage(image_id)) {
                Image& image = reconstructions[anchor_node.id]->Image(image_id);
                image.SetClusterId(i);
            }
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
    Node anchor_node;
    MergeClusters(reconstructions, num_eff_threads, anchor_node);

    // std::unique_ptr<IncrementalTriangulator> triangulator;
    // triangulator.reset(new IncrementalTriangulator(
    //         reconstructions[anchor_node.id]->GetCorrespondenceGraph(), 
    //         reconstructions[anchor_node.id]));
    // IncrementalTriangulator::Options triangulator_options;
    // triangulator->Retriangulate(triangulator_options);

    LOG(INFO) << "Adding the final cluster...";
    reconstructions[anchor_node.id]->ShowReconInfo();

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

void DistributedMapperController::RepartitionScenesForMVS()
{
    const Reconstruction reconstruction = 
        reconstruction_manager_->Get(reconstruction_manager_->Size() - 1);
    std::vector<image_t> reg_image_ids = reconstruction.RegImageIds();
    const double epsilon = 0.9;

    // Image projection centers are used to compute the weight
    std::vector<Eigen::Vector3d> reg_image_centers;
    reg_image_centers.reserve(reg_image_ids.size());
    for (auto image_id : reg_image_ids) {
        reg_image_centers.push_back(reconstruction.Image(image_id).ProjectionCenter());
    }

    ImageCluster root_cluster;
    root_cluster.image_ids = reg_image_ids;
    for (uint i = 0; i < reg_image_centers.size(); i++) {
        for (uint j = i + 1; j < reg_image_centers.size(); j++) {
            const double diff = (reg_image_centers[i] - reg_image_centers[j]).norm();
            if (diff > epsilon) continue;
            const ViewIdPair view_pair = reg_image_ids[i] > reg_image_ids[j] ?
                                         ViewIdPair(reg_image_ids[j], reg_image_ids[i]) :
                                         ViewIdPair(reg_image_ids[i], reg_image_ids[j]);
            root_cluster.edges[view_pair] = diff < 1e-6 ? 10000 : 10 / diff;
        }
    }

    clustering_options_.cluster_type = "NCUT";
    ImageClustering image_cluster(clustering_options_, root_cluster);
    image_cluster.Cut();
    std::vector<ImageCluster> intra_clusters = image_cluster.GetIntraClusters();
    for (uint i = 0; i < intra_clusters.size(); i++) {
        const std::string reconstruction_path = JoinPaths(options_.output_path, 
                                                  "tpartition_" + std::to_string(i));
        CreateDirIfNotExists(reconstruction_path);

        EIGEN_STL_UMAP(camera_t, class Camera) cameras = reconstruction.Cameras();;
        EIGEN_STL_UMAP(image_t, class Image) images;
        EIGEN_STL_UMAP(point3D_t, class Point3D) points3D;

        const std::vector<image_t> image_ids = intra_clusters[i].image_ids;
        for (auto image_id : image_ids) {
            images[image_id] = reconstruction.Image(image_id);
            for (const Point2D point2D : images[image_id].Points2D()) {
                if (point2D.HasPoint3D()) {
                    points3D[point2D.Point3DId()] = reconstruction.Point3D(point2D.Point3DId());
                }
            }
        }
        
        WriteCamerasBinary(cameras, JoinPaths(reconstruction_path, "cameras.bin"));
        WriteImagesBinary(images, JoinPaths(reconstruction_path, "images.bin"));
        WritePoints3DBinary(points3D, JoinPaths(reconstruction_path, "points3D.bin"));
    }
}

bool DistributedMapperController::AdjustGlobalBundle()
{
    BundleAdjustmentOptions ba_options = this->GlobalBundleAdjustment();
    // ba_options.solver_options.num_threads = num_eff_threads;

    Reconstruction& global_recon = reconstruction_manager_->Get(0);
    
    const std::vector<image_t>& reg_image_ids = global_recon.RegImageIds();

    CHECK_GE(reg_image_ids.size(), 2) << "At least two images must be "
                                         "registered for global bundle-adjustment";
    
    // Avoid degeneracies in bundle adjustment.
    global_recon.FilterObservationsWithNegativeDepth();

    // Configure bundle adjustment
    BundleAdjustmentConfig ba_config;
    for (const image_t image_id : reg_image_ids) {
        ba_config.AddImage(image_id);
    }

    // Fix 7-DOFs of the bundle adjustment problem.
    ba_config.SetConstantPose(reg_image_ids[0]);
    ba_config.SetConstantTvec(reg_image_ids[1], {0});

    EIGEN_STL_UMAP(point3D_t, Point3D) points3d = global_recon.Points3D();
    std::unordered_set<point3D_t> tracks_to_optimize;
    if (options_.select_tracks_for_bundle_adjustment) {
        SelectGoodTracksForBundleAdjustment(global_recon,
                                            options_.long_track_length_threshold,
                                            options_.image_grid_cell_size_pixels,
                                            options_.min_num_optimized_tracks_per_view,
                                            &tracks_to_optimize);
        
        for (const auto& point3d : points3d) {
            if (tracks_to_optimize.count(point3d.first) != 0) {
                ba_config.AddVariablePoint(point3d.first);
            } else {
                ba_config.AddConstantPoint(point3d.first);
            }
        }
    } else {
        for (const auto& point3d : points3d) {
            ba_config.AddVariablePoint(point3d.first);
        }
    }

    // Run bundle adjustment.
    BundleAdjuster bundle_adjuster(ba_options, ba_config);
    if (!bundle_adjuster.Solve(&global_recon)) {
        return false;
    }

    LOG(INFO) << "Selected " << tracks_to_optimize.size() << " to optimize.";
    LOG(INFO) << "Total tracks: " << points3d.size();

    // Normalize scene for numerical stability and
    // to avoid large scale changes in viewer.
    global_recon.Normalize();

    return true;
}

void DistributedMapperController::ExportUntransformedLocalRecons(
    const std::vector<Reconstruction*>& reconstructions) const
{
    for (size_t i = 0; i < reconstructions.size(); ++i) {
        const std::string reconstruction_path = JoinPaths(options_.output_path, 
                                                              "partition_" + std::to_string(i));
        CreateDirIfNotExists(reconstruction_path);
        reconstructions[i]->Write(reconstruction_path);
    }
}

} // namespace GraphSfM