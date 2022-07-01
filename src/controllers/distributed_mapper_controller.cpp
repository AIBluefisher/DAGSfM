#include "controllers/distributed_mapper_controller.h"

#include <ceres/rotation.h>
#include <omp.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base/database.h"
#include "base/database_info.h"
#include "base/track_selection.h"
#include "controllers/sfm_aligner.h"
#include "map_reduce/distributed_task_manager.h"
#include "math/util.h"
#include "rotation_estimation/rotation_estimator.h"
#include "sfm/filter_view_pairs_from_orientation.h"
#include "sfm/incremental_triangulator.h"
#include "util/hash.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/reconstruction_io.h"
#include "util/timer.h"

using namespace colmap;

#define __DEBUG__

namespace DAGSfM {
namespace {

void ExtractColors(const std::string& image_path,
                   Reconstruction* reconstruction) {
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

bool VerifyCameraParams(const std::string& camera_model,
                        const std::string& params) {
  if (!ExistsCameraModelWithName(camera_model)) {
    std::cerr << "ERROR: Camera model does not exist" << std::endl;
    return false;
  }

  const std::vector<double> camera_params = CSVToVector<double>(params);
  const int camera_model_id = CameraModelNameToId(camera_model);

  if (camera_params.size() > 0 &&
      !CameraModelVerifyParams(camera_model_id, camera_params)) {
    std::cerr << "ERROR: Invalid camera parameters" << std::endl;
    return false;
  }
  return true;
}
}  // namespace

bool DistributedMapperController::Options::Check() const {
  CHECK_OPTION_GT(num_workers, -1);
  return true;
}

DistributedMapperController::DistributedMapperController(
    const Options& options,
    const VocabSimilaritySearchOptions& similarity_search_options,
    const SiftExtractionOptions& extraction_options,
    const ImageClustering::Options& clustering_options,
    const IncrementalMapperOptions& mapper_options,
    ReconstructionManager* reconstruction_manager)
    : options_(options),
      clustering_options_(clustering_options),
      mapper_options_(mapper_options),
      database_(options_.database_path),
      reconstruction_manager_(reconstruction_manager),
      similarity_graph_(similarity_search_options, database_) {
  CHECK(options.Check());
}

void DistributedMapperController::SetMapReduceConfig(
    const MapReduceConfig& map_reduce_config) {
  map_reduce_config_ = map_reduce_config;
}

void DistributedMapperController::Run() {

  Timer timer;
  double load_twoview_geometries_time = 0.0;
  double indexing_time = 0.0;
  double matching_time = 0.0;
  double extract_largest_cc_time = 0.0;
  double ra_time = 0.0;
  double cluster_time = 0.0;
  double sfm_time = 0.0;
  double triangulate_time = 0.0;
  double ba_time = 0.0;

  // Load existed database.
  timer.Start();
  bool load_exist_database = LoadTwoviewGeometries();
  timer.Pause();
  load_twoview_geometries_time = timer.ElapsedSeconds();

  //////////////////////////////////////////////////////////////////
  // //  1. Indexing images by similarity search ///////////////////
  //////////////////////////////////////////////////////////////////
  if (!load_exist_database) {
    timer.Start();
    similarity_graph_.Start();
    similarity_graph_.Wait();
    timer.Pause();

    indexing_time = timer.ElapsedSeconds();
  }

  // 2. Distributed/Sequential Feature Extraction and Matching.
  if (!load_exist_database) {
    timer.Start();
    if (options_.distributed) {
      DistributedFeatureExtractionAndMatching();
    } else {
      SequentialFeatureExtractionAndMatching();
    }
    timer.Pause();

    matching_time = timer.ElapsedSeconds();
  }

  //////////////////////////////////////////////////////////////////
  // // 3. Load Two view geometries from match inliers. ////////////
  //////////////////////////////////////////////////////////////////
  if (!load_exist_database) {
    LoadTwoviewGeometries();
  }

  if (options_.reconstruct_largest_cc) {
    PrintHeading1("Extracting Largest Connected Component...");
    LOG(INFO) << "Total image number: " << view_graph_.ImageIds().size();
    timer.Start();
    view_graph_.FilterViewGraphCyclesByRotation(5.0);
    view_graph_.TwoviewGeometriesToImagePairs();
    view_graph_.ExtractLargestCC();
    timer.Pause();
    extract_largest_cc_time = timer.ElapsedSeconds();
    LOG(INFO) << "image number in largest cc: "
              << view_graph_.ImageIds().size();
  }

  //////////////////////////////////////////////////////////////////
  // // 4. Global Rotation Averaging ///////////////////////////////
  //////////////////////////////////////////////////////////////////
  PrintHeading1("Global Rotation Averaging...");
  timer.Start();
  GlobalRotationAveraging();
  timer.Pause();
  ra_time = timer.ElapsedSeconds();

  //////////////////////////////////////////////////////////////////
  // 5. Partitioning the images into the given number of clusters //
  //////////////////////////////////////////////////////////////////
  PrintHeading1("Partitioning the Scene...");
  timer.Start();
  ClusteringScenes();
  timer.Pause();
  cluster_time = timer.ElapsedSeconds();

  //////////////////////////////////////////////////////////////////
  // // 6. Distributed/Parallel Reconstruction /////////////////////
  //////////////////////////////////////////////////////////////////
  timer.Start();
  if (options_.distributed) {
    DistributedSfM();
  } else {
    SequentialSfM();
  }
  timer.Pause();
  sfm_time = timer.ElapsedSeconds();

  //////////////////////////////////////////////////////////////////
  // // 7. Re-triangulate scene structures /////////////////////////
  //////////////////////////////////////////////////////////////////
  if (options_.retriangulate) {
    LOG(INFO) << "Triangulating for separators...";
    timer.Start();
    Triangulate();
    timer.Pause();
    triangulate_time = timer.ElapsedSeconds();
  }

  //////////////////////////////////////////////////////////////////
  // // 8. Final global bundle adjustment //////////////////////////
  //////////////////////////////////////////////////////////////////
  if (options_.final_ba && (image_clustering_->GetInterClusters().size() > 1)) {
    LOG(INFO) << "Final Global Bundle Adjustment";

    timer.Start();
    this->AdjustGlobalBundle();
    timer.Pause();
    ba_time = timer.ElapsedSeconds();
  }

  LOG(INFO) << "Time elapsed: \n"
            << "  -[load two view geometries]: " << load_twoview_geometries_time << " seconds.\n"
            << "  -[image indexing]: " << indexing_time << " seconds.\n"
            << "  -[extraction + matching]: " << matching_time << " seconds.\n"
            << "  -[Largest connected components extraction]: " << extract_largest_cc_time << " seconds.\n"
            << "  -[Rotation Averaging]: " << ra_time << " seconds.\n"
            << "  -[Image Clustering]: " << cluster_time << " seconds.\n"
            << "  -[SfM]: " << sfm_time << " seconds.\n"
            << "  -[Triangulate]: " << triangulate_time << " seconds.\n"
            << "  -[Bundle Adjustment]: " << ba_time << " seconds.\n";
  LOG(INFO) << "Reconstruction size: " << reconstruction_manager_->Size();
  reconstruction_manager_->Get(0).ShowReconInfo();
}

bool DistributedMapperController::SequentialSfM() {
  ////////////////////////////////////////////////////////
  //// 1. Reconstruct all clusters in sequential manner //
  ////////////////////////////////////////////////////////
  PrintHeading1("Reconstructing Clusters...");
  std::unordered_map<size_t, ReconstructionManager> reconstruction_managers;
  std::vector<Reconstruction*> reconstructions;
  ReconstructPartitions(reconstruction_managers, reconstructions);

  //////////////////////////////////////////////////
  //// 2. Merge clusters ///////////////////////////
  //////////////////////////////////////////////////
  PrintHeading1("Merging Clusters...");

  // Determine the number of workers and threads per worker
  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  MergeClusters(reconstructions, reconstruction_managers, num_eff_threads);

  return true;
}

bool DistributedMapperController::DistributedSfM() {
  Timer timer;
  /////////////////////////////////////////////////////////
  //// 1. Data preparation for local clusters /////////////
  /////////////////////////////////////////////////////////
  PrintHeading1("Reconstructing Clusters...");
  std::unique_ptr<std::unordered_map<size_t, DatabaseCache>> 
                  cluster_database_caches(new std::unordered_map<size_t, DatabaseCache>());
  std::unordered_map<size_t, std::vector<std::string>> cluster_images;
  const std::unordered_map<image_t, std::string>& image_id_to_name =
      view_graph_.ImageIdToName();

  cluster_database_caches->reserve(inter_clusters_.size());
  cluster_images.reserve(inter_clusters_.size());

  IncrementalMapperOptions local_mapper_options;
  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    std::vector<std::string> image_name_list;
    image_name_list.reserve(inter_clusters_[k].ImageIdsSize());

    for (const auto image_id : inter_clusters_[k].ImageIds()) {
      image_name_list.push_back(image_id_to_name.at(image_id));
    }
    std::sort(image_name_list.begin(), image_name_list.end());
    LOG(INFO) << "cluster " << k << " has " << image_name_list.size()
              << " images.";
    cluster_images.emplace(k, image_name_list);

    std::unordered_set<std::string> image_name_set(image_name_list.begin(),
                                                   image_name_list.end());
    (*cluster_database_caches)[k].Load(
        database_, static_cast<size_t>(local_mapper_options.min_num_matches),
        local_mapper_options.ignore_watermarks, image_name_set);
  }

  /////////////////////////////////////////////////////////
  //// 2. Reconstruct all clusters in distributed manner //
  /////////////////////////////////////////////////////////
  CHECK_GT(map_reduce_config_.server_ips.size(), 0);

  const size_t cluster_num = inter_clusters_.size();

  DistributedTaskManager<SfMDataContainer> distributed_task_manager;
  distributed_task_manager.SetMapReduceConfig(map_reduce_config_);

  SfMDataContainer& sfm_data_container =
      distributed_task_manager.DataContainer();

  sfm_data_container.cluster_num = cluster_num;
  sfm_data_container.master_image_path = options_.image_path;
  sfm_data_container.server_ips =
      distributed_task_manager.GetMapReduceConfig().server_ips;
  sfm_data_container.server_ports =
      distributed_task_manager.GetMapReduceConfig().server_ports;
  if (options_.transfer_images_to_server) {
    sfm_data_container.cluster_images = cluster_images;
  }
  sfm_data_container.cluster_database_caches.swap(cluster_database_caches);
  sfm_data_container.task_type = "mapping";

  for (size_t i = 0; i < cluster_num; i++) {
    distributed_task_manager.Push(i);
  }

  distributed_task_manager.RunDistributed();

  // Stop all workers.
  for (size_t i = 0; i < map_reduce_config_.server_ips.size(); i++) {
    rpc::client c(map_reduce_config_.server_ips[i],
                  map_reduce_config_.server_ports[i]);
    c.call("Exit");
    c.call("StopServer");
  }

  //////////////////////////////////////////////////
  //// 3. Merge clusters ///////////////////////////
  //////////////////////////////////////////////////
  std::vector<Reconstruction*> reconstructions;
  for (uint i = 0; i < sfm_data_container.reconstructions.size(); i++) {
    reconstructions.push_back(sfm_data_container.reconstructions[i].release());
  }

  for (uint i = 0; i < reconstructions.size(); i++) {
    reconstructions[i]->ShowReconInfo();
  }
  // Determine the number of workers and threads per worker
  LOG(INFO) << "Sub-reconstructions size: " << reconstructions.size();
#ifdef __DEBUG__
  LOG(INFO) << "Extracting colors for local maps.";
  for (auto reconstruction : reconstructions) {
    ExtractColors(options_.image_path, reconstruction);
  }
  ExportUntransformedLocalRecons(reconstructions);
  LOG(INFO) << "Local reconstructions are exported to " << options_.output_path;
#endif

  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);

  PrintHeading1("Merging Clusters...");
  Node anchor_node;
  MergeClusters(reconstructions, num_eff_threads, anchor_node);

  LOG(INFO) << "Saving anchor node reconstruction...";
  timer.Start();
  reconstructions[anchor_node.id]->ShowReconInfo();
  reconstruction_manager_->Add(reconstructions[anchor_node.id]);
  reconstructions[anchor_node.id] = nullptr;
  timer.Pause();
  LOG(INFO) << "Time elapsed (saving anchor node reconstruction): " << timer.ElapsedSeconds();
  for (int i = 0; i < reconstructions.size(); ++i) {
    if (i != anchor_node.id) {
      delete(reconstructions[i]);
    }
  }
  
  return true;
}

bool DistributedMapperController::SequentialFeatureExtractionAndMatching() {
  Timer timer;
  LOG(INFO) << "Extracting features";
  timer.Start();
  ExtractFeature();
  timer.Pause();
  LOG(INFO) << "Time elapsed (feature extraction): " << timer.ElapsedSeconds()
            << " seconds";

  LOG(INFO) << "Matching...";
  timer.Start();
  Match();
  timer.Pause();
  LOG(INFO) << "Time elapsed: " << timer.ElapsedSeconds() << " seconds";

  return true;
}

bool DistributedMapperController::DistributedFeatureExtractionAndMatching() {
  const std::vector<ImagePair>& all_image_pairs =
      similarity_graph_.ImagePairs();
  const std::vector<int>& num_inliers = similarity_graph_.Scores();

  // Clustering images
  ImageCluster image_cluster;
  image_cluster.SetImageIds(
    std::unordered_set<image_t>(similarity_graph_.ImageIds().cbegin(),
                                similarity_graph_.ImageIds().cend()));
  for (uint i = 0; i < all_image_pairs.size(); i++) {
    const ImagePair image_pair = all_image_pairs[i];
    image_cluster.AddEdge(image_pair, num_inliers[i]);
  }

  std::unique_ptr<ImageClustering> image_clustering(
      new ImageClustering(clustering_options_, image_cluster));
  image_clustering->Cut();
  image_clustering->ExpandAllEdges();
  image_clustering->OutputClusteringSummary();

  std::vector<ImageCluster> inter_clusters =
      image_clustering->GetInterClusters();

  /////////////////////////////////////////////////////////
  //// 1. Data preparation for local clusters /////////////
  /////////////////////////////////////////////////////////
  // PrintHeading1("Reconstructing Clusters...");
  const auto& image_id_to_name = similarity_graph_.ImageIdToName();
  std::unordered_map<size_t, std::vector<std::string>> cluster_images;
  std::unordered_map<size_t, std::vector<ImageNamePair>> cluster_matching_pairs;

  const size_t cluster_num = inter_clusters.size();
  cluster_images.reserve(cluster_num);
  cluster_matching_pairs.reserve(cluster_num);

  for (size_t k = 0; k < cluster_num; k++) {
    cluster_images[k].reserve(inter_clusters[k].ImageIdsSize());
    for (const auto image_id : inter_clusters[k].ImageIds()) {
      cluster_images[k].push_back(image_id_to_name.at(image_id));
    }
    LOG(INFO) << "cluster #" << k << " has " << cluster_images[k].size()
              << " images.";

    std::vector<ImagePair> image_pairs;
    for (const auto& image_pair : inter_clusters[k].Edges()) {
      image_pairs.emplace_back(image_pair.first);
    }

    cluster_matching_pairs[k].reserve(image_pairs.size());
    for (auto image_pair : image_pairs) {
      cluster_matching_pairs[k].emplace_back(
          image_id_to_name.at(image_pair.first),
          image_id_to_name.at(image_pair.second));
    }
  }

  /////////////////////////////////////////////////////////
  //// 2. Matching all clusters in distributed manner //
  /////////////////////////////////////////////////////////
  PrintHeading1("Reconstructing Clusters...");
  CHECK_GT(map_reduce_config_.server_ips.size(), 0);

  DistributedTaskManager<MatchesDataContainer> distributed_task_manager;
  distributed_task_manager.SetMapReduceConfig(map_reduce_config_);

  MatchesDataContainer& matches_data_container =
      distributed_task_manager.DataContainer();

  matches_data_container.cluster_num = inter_clusters.size();
  matches_data_container.master_image_path = options_.image_path;
  matches_data_container.server_ips =
      distributed_task_manager.GetMapReduceConfig().server_ips;
  matches_data_container.server_ports =
      distributed_task_manager.GetMapReduceConfig().server_ports;
  matches_data_container.image_name_to_id = similarity_graph_.ImageNameToId();
  matches_data_container.cluster_images = cluster_images;
  matches_data_container.cluster_matching_pairs = cluster_matching_pairs;
  matches_data_container.task_type = "matching";

  for (size_t i = 0; i < cluster_num; i++) {
    distributed_task_manager.Push(i);
  }

  distributed_task_manager.RunDistributed();
  // merge databases;
  matches_data_container.MergeData();
  matches_data_container.database_infos[0].ExportToDatabase(database_);

  return true;
}

void DistributedMapperController::ExtractFeature() {
  ImageReaderOptions reader_options;
  reader_options.database_path = options_.database_path;
  reader_options.image_path = options_.image_path;

  // if (!image_list_.empty()) {
  //   reader_options.image_list = image_list_;
  // }

  if (!ExistsCameraModelWithName(reader_options.camera_model)) {
    std::cerr << "ERROR: Camera model does not exist" << std::endl;
  }

  if (!VerifyCameraParams(reader_options.camera_model,
                          reader_options.camera_params)) {
    return;
  }
  LOG(INFO) << "Image path: " << reader_options.image_path;
  LOG(INFO) << "Database path: " << reader_options.database_path;
  // SiftExtractionOptions sift_extraction;
  SiftFeatureExtractor feature_extractor(reader_options, extraction_options_);

  feature_extractor.Start();
  feature_extractor.Wait();
}

void DistributedMapperController::Match() {
  const auto& images = database_.ReadAllImages();

  SiftMatchingOptions options;
  // Database database(database_path_);
  FeatureMatcherCache cache(5 * similarity_graph_.ImageIds().size(),
                            &database_);

  const std::vector<ImagePair>& image_id_pairs = similarity_graph_.ImagePairs();

  SiftFeatureMatcher sift_feature_matcher(options, &database_, &cache);
  sift_feature_matcher.Setup();
  cache.Setup();
  sift_feature_matcher.Match(image_id_pairs);
}

BundleAdjustmentOptions DistributedMapperController::GlobalBundleAdjustment()
    const {
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

bool DistributedMapperController::IsPartialReconsExist(
    std::vector<Reconstruction*>& recons) const {
  const std::vector<std::string> dirs =
      colmap::GetRecursiveDirList(options_.output_path);
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
      LOG(WARNING) << "cameras, images, points3D files do not exist at "
                   << path;
      continue;
    }
    recons.push_back(recon);
  }

  if (recons.empty()) return false;
  return true;
}

void DistributedMapperController::LoadImages() {
  // Loading database.
  Timer timer;

  // Reading all images.
  LOG(INFO) << "Reading images...";
  timer.Start();
  const std::vector<Image> vec_images = database_.ReadAllImages();

  for (const auto& image : vec_images) {
    similarity_graph_.AddImage(image.ImageId(), image.Name());
  }
}

bool DistributedMapperController::LoadTwoviewGeometries() {
  // Loading database.
  Timer timer;

  // Reading all images.
  LOG(INFO) << "Reading images...";
  timer.Start();
  const std::vector<Image> vec_images = database_.ReadAllImages();

  for (const auto& image : vec_images) {
    view_graph_.AddImage(image.ImageId(), image.Name());
  }

  // Reading scene graph
  LOG(INFO) << "Reading two view geometries...";
  std::vector<TwoViewGeometry> two_view_geometries;
  std::vector<image_pair_t> image_pair_ids;
  std::vector<std::pair<image_t, image_t>> image_pairs;
  std::vector<int> num_inliers;
  database_.ReadTwoViewGeometryNumInliers(&image_pairs, &num_inliers);
  database_.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

  CHECK_EQ(image_pairs.size(), image_pair_ids.size());

  LOG(INFO) << "Estimating two view geometries";
  // #pragma omp parallel for
  for (uint i = 0; i < image_pairs.size(); i++) {
    const image_t image_id1 = image_pairs[i].first,
                  image_id2 = image_pairs[i].second;
    const ImagePair view_id_pair = ImagePair(image_id1, image_id2);
    TwoViewGeometry two_view_geometry = two_view_geometries[i];

    TwoViewInfo two_view_info;
    ceres::QuaternionToAngleAxis(two_view_geometry.qvec.data(),
                                 two_view_info.rotation_2.data());
    two_view_info.position_2 = two_view_geometry.tvec;
    two_view_info.visibility_score = two_view_geometry.inlier_matches.size();
    view_graph_.AddTwoViewGeometry(view_id_pair, two_view_info);
  }

  timer.Pause();
  LOG(INFO) << "Elapsed Time[database reading]: " << timer.ElapsedSeconds()
            << " seconds";
  LOG(INFO) << "image pairs: " << view_graph_.TwoViewGeometries().size();

  return view_graph_.TwoViewGeometries().size() > 0;
}

void DistributedMapperController::ClusteringScenes() {
  // Clustering images
  ImageCluster image_cluster;
  image_cluster.SetImageIds(std::unordered_set<image_t>(
                            view_graph_.ImageIds().cbegin(), view_graph_.ImageIds().cend()));

  const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();
  for (const auto& view_pair_it : view_pairs) {
    const ImagePair view_pair = view_pair_it.first;
    image_cluster.AddEdge(view_pair, view_pair_it.second.visibility_score);
  }

  image_clustering_ = std::unique_ptr<ImageClustering>(
      new ImageClustering(clustering_options_, image_cluster));
  image_clustering_->Cut();
  image_clustering_->Expand(clustering_options_.num_threads, this);
  image_clustering_->OutputClusteringSummary();

  inter_clusters_ = image_clustering_->GetInterClusters();
  intra_clusters_ = image_clustering_->GetIntraClusters();
  for (const auto& cluster : inter_clusters_) {
    cluster.ShowInfo();
  }
}

void DistributedMapperController::ReconstructPartitions(
    std::unordered_map<size_t, ReconstructionManager>& reconstruction_managers,
    std::vector<Reconstruction*>& reconstructions) {
  // Determine the number of workers and threads per worker
  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  const std::unordered_map<image_t, std::string>& image_id_to_name =
      view_graph_.ImageIdToName();
  const int kDefaultNumWorkers = 8;
  const int num_eff_workers =
      std::min(static_cast<int>(inter_clusters_.size()),
               std::min(kDefaultNumWorkers, num_eff_threads));
  const int num_threads_per_worker =
      std::max(1, num_eff_threads / num_eff_workers);

  // Start reconstructing the bigger clusters first for resource usage.
  const auto cmp = [](const ImageCluster& cluster1,
                      const ImageCluster& cluster2) {
    return cluster1.ImageIdsSize() > cluster2.ImageIdsSize();
  };
  std::sort(inter_clusters_.begin(), inter_clusters_.end(), cmp);

  // Start the reconstruction workers.
  reconstruction_managers.reserve(inter_clusters_.size());

  std::unordered_map<size_t, DatabaseCache> cluster_database_caches;
  cluster_database_caches.reserve(inter_clusters_.size());

  IncrementalMapperOptions local_mapper_options;
  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    std::vector<std::string> image_name_list;
    image_name_list.reserve(inter_clusters_[k].ImageIdsSize());

    for (const auto image_id : inter_clusters_[k].ImageIds()) {
      image_name_list.push_back(image_id_to_name.at(image_id));
    }
    std::sort(image_name_list.begin(), image_name_list.end());

    std::unordered_set<std::string> image_name_set(image_name_list.begin(),
                                                   image_name_list.end());
    cluster_database_caches[k].Load(
        database_, static_cast<size_t>(local_mapper_options.min_num_matches),
        local_mapper_options.ignore_watermarks, image_name_set);
  }

#pragma omp parallel for
  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    const auto& cluster = inter_clusters_[k];

    IncrementalMapperOptions custom_options = mapper_options_;
    custom_options.max_model_overlap = 20;
    custom_options.init_num_trials = options_.init_num_trials;
    custom_options.num_threads = num_threads_per_worker;
    custom_options.extract_colors = true;

    for (const auto image_id : cluster.ImageIds()) {
      custom_options.image_names.insert(image_id_to_name.at(image_id));
    }

    IncrementalMapperController mapper(&custom_options,
                                       &reconstruction_managers[k]);
    mapper.SetDatabaseCache(&cluster_database_caches[k]);

    mapper.Start();
    mapper.Wait();
  }

  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    auto& recon_manager = reconstruction_managers.at(k);
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
  LOG(INFO) << "Local reconstructions are exported to " << options_.output_path;
#endif
}

void DistributedMapperController::MergeClusters(
    std::vector<Reconstruction*>& reconstructions, const int num_eff_threads,
    Node& anchor_node) {
  if (reconstructions.size() > 1) {
    SfMAligner::AlignOptions align_options;
    align_options.image_path = options_.image_path;
    SfMAligner sfm_aligner(reconstructions, align_options);

    if (sfm_aligner.Align()) {
      anchor_node = sfm_aligner.GetAnchorNode();
      separators_ = sfm_aligner.GetSeparators();
    }

    CHECK_NE(anchor_node.id, -1);
    CHECK_NOTNULL(reconstructions[anchor_node.id]);

    const size_t filtered_points_num =
        reconstructions[anchor_node.id]->FilterAllPoints3D(
            options_.filter_max_reproj_error, options_.filter_min_tri_angle);
    LOG(INFO) << filtered_points_num << " 3D points are filtered";

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
  } else {  // Only one cluster.
    anchor_node.id = 0;
  }

  // Assign cluster id for each image.
  for (size_t i = 0; i < intra_clusters_.size(); i++) {
    const ImageCluster& intra_cluster = intra_clusters_[i];
    const std::unordered_set<image_t>& image_ids = intra_cluster.ImageIds();

    for (auto image_id : image_ids) {
      if (reconstructions[anchor_node.id]->ExistsImage(image_id)) {
        Image& image = reconstructions[anchor_node.id]->Image(image_id);
        image.SetClusterId(i);
      }
    }
  }
}

void DistributedMapperController::MergeClusters(
    std::vector<Reconstruction*>& reconstructions,
    std::unordered_map<size_t, ReconstructionManager>& reconstruction_managers,
    const int num_eff_threads) {
  LOG(INFO) << "Sub-reconstructions size: " << reconstructions.size();
  Node anchor_node;
  MergeClusters(reconstructions, num_eff_threads, anchor_node);

  LOG(INFO) << "Adding the final cluster...";
  // reconstructions[anchor_node.id]->ShowReconInfo();

  // Insert a new reconstruction manager for merged cluster.
  const size_t k = reconstruction_managers.size();
  auto& reconstruction_manager = reconstruction_managers[k];
  reconstruction_manager.Add();
  reconstruction_manager.Get(reconstruction_manager.Size() - 1) =
      *reconstructions[anchor_node.id];

  LOG(INFO) << "Erasing clusters...";
  for (size_t i = 0; i < k; i++) {
    reconstruction_managers.erase(i);
  }
  CHECK_EQ(reconstruction_managers.size(), 1);
  *reconstruction_manager_ = std::move(reconstruction_managers.begin()->second);
}

bool DistributedMapperController::Triangulate() {
  Reconstruction& global_recon = reconstruction_manager_->Get(0);

  IncrementalTriangulator triangulator(global_recon.GetCorrespondenceGraph(),
                                       &global_recon);
  IncrementalTriangulator::Options triangulate_options;
  for (const image_t image_id : separators_) {
    triangulator.TriangulateImage(triangulate_options, image_id);
  }

  return true;
}

bool DistributedMapperController::AdjustGlobalBundle() {
  Reconstruction& reconstruction =
      reconstruction_manager_->Get(reconstruction_manager_->Size() - 1);
  std::vector<point3D_t> separator_points3D;
  for (auto image_id : separators_) {
    if (!reconstruction.ExistsImage(image_id)) {
      continue;
    }
    const std::vector<point3D_t> observed_points3D =
        reconstruction.Image(image_id).ObservedPoints3D();
    separator_points3D.insert(separator_points3D.begin(),
                              observed_points3D.begin(),
                              observed_points3D.end());
  }
  std::sort(separator_points3D.begin(), separator_points3D.end());
  separator_points3D.erase(
      separator_points3D.begin(),
      std::unique(separator_points3D.begin(), separator_points3D.end()));
  LOG(INFO) << "Total separators: " << separators_.size();
  LOG(INFO) << "Total points3D observed by separators: "
            << separator_points3D.size();
  const double separator_rmse_without_ba =
      reconstruction.ComputeMeanReprojectionError(separator_points3D);
  const double all_rmse_without_ba =
      reconstruction.ComputeMeanReprojectionError({});

  BundleAdjustmentOptions ba_options = this->GlobalBundleAdjustment();
  // ba_options.solver_options.num_threads = num_eff_threads;

  Reconstruction& global_recon = reconstruction_manager_->Get(0);

  const std::vector<image_t>& reg_image_ids = global_recon.RegImageIds();

  CHECK_GE(reg_image_ids.size(), 2)
      << "At least two images must be "
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
    SelectGoodTracksForBundleAdjustment(
        global_recon, options_.long_track_length_threshold,
        options_.image_grid_cell_size_pixels,
        options_.min_num_optimized_tracks_per_view, &tracks_to_optimize);

    LOG(INFO) << tracks_to_optimize.size() << " / "
              << global_recon.NumPoints3D() << "(" << std::setprecision(2)
              << (float)tracks_to_optimize.size() /
                     (float)global_recon.NumPoints3D()
              << "%) tracks are selected for bundle adjustment.";

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

  LOG(INFO) << "RMSE for separators(before BA): " << separator_rmse_without_ba;
  LOG(INFO) << "RMSE for all images(before BA): " << all_rmse_without_ba;
  LOG(INFO) << "RMSE for separators(after BA): "
            << reconstruction.ComputeMeanReprojectionError(separator_points3D);
  LOG(INFO) << "RMSE for all images(after BA): "
            << reconstruction.ComputeMeanReprojectionError({});
  LOG(INFO) << "Selected " << tracks_to_optimize.size() << " to optimize.";
  LOG(INFO) << "Total tracks: " << points3d.size();

  // Normalize scene for numerical stability and
  // to avoid large scale changes in viewer.
  global_recon.Normalize();

  return true;
}

void DistributedMapperController::ExportUntransformedLocalRecons(
    const std::vector<Reconstruction*>& reconstructions) const {
  for (size_t i = 0; i < reconstructions.size(); ++i) {
    const std::string reconstruction_path =
        JoinPaths(options_.output_path, "partition" + std::to_string(i));
    CreateDirIfNotExists(reconstruction_path);
    reconstructions[i]->Write(reconstruction_path);
  }
}

bool DistributedMapperController::GlobalRotationAveraging() {
  // Initialize the orientation estimations by walking along the maximum
  // spanning tree.
  // TODO: (chenyu)
  const auto& image_ids = view_graph_.ImageIds();
  for (auto image_id : image_ids) {
    rotations_[image_id] = Eigen::Vector3d::Zero();
  }
  // Clear pairs that are not included in largest connected components.
  std::unordered_set<ImagePair> view_pairs_to_remove;
  std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();
  for (auto view_pair_it : view_pairs) {
    if (rotations_.count(view_pair_it.first.first) == 0 ||
        rotations_.count(view_pair_it.first.second) == 0) {
      view_pairs_to_remove.insert(view_pair_it.first);
    }
  }
  for (const ImagePair view_pair : view_pairs_to_remove) {
    view_pairs.erase(view_pair);
    database_.DeleteMatches(view_pair.first, view_pair.second);
    database_.DeleteInlierMatches(view_pair.first, view_pair.second);
  }

  // Choose the global rotation estimation type.
  std::unique_ptr<RotationEstimator> rotation_estimator;
  switch (options_.global_rotation_estimator_type) {
    case GlobalRotationEstimatorType::ROBUST_L1L2: {
      RobustRotationEstimator::Options robust_rotation_estimator_options;
      rotation_estimator.reset(
          new RobustRotationEstimator(robust_rotation_estimator_options));
      break;
    }
    case GlobalRotationEstimatorType::NONLINEAR: {
      rotation_estimator.reset(new NonlinearRotationEstimator());
      break;
    }
    default: {
      LOG(FATAL) << "Invalid type of global rotation estimation chosen.";
      break;
    }
  }

  bool success = rotation_estimator->EstimateRotations(view_pairs, &rotations_);

  if (!success) {
    return false;
  }

  // Filter view pairs based on the relative rotation and the
  // estimated global orientations.
  FilterViewPairsFromOrientation(
      rotations_, options_.max_relative_rotation_difference_degrees,
      view_graph_.TwoViewGeometries(), database_);
  UpdateViewGraph();
  // Remove any disconnected views from the estimation.
  PrintHeading1("Extracting Largest Connected Component...");
  LOG(INFO) << "Total image number: " << view_graph_.ImageIds().size();
  view_graph_.TwoviewGeometriesToImagePairs();
  view_graph_.ExtractLargestCC();
  LOG(INFO) << "image number in largest cc: " << view_graph_.ImageIds().size();

  return true;
}

void DistributedMapperController::UpdateViewGraph() {
  const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();
  for (auto view_pair_it : view_pairs) {
    const ImagePair view_pair = view_pair_it.first;
    TwoViewInfo& two_view_info = view_pair_it.second;

    const Eigen::Vector3d R1 = rotations_[view_pair.first];
    const Eigen::Vector3d R2 = rotations_[view_pair.second];
    Eigen::Vector3d R12 = RelativeRotationFromTwoRotations(R1, R2);
    two_view_info.rotation_2 = R12;
  }
}

}  // namespace DAGSfM