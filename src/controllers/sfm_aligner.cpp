#include "controllers/sfm_aligner.h"

#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <glog/logging.h>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "base/similarity_transform.h"
#include "base/track_selection.h"
#include "estimators/ransac_similarity.h"
#include "estimators/sim3.h"
#include "estimators/similarity_transform.h"
#include "math/util.h"
#include "optim/bundle_adjustment.h"
#include "sfm/incremental_triangulator.h"
#include "solver/l1_solver.h"
#include "util/misc.h"
#include "util/reconstruction_io.h"
#include "util/timer.h"

namespace DAGSfM {
namespace {

double CheckReprojError(const vector<Eigen::Vector3d>& src_observations,
                        const vector<Eigen::Vector3d>& dst_observations,
                        const double& scale, const Eigen::Matrix3d& R,
                        const Eigen::Vector3d& t) {
  double reproj_err = 0.0;
  const int size = src_observations.size();
  for (int i = 0; i < size; i++) {
    Eigen::Vector3d reproj_obv = scale * R * src_observations[i] + t;
    reproj_err += (reproj_obv - dst_observations[i]).norm();
  }

  LOG(INFO) << "Mean Reprojection Error: " << reproj_err / size << " ("
            << reproj_err << "/" << size << ")";
  return reproj_err / size;
}

void FindSimilarityTransform(const std::vector<Eigen::Vector3d>& observations1,
                             const std::vector<Eigen::Vector3d>& observations2,
                             const double threshold, const double p,
                             Eigen::Matrix3d& R, Eigen::Vector3d& t,
                             double& scale, double& msd) {
  std::vector<Eigen::Vector3d> inliers1, inliers2;

  if (observations1.size() > 5) {
    LOG(INFO) << "Finding Similarity by RANSAC";
    RansacSimilarity(observations1, observations2, inliers1, inliers2, R, t,
                     scale, threshold, p);
    VLOG(2) << "inliers size: " << inliers1.size();
    // Re-compute similarity by inliers
    Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, inliers1.size()),
                    x2 = Eigen::MatrixXd::Zero(3, inliers2.size());
    for (uint i = 0; i < inliers1.size(); i++) {
      x1.col(i) = inliers1[i];
      x2.col(i) = inliers2[i];
    }
    DAGSfM::FindRTS(x1, x2, &scale, &t, &R);
    // Optional non-linear refinement of the found parameters
    DAGSfM::Refine_RTS(x1, x2, &scale, &t, &R);

    if (inliers1.size() < 4) {
      msd = numeric_limits<double>::max();
      return;
    }
    // else msd = CheckReprojError(inliers1, inliers2, scale, R, t);
  }

  if (observations1.size() <= 5 || inliers1.size() <= 5) {
    Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, observations1.size()),
                    x2 = Eigen::MatrixXd::Zero(3, observations2.size());
    for (uint i = 0; i < observations1.size(); i++) {
      x1.col(i) = observations1[i];
      x2.col(i) = observations2[i];
    }
    DAGSfM::FindRTS(x1, x2, &scale, &t, &R);
    DAGSfM::Refine_RTS(x1, x2, &scale, &t, &R);

    // msd = CheckReprojError(observations1, observations2, scale, R, t);
  }

  msd = CheckReprojError(observations1, observations2, scale, R, t);
}

void FindCommon3DPoints(const std::vector<image_t>& common_reg_images,
                        const Reconstruction& recon1,
                        const Reconstruction& recon2,
                        std::vector<Eigen::Vector3d>& src_points,
                        std::vector<Eigen::Vector3d>& ref_points) {
  // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> common_3D_points;
  std::unordered_set<image_t> common_image_ids(common_reg_images.begin(),
                                               common_reg_images.end());
  LOG(INFO) << "Begin find common 3D points";
  for (const auto& point3D : recon2.Points3D()) {
    const Eigen::Vector3d point3D2 = point3D.second.XYZ();

    for (const auto& track_el : point3D.second.Track().Elements()) {
      if (common_image_ids.count(track_el.image_id) > 0) {
        const auto& point2D =
            recon1.Image(track_el.image_id).Point2D(track_el.point2D_idx);
        if (point2D.HasPoint3D()) {
          const Eigen::Vector3d point3D1 =
              recon1.Point3D(point2D.Point3DId()).XYZ();
          // common_3D_points.emplace_back(point3D1, point3D2);
          src_points.emplace_back(point3D1);
          ref_points.emplace_back(point3D2);
        }
      }
    }
  }
  LOG(INFO) << "Find " << ref_points.size() << " common 3D points.";
}

}  // namespace

SfMAligner::SfMAligner(const std::vector<Reconstruction*>& reconstructions,
                       const AlignOptions& options)
    : options_(options), reconstructions_(reconstructions) {
  // some logic or parameters check
  CHECK_GT(reconstructions.size(), 0);
  CHECK_GT(reconstructions_.size(), 0);

  for (uint i = 0; i < reconstructions_.size(); i++) {
    LOG(INFO) << "Node id: " << i;
    CHECK_NOTNULL(reconstructions_[i]);
    LOG(INFO) << "Total images number: " << reconstructions_[i]->NumImages();
  }
}

const std::vector<BitmapColor<float>> SfMAligner::ColorContainers = {
    BitmapColor<float>(255, 25.5, 0), BitmapColor<float>(0, 255, 255),
    BitmapColor<float>(255, 102, 0),  BitmapColor<float>(153, 51, 204),
    BitmapColor<float>(0, 255, 51),   BitmapColor<float>(255, 0, 204),
    BitmapColor<float>(255, 255, 0),  BitmapColor<float>(255, 153, 255),
    BitmapColor<float>(255, 51, 0),   BitmapColor<float>(0, 204, 255),
    BitmapColor<float>(255, 204, 255)};

bool SfMAligner::Align() {
  Timer timer;

  // 1. Constructing a graph from reconstructions,
  // each node is a reconstruction, edges represent the connections between
  // reconstructions (by the means of common images or common 3D points), the
  // weight of edge represents the mean reprojection error.
  LOG(INFO) << "Constructing Reconstructions Graph...";
  timer.Start();
  ConstructReconsGraph();
  timer.Pause();
  summary_.construct_recon_graph_time = timer.ElapsedSeconds();
  recons_graph_.ShowInfo();
  CHECK_EQ(recons_graph_.GetNodesNum(), reconstructions_.size());

  // The reconstruction graph should be at least a spanning tree,
  // or we couldn't stitch all reconstructions together due to
  // too large alignment error or disconnected components.
  if (recons_graph_.GetEdgesNum() < recons_graph_.GetNodesNum() - 1) {
    LOG(WARNING) << "Can't align all reconstructions together due to "
                 << "too large alignment error or disconnected components."
                 << "We would just merge local maps in the largest connected "
                    "components.";
  }
  const Graph<Node, Edge> largest_cc = recons_graph_.ExtractLargestCC();
  std::vector<size_t> vec_largest_cc_nodes;
  vec_largest_cc_nodes.reserve(largest_cc.GetNodes().size());
  for (auto node_it : largest_cc.GetNodes()) {
    vec_largest_cc_nodes.push_back(node_it.first);
  }

  // 2. Constructing a minimum spanning tree, thus we can select the
  // most accurate n - 1 edges for accurate alignment.
  LOG(INFO) << "Finding Minimum Spanning Tree...";
  timer.Start();
  std::vector<Edge> mst_edges = largest_cc.Kruskal();

  Graph<Node, Edge> mst;
  for (const auto edge : mst_edges) {
    mst.AddEdge(edge);
  }

  if (mst_edges.size() < largest_cc.GetNodesNum() - 1) {
    LOG(WARNING) << "Invalid MST";
    mst.ShowInfo();
    return false;
  }
  mst.ShowInfo();
  timer.Pause();
  summary_.construct_mst_time = timer.ElapsedSeconds();

  // 3. Finding an anchor node, an anchor node is a reference reconstruction
  // that all other reconstructions should be aligned to.
  LOG(INFO) << "Finding Anchor Node...";
  timer.Start();
  FindAnchorNode(&mst);
  timer.Pause();
  summary_.find_anchor_node_time = timer.ElapsedSeconds();

  // 4. Compute the final transformation to anchor node for each cluster
  LOG(INFO) << "Computing Final Similarity Transformations...";
  timer.Start();
  for (auto i : vec_largest_cc_nodes) {
    if (static_cast<int>(i) != anchor_node_.id) {
      this->ComputePath(i, anchor_node_.id);
    }
  }
  sim3_to_anchor_[anchor_node_.id] = Sim3();
  timer.Pause();
  summary_.compute_final_transformation_time = timer.ElapsedSeconds();

  // 5. Merging all other reconstructions to anchor node
  LOG(INFO) << "Merging Reconstructions...";
  timer.Start();
  this->MergeReconstructions(vec_largest_cc_nodes);
  timer.Pause();
  summary_.merging_time = timer.ElapsedSeconds();

  return true;
}

Node SfMAligner::GetAnchorNode() const { return anchor_node_; }

std::vector<Sim3> SfMAligner::GetSim3ToAnchor() const {
  return sim3_to_anchor_;
}

const std::unordered_set<image_t>& SfMAligner::GetSeparators() const {
  return separators_;
}

void SfMAligner::ConstructReconsGraph() {
  // 1. Add nodes
  for (size_t i = 0; i < reconstructions_.size(); i++) {
    Node node(i);
    // node.recon = reconstructions_[i];
    recons_graph_.AddNode(node);
  }

  // 2. Add edges
  for (uint i = 0; i < reconstructions_.size(); i++) {
    for (uint j = i + 1; j < reconstructions_.size(); j++) {
      const double weight = ComputeEdgeWeight(i, j);
      LOG(INFO) << "weight: " << weight;
      if (weight != std::numeric_limits<double>::max()) {
        recons_graph_.AddEdge(Edge(i, j, (float)weight));
      }
    }
  }
}

double SfMAligner::ComputeEdgeWeight(const uint i, const uint j) {
  const Reconstruction& recon1 = *reconstructions_[i];
  const Reconstruction& recon2 = *reconstructions_[j];
  double weight = std::numeric_limits<double>::max();

  Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity(3, 3),
                  R2 = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Vector3d t1 = Eigen::Vector3d::Zero(), t2 = Eigen::Vector3d::Zero();
  double s1 = 1.0, s2 = 1.0;

  // Find common registered images
  std::vector<image_t> common_reg_images = recon1.FindCommonRegImageIds(recon2);
  for (auto image_id : common_reg_images) {
    separators_.insert(image_id);
  }
  std::vector<Eigen::Vector3d> src_points, ref_points;
  // for (const auto common_id : common_reg_images) {
  //     src_points.push_back(recon1.Image(common_id).ProjectionCenter());
  //     ref_points.push_back(recon2.Image(common_id).ProjectionCenter());
  // }
  FindCommon3DPoints(common_reg_images, recon1, recon2, src_points, ref_points);
  LOG(INFO) << "Common registerd images number: " << common_reg_images.size();

  if (common_reg_images.size() < 2) {
    LOG(WARNING) << "Not found enough common registered images.";
    return std::numeric_limits<double>::max();
  } else {
    double msd1 = 0.0, msd2 = 0.0;
    FindSimilarityTransform(src_points, ref_points, options_.threshold,
                            options_.confidence, R1, t1, s1, msd1);
    FindSimilarityTransform(ref_points, src_points, options_.threshold,
                            options_.confidence, R2, t2, s2, msd2);

    weight = std::max(msd1, msd2);

    if (weight != numeric_limits<double>::max()) {
      sim3_graph_[i][j] = Sim3(R1, t1, s1);
      sim3_graph_[j][i] = Sim3(R2, t2, s2);
    }
  }
  // else if (common_reg_images.size() < 3) {
  //     std::vector<Eigen::Matrix3d> src_rotations, ref_rotations;
  //     for (const auto common_id : common_reg_images) {
  //         src_rotations.push_back(recon1.Image(common_id).RotationMatrix());
  //         ref_rotations.push_back(recon2.Image(common_id).RotationMatrix());
  //     }
  //     ComputeSimilarityByCameraMotions(src_points, ref_points,
  //                                      src_rotations, ref_rotations, R1, t1,
  //                                      s1);
  //     ComputeSimilarityByCameraMotions(ref_points, src_points,
  //                                      ref_rotations, src_rotations, R2, t2,
  //                                      s2);

  //     double msd1 = CheckReprojError(src_points, ref_points, s1, R1, t1),
  //            msd2 = CheckReprojError(ref_points, src_points, s2, R2, t2);

  //     weight = std::max(msd1, msd2);

  //     if (weight != numeric_limits<double>::max()) {
  //         sim3_graph_[i][j] = Sim3(R1, t1, s1);
  //         sim3_graph_[j][i] = Sim3(R2, t2, s2);
  //     }
  // }

  return (weight > options_.max_reprojection_error)
             ? std::numeric_limits<double>::max()
             : weight;
}

void SfMAligner::FindAnchorNode(Graph<Node, Edge>* graph) {
  paths_.resize(recons_graph_.GetNodesNum());
  sim3_to_anchor_.resize(recons_graph_.GetNodesNum());

  // The anchor is found by merging all leaf nodes to their adjacent nodes,
  // until one node or two nodes left. If two nodes left, we choose the
  // reconstruction that has the largest size as the anchor.
  int layer = 1;
  uint anchor_index = 0;

  while (graph->GetNodesNum() > 1) {
    LOG(INFO) << "Merging the " << layer++ << "-th layer leaf nodes";

    graph->CountOutDegrees();
    graph->CountInDegrees();
    graph->CountDegrees();
    std::unordered_map<size_t, size_t> degrees = graph->GetDegrees();

    // Finding all leaf nodes. Leaf node in graph has degree equals to 1.
    std::vector<size_t> indexes;
    if (graph->GetNodesNum() == 2) {
      indexes.push_back(degrees.begin()->first);
    } else {
      for (auto it = degrees.begin(); it != degrees.end(); ++it) {
        LOG(INFO) << "node: " << it->first << ", "
                  << "degree: " << it->second;
        if (it->second == 1) indexes.push_back(it->first);
      }
    }
    if (indexes.empty()) break;

    for (auto idx : indexes) {
      // if (idx == -1) break;
      const Edge& edge = graph->FindConnectedEdge(idx);

      LOG(INFO) << "Find node [degree = 1]: " << idx;
      LOG(INFO) << edge.src << "->" << edge.dst << ": " << edge.weight;

      // src is the node with degree = 1
      uint src = (idx == edge.src) ? edge.src : edge.dst;
      uint dst = (idx == edge.src) ? edge.dst : edge.src;

      LOG(INFO) << "Merge Clusters: " << src << "->" << dst << ": "
                << edge.weight;
      anchor_index = dst;
      const Sim3 sim = sim3_graph_[src][dst];
      paths_[src].insert(std::make_pair(dst, sim));

      graph->DeleteNode(src);
      graph->DeleteEdge(src, dst);
      graph->DeleteEdge(dst, src);
      graph->ShowInfo();
    }
  }

  anchor_node_.id = anchor_index;
}

void SfMAligner::ComputePath(int src, int dst) {
  LOG(INFO) << "Computing Path: " << src << "->" << dst;
  std::queue<int> qu;
  qu.push(src);

  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double s = 1.0;

  Sim3 sim(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Identity(), 1.0);
  LOG(INFO) << "v: " << src;
  while (!qu.empty()) {
    int u = qu.front();
    qu.pop();
    auto it = paths_[u].begin();
    int v = it->first;
    LOG(INFO) << "v: " << v;
    s = it->second.s * s;
    r = it->second.R * r.eval();
    t = it->second.s * it->second.R * t.eval() + it->second.t;
    if (v == dst) {
      sim.s = s;
      sim.R = r;
      sim.t = t;
      sim3_to_anchor_[src] = sim;
      return;
    } else
      qu.push(v);
  }
  LOG(INFO) << "\n";
}

void SfMAligner::MergeReconstructions(std::vector<size_t>& node_ids) {
  if (options_.assign_color_for_clusters) {
    for (size_t i = 0; i < reconstructions_.size(); i++) {
      const int color_id = i % SfMAligner::ColorContainers.size();
      reconstructions_[i]->AssignColorsForAllPoints(
          SfMAligner::ColorContainers[color_id]);
      // // Assign cluster id for each image.
      // const std::vector<image_t> reg_image_ids =
      // reconstructions_[i]->RegImageIds(); for (auto image_id : reg_image_ids)
      // {
      //     Image& image = reconstructions_[i]->Image(image_id);
      //     image.SetClusterId(i);
      // }
    }
  }

  for (auto id : node_ids) {
    if (static_cast<int>(id) == anchor_node_.id) {
      continue;
    }

    Sim3 sim3 = sim3_to_anchor_[id];
    Eigen::Matrix3x4d alignment;
    alignment.block(0, 0, 3, 3) = sim3.s * sim3.R;
    alignment.block(0, 3, 3, 1) = sim3.t;

    reconstructions_[anchor_node_.id]->Merge(*reconstructions_[id], alignment);
  }
}

bool ComputeSimilarityByCameraMotions(
    std::vector<Eigen::Vector3d>& camera_centers1,
    std::vector<Eigen::Vector3d>& camera_centers2,
    std::vector<Eigen::Matrix3d>& camera_rotations1,
    std::vector<Eigen::Matrix3d>& camera_rotations2,
    Eigen::Matrix3d& relative_r, Eigen::Vector3d& relative_t, double& scale) {
  // my hybrid approach by combining "Divide and Conquer: Efficient Large-Scale
  // Structure from Motion Using Graph Partitioning" and RANSAC

  const uint n = camera_centers1.size();
  std::vector<Eigen::Vector3d> ts1(n);
  std::vector<Eigen::Vector3d> ts2(n);

  for (uint i = 0; i < n; i++) {
    ts1[i] = -camera_rotations1[i] * camera_centers1[i];
    ts2[i] = -camera_rotations2[i] * camera_centers2[i];
  }

  // compute relative scale from a->b
  std::vector<double> scales;
  for (uint i = 0; i < n; i++) {
    Eigen::Vector3d center_a1 = camera_centers1[i];
    Eigen::Vector3d center_b1 = camera_centers2[i];
    for (uint j = i + 1; j < n; j++) {
      Eigen::Vector3d center_a2 = camera_centers1[j];
      Eigen::Vector3d center_b2 = camera_centers2[j];
      double scale_ab =
          (center_b1 - center_b2).norm() / (center_a1 - center_a2).norm();
      scales.push_back(scale_ab);
    }
  }
  // retrieve the median of scales, according to
  // the equation (5) of the paper "Divide and Conquer: Efficient Large-Scale
  // Structure from Motion Using Graph Partitioning"
  std::sort(scales.begin(), scales.end());
  scale = scales[scales.size() / 2];

  // compute relative rotation & relative translation from a->b
  std::vector<Correspondence3D> corres3d;
  std::vector<CorrespondenceEuc> input_datas;
  for (uint i = 0; i < camera_centers1.size(); i++) {
    corres3d.emplace_back(camera_centers1[i], camera_centers2[i]);
    input_datas.push_back(make_pair(Euclidean3D(camera_rotations1[i], ts1[i]),
                                    Euclidean3D(camera_rotations2[i], ts2[i])));
  }
  EuclideanEstimator euc_estimator(scale, corres3d);

  Euclidean3D euc3d;
  RansacParameters params;
  params.rng =
      std::make_shared<RandomNumberGenerator>((unsigned int)time(NULL));
  params.error_thresh = 0.002;
  params.max_iterations = 1000;

  Prosac<EuclideanEstimator> prosac_euc3(params, euc_estimator);
  prosac_euc3.Initialize();
  RansacSummary summary;
  prosac_euc3.Estimate(input_datas, &euc3d, &summary);

  relative_r = euc3d.R;
  relative_t = euc3d.t;

  return true;
}

}  // namespace DAGSfM