#include <limits>
#include <vector>
#include <memory>
#include <unordered_map>

#include "controllers/sfm_aligner.h"
#include "estimators/similarity_transform.h"
#include "estimators/ransac_similarity.h"
#include "estimators/sim3.h"
#include "optim/bundle_adjustment.h"
#include "util/timer.h"
#include "math/util.h"
#include "sfm/incremental_triangulator.h"
#include "solver/l1_solver.h"
#include "base/similarity_transform.h"
#include "util/reconstruction_io.h"
#include "util/misc.h"

#include <glog/logging.h>
#include <Eigen/Geometry>
#include <ceres/rotation.h>
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>



namespace GraphSfM {
namespace {
double MeanReprojectionResiduals(const std::vector<double>& residuals)
{
    double mean_residual = 0.0;
    for (auto residual : residuals) {
        mean_residual += residual;
    }
    return mean_residual / residuals.size();
}

std::vector<double> ComputeReprojectionResiduals(const std::vector<Eigen::Vector3d>& src_points,
                                    const std::vector<Eigen::Vector3d>& ref_points,
                                    const Eigen::Matrix3x4d& alignment)
{
    std::vector<double> residuals;
    for (uint i = 0; i < src_points.size(); i++) {
        const Eigen::Vector3d& tsrc_point = alignment * src_points[i].homogeneous();
        const Eigen::Vector3d& ref_point = ref_points[i];

        double residual = (tsrc_point - ref_point).norm();
        residuals.push_back(residual);
    }
    return residuals;
}

double CheckReprojError(const vector<Eigen::Vector3d>& src_observations,
                        const vector<Eigen::Vector3d>& dst_observations,
                        const double& scale,
                        const Eigen::Matrix3d& R,
                        const Eigen::Vector3d& t)
{
    double reproj_err = 0.0;
    const int size = src_observations.size();
    for (int i = 0; i < size; i++) {
        Eigen::Vector3d reproj_obv = scale * R * src_observations[i] + t;
        reproj_err += (reproj_obv - dst_observations[i]).norm();
    }

    LOG(INFO) << "Mean Reprojection Error: " << reproj_err / size
              << " (" << reproj_err << "/" 
              << size << ")";
    return reproj_err / size;
}

void FindSimilarityTransform(const std::vector<Eigen::Vector3d>& observations1,
                             const std::vector<Eigen::Vector3d>& observations2,
                             const double threshold,
                             const double p,
                             Eigen::Matrix3d& R,
                             Eigen::Vector3d& t,
                             double& scale,
                             double& msd)
{
    std::vector<Eigen::Vector3d> inliers1, inliers2;

    if (observations1.size() > 5) {
        LOG(INFO) << "Finding Similarity by RANSAC";
        RansacSimilarity(observations1, observations2, 
                         inliers1, inliers2, 
                         R, t, scale, 
                         threshold, p);
        VLOG(2) << "inliers size: " << inliers1.size();
        // Re-compute similarity by inliers
        Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, inliers1.size()),
                        x2 = Eigen::MatrixXd::Zero(3, inliers2.size());
        for(uint i = 0; i < inliers1.size(); i++) {
            x1.col(i) = inliers1[i];
            x2.col(i) = inliers2[i];
        }
        GraphSfM::FindRTS(x1, x2, &scale, &t, &R);
        // Optional non-linear refinement of the found parameters
        GraphSfM::Refine_RTS(x1, x2, &scale, &t, &R);

        if (inliers1.size() < 4) { msd = numeric_limits<double>::max(); return; }
        // else msd = CheckReprojError(inliers1, inliers2, scale, R, t);
    }

    if (observations1.size() <= 5 || inliers1.size() <= 5) {
        Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, observations1.size()),
                        x2 = Eigen::MatrixXd::Zero(3, observations2.size());
        for (uint i = 0; i < observations1.size(); i++) {
            x1.col(i) = observations1[i];
            x2.col(i) = observations2[i];
        }
        GraphSfM::FindRTS(x1, x2, &scale, &t, &R);
        GraphSfM::Refine_RTS(x1, x2, &scale, &t, &R);
        
        // msd = CheckReprojError(observations1, observations2, scale, R, t);
    }

    msd = CheckReprojError(observations1, observations2, scale, R, t);
}

double CheckAngularResidual(const std::vector<Eigen::Matrix3d>& src_rotations,
                           const std::vector<Eigen::Matrix3d>& dst_rotations,
                           const Eigen::Matrix3d& R)
{
    double angular_residual = 0.0;
    const uint n = src_rotations.size();
    for (uint i = 0; i < n; i++) {
        Eigen::Matrix3d rel_rotation = 
            dst_rotations[i].transpose() * src_rotations[i] * R.transpose();
        Eigen::Vector3d angle_axis;
        ceres::RotationMatrixToAngleAxis(rel_rotation.data(), angle_axis.data());
        angular_residual += RadToDeg(angle_axis.norm());
    }

    angular_residual /= n;
    LOG(INFO) << "Average Angular Residual: " << angular_residual << " degree.";
    return angular_residual;
}

bool ComputeSimilarityByCameraMotions(
    std::vector<Eigen::Vector3d>& camera_centers1,
    std::vector<Eigen::Vector3d>& camera_centers2,
    std::vector<Eigen::Matrix3d>& camera_rotations1,
    std::vector<Eigen::Matrix3d>& camera_rotations2,
    Eigen::Matrix3d& relative_r,
    Eigen::Vector3d& relative_t,
    double& scale)
{
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
            double scale_ab = (center_b1 - center_b2).norm() / 
                             (center_a1 - center_a2).norm();
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
    for (uint i = 0; i < camera_centers1.size() ;i++) {
        corres3d.emplace_back(camera_centers1[i], camera_centers2[i]);
        input_datas.push_back(make_pair(Euclidean3D(camera_rotations1[i], ts1[i]),
                                        Euclidean3D(camera_rotations2[i], ts2[i])));
    }
    EuclideanEstimator euc_estimator(scale, corres3d);
    
    Euclidean3D euc3d;
    RansacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>((unsigned int)time(NULL));
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
} // namespace

SfMAligner::SfMAligner(const std::vector<Reconstruction*>& reconstructions,
                       const BundleAdjustmentOptions& ba_options)
     : reconstructions_(reconstructions),
       ba_options_(ba_options)
{
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
    BitmapColor<float>(255, 102, 0), BitmapColor<float>(153, 51, 204),
    BitmapColor<float>(0, 255, 51), BitmapColor<float>(255, 0, 204),
    BitmapColor<float>(255, 255, 0), BitmapColor<float>(255, 153, 255), 
    BitmapColor<float>(255, 51, 0), BitmapColor<float>(0, 204, 255),
    BitmapColor<float>(255, 204, 255)
};

bool SfMAligner::Align()
{
    // 1. Constructing a graph from reconstructions,
    // each node is a reconstruction, edges represent the connections between 
    // reconstructions (by the means of common images or common 3D points), the weight
    // of edge represents the mean reprojection error.
    LOG(INFO) << "Constructing Reconstructions Graph...";
    ConstructReconsGraph();
    recons_graph_.ShowInfo();
    CHECK_EQ(recons_graph_.GetNodesNum(), reconstructions_.size());

    // The reconstruction graph should be at least a spanning tree,
    // or we couldn't stitch all reconstructions together due to 
    // too large alignment error or disconnected components.
    if (recons_graph_.GetEdgesNum() < recons_graph_.GetNodesNum() - 1) {
        LOG(ERROR) << "Can't align all reconstructions together due to "
                   << "too large alignment error or disconnected components";
        return false;
    }

    // 2. Constructing a minimum spanning tree, thus we can select the
    // most accurate n - 1 edges for accurate alignment.
    LOG(INFO) << "Finding Minimum Spanning Tree...";
    std::vector<Edge> mst_edges = recons_graph_.Kruskal();

    Graph<Node, Edge> mst;
    for (auto node : recons_graph_.GetNodes()) {
        mst.AddNode(node.second);
    }
    for (const auto edge : mst_edges) {
        mst.AddEdge(edge);
    }

    if (mst_edges.size() < recons_graph_.GetNodesNum() - 1) {
        LOG(WARNING) << "Invalid MST";
        mst.ShowInfo();
        return false;
    }
    mst.ShowInfo();

    // 3. Finding an anchor node, an anchor node is a reference reconstruction 
    // that all other reconstructions should be aligned to.
    LOG(INFO) << "Finding Anchor Node...";
    FindAnchorNode(&mst);

    // 4. Compute the final transformation to anchor node for each cluster
    LOG(INFO) << "Computing Final Similarity Transformations...";
    for (uint i = 0; i < reconstructions_.size(); i++) {
        if (static_cast<int>(i) != anchor_node_.id) {
            this->ComputePath(i, anchor_node_.id);
        }
    }
    sim3_to_anchor_[anchor_node_.id] = Sim3();

    // 5. Merging all other reconstructions to anchor node
    LOG(INFO) << "Merging Reconstructions...";
    this->MergeReconstructions();

    // 6. Re-triangulation
    if (options_.retriangulate) {
        LOG(INFO) << "Re-triangulating...";

        std::unique_ptr<IncrementalTriangulator> triangulator;
        triangulator.reset(new IncrementalTriangulator(
                reconstructions_[anchor_node_.id]->GetCorrespondenceGraph(), 
                reconstructions_[anchor_node_.id]));
        IncrementalTriangulator::Options triangulator_options;
        triangulator->Retriangulate(triangulator_options);
    }

    // 7. Final Bundle Adjustment
    if (options_.final_ba) {
        LOG(INFO) << "Final Global Bundle Adjustment";
        this->AdjustGlobalBundle();
    }

    return true;
}

Node SfMAligner::GetAnchorNode() const
{
    return anchor_node_;
}

std::vector<Sim3> SfMAligner::GetSim3ToAnchor() const
{
    return sim3_to_anchor_;
}

void SfMAligner::ConstructReconsGraph()
{
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

double SfMAligner::ComputeEdgeWeight(const uint i, const uint j)
{
    const Reconstruction& recon1 = *reconstructions_[i];
    const Reconstruction& recon2 = *reconstructions_[j];
    double weight = std::numeric_limits<double>::max();

    Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity(3, 3), 
                    R2 = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Vector3d t1 = Eigen::Vector3d::Zero(),
                    t2 = Eigen::Vector3d::Zero();
    double s1 = 1.0, s2 = 1.0;

    // Find common registered images
    std::vector<image_t> common_reg_images = recon1.FindCommonRegImageIds(recon2);
    std::vector<Eigen::Vector3d> src_points, ref_points;
    std::vector<Eigen::Matrix3d> src_rotations, ref_rotations;
    for (const auto common_id : common_reg_images) {
        src_points.push_back(recon1.Image(common_id).ProjectionCenter());
        src_rotations.push_back(recon1.Image(common_id).RotationMatrix());
        ref_points.push_back(recon2.Image(common_id).ProjectionCenter());
        ref_rotations.push_back(recon2.Image(common_id).RotationMatrix());
    }
    LOG(INFO) << "Common registerd images number: " << common_reg_images.size();

    if (common_reg_images.size() < 2) {
        LOG(WARNING) << "Not found enough common registered images.";
        return std::numeric_limits<double>::max();
    } else if (common_reg_images.size() > 1) {
        ComputeSimilarityByCameraMotions(src_points, ref_points, 
                                         src_rotations, ref_rotations, R1, t1, s1);
        ComputeSimilarityByCameraMotions(ref_points, src_points, 
                                         ref_rotations, src_rotations, R2, t2, s2);
            
        double msd1 = CheckReprojError(src_points, ref_points, s1, R1, t1),
               msd2 = CheckReprojError(ref_points, src_points, s2, R2, t2);

        // // angular residual should be considered
        // const double angular_residual1 = 
        //     CheckAngularResidual(src_rotations, ref_rotations, R1);
        // const double angular_residual2 = 
        //     CheckAngularResidual(ref_rotations, src_rotations, R2);

        weight = std::max(msd1, msd2);
        // weight = std::max(std::max(msd1, msd2), 
        //                   std::max(angular_residual1, angular_residual2));

        if (weight != numeric_limits<double>::max()) {
            sim3_graph_[i][j] = Sim3(R1, t1, s1);
            sim3_graph_[j][i] = Sim3(R2, t2, s2);
        }
    } else {
        double msd1 = 0.0, msd2 = 0.0;
        FindSimilarityTransform(src_points, ref_points, 
                                options_.threshold, options_.confidence, 
                                R1, t1, s1, msd1);
        FindSimilarityTransform(ref_points, src_points,
                                options_.threshold, options_.confidence, 
                                R2, t2, s2, msd2);

        // // angular residual should be considered
        // const double angular_residual1 = 
        //     CheckAngularResidual(src_rotations, ref_rotations, R1);
        // const double angular_residual2 = 
        //     CheckAngularResidual(ref_rotations, src_rotations, R2);

        weight = std::max(msd1, msd2);
        // weight = std::max(std::max(msd1, msd2), 
                        //   std::max(angular_residual1, angular_residual2));

        if (weight != numeric_limits<double>::max()) {
            sim3_graph_[i][j] = Sim3(R1, t1, s1);
            sim3_graph_[j][i] = Sim3(R2, t2, s2);
        }
    }

    return (weight > options_.max_reprojection_error) ? 
                     std::numeric_limits<double>::max() :
                     weight;
}

void SfMAligner::FindAnchorNode(Graph<Node, Edge>* graph)
{
    paths_.resize(recons_graph_.GetNodesNum());
    sim3_to_anchor_.resize(recons_graph_.GetNodesNum());

    // The anchor is found by merging all leaf nodes to their adjacent nodes, 
    // until one node or two nodes left. If two nodes left, we choose the reconstruction 
    // that has the largest size as the anchor.
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

            LOG(INFO) << "Merge Clusters: " << src << "->" << dst << ": " << edge.weight;
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

void SfMAligner::ComputePath(int src, int dst)
{
    LOG(INFO) << "Computing Path: " << src << "->" << dst;
    std::queue<int> qu;
    qu.push(src);

    Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    double s = 1.0;

    Sim3 sim(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Identity(), 1.0);
    LOG(INFO) << "v: " << src;
    while (!qu.empty()) {
        int u = qu.front(); qu.pop();
        auto it = paths_[u].begin();
        int v = it->first;
        LOG(INFO) << "v: " << v;
        s = it->second.s * s;
        r = it->second.R * r.eval();
        t = it->second.s * it->second.R * t.eval() + it->second.t;
        if (v == dst) {
            sim.s = s; sim.R = r; sim.t = t;
            sim3_to_anchor_[src] = sim;
            return;
        }
        else qu.push(v);
    }
    LOG(INFO) << "\n";
}

void SfMAligner::MergeReconstructions()
{
    if (options_.assign_color_for_clusters) {
        for (size_t i = 0; i < reconstructions_.size(); i++) {
            reconstructions_[i]->AssignColorsForAllPoints(SfMAligner::ColorContainers[i]);
            // // Assign cluster id for each image.
            // const std::vector<image_t> reg_image_ids = reconstructions_[i]->RegImageIds();
            // for (auto image_id : reg_image_ids) {
            //     Image& image = reconstructions_[i]->Image(image_id);
            //     image.SetClusterId(i);
            // }
        }
    }

    for (uint i = 0; i < reconstructions_.size(); i++) {
        if (static_cast<int>(i) == anchor_node_.id) { continue; }

        Sim3 sim3 = sim3_to_anchor_[i];
        Eigen::Matrix3x4d alignment;
        alignment.block(0, 0, 3, 3) = sim3.s * sim3.R;
        alignment.block(0, 3, 3, 1) = sim3.t;

        reconstructions_[anchor_node_.id]->Merge(*reconstructions_[i],
                                                 alignment);
    }
}

bool SfMAligner::AdjustGlobalBundle()
{
    Reconstruction* final_recon = reconstructions_[anchor_node_.id];
    CHECK_NOTNULL(final_recon);

    const std::vector<image_t>& reg_image_ids = final_recon->RegImageIds();

    CHECK_GE(reg_image_ids.size(), 2) << "At least two images must be "
                                         "registered for global bundle-adjustment";
    
    // Avoid degeneracies in bundle adjustment.
    final_recon->FilterObservationsWithNegativeDepth();

    // Configure bundle adjustment
    BundleAdjustmentConfig ba_config;
    for (const image_t image_id : reg_image_ids) {
        ba_config.AddImage(image_id);
    }

    // Fix 7-DOFs of the bundle adjustment problem.
    ba_config.SetConstantPose(reg_image_ids[0]);
    ba_config.SetConstantTvec(reg_image_ids[1], {0});

    // Run bundle adjustment.
    BundleAdjuster bundle_adjuster(ba_options_, ba_config);
    if (!bundle_adjuster.Solve(final_recon)) {
        return false;
    }

    // Normalize scene for numerical stability and
    // to avoid large scale changes in viewer.
    final_recon->Normalize();

    return true;
}



} // namespace GraphSfM