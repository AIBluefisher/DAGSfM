#ifndef SRC_CONTROLLERS_SFM_ALIGNER_H_
#define SRC_CONTROLLERS_SFM_ALIGNER_H_

#include <vector>
#include <memory>

#include "base/reconstruction.h"
#include "base/similarity_transform.h"
#include "estimators/sim3.h"
#include "optim/bundle_adjustment.h"
#include "graph/graph.h"

using namespace colmap;
using namespace GraphSfM::graph;

namespace GraphSfM {

using SimilarityGraph = std::unordered_map<size_t, std::unordered_map<size_t, Sim3>>;

class SfMAligner
{
public:
    struct AlignOptions
    {
        double min_inlier_observations = 0.3;

        double max_reprojection_error = 1.5;
    };

    struct Summary
    {
        double recon_graph_construction_time;

        double merging_time;

        double final_ba_time;
    };

    SfMAligner(const std::vector<Reconstruction*>& reconstructions,
               const BundleAdjustmentOptions& ba_options);

    bool Align();

    Node GetAnchorNode() const;

    std::vector<Sim3> GetSim3ToAnchor() const;

private:
    AlignOptions options_;

    BundleAdjustmentOptions ba_options_;

    std::vector<Reconstruction*> reconstructions_;

    Graph<Node, Edge> recons_graph_;

    Node anchor_node_;

    // The similarity transformation to anchor node, which is used for final transformation
    std::vector<Sim3> sim3_to_anchor_;

    std::vector<std::unordered_map<size_t, Sim3>> paths_;

    // similarity graph, each cluster is represented as a node,
    // and each edge is the relative similarity transformation
    SimilarityGraph sim3_graph_;    

    void ConstructReconsGraph();

    double ComputeEdgeWeight(const uint i, const uint j);

    void FindAnchorNode(Graph<Node, Edge>* graph);

    void ComputePath(int src, int dst);

    void MergeReconstructions();

    bool AdjustGlobalBundle();
};

double MeanReprojectionResiduals(const std::vector<double>& residuals);

std::vector<double> ComputeReprojectionResiduals(
                                    const std::vector<Eigen::Vector3d>& src_points,
                                    const std::vector<Eigen::Vector3d>& ref_points,
                                    const Eigen::Matrix3x4d& alignment);

double CheckReprojError(const std::vector<Eigen::Vector3d>& src_observations,
                        const std::vector<Eigen::Vector3d>& dst_observations,
                        const double& scale,
                        const Eigen::Matrix3d& R,
                        const Eigen::Vector3d& t);

double CheckAngularResidual(const std::vector<Eigen::Matrix3d>& src_rotations,
                           const std::vector<Eigen::Matrix3d>& dst_rotations,
                           const Eigen::Matrix3d& R);

void FindSimilarityTransform(const std::vector<Eigen::Vector3d>& observations1,
                             const std::vector<Eigen::Vector3d>& observations2,
                             Eigen::Matrix3d& R,
                             Eigen::Vector3d& t,
                             double& scale,
                             double& msd);

bool ComputeSimilarityByCameraMotions(
                            std::vector<Eigen::Vector3d>& camera_centers1,
                            std::vector<Eigen::Vector3d>& camera_centers2,
                            std::vector<Eigen::Matrix3d>& camera_rotations1,
                            std::vector<Eigen::Matrix3d>& camera_rotations2,
                            Eigen::Matrix3d& relative_r,
                            Eigen::Vector3d& relative_t,
                            double& scale);

} // namespace GraphSfM

#endif