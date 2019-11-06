// References:
// [CVPR 2006] Nister D, Stewenius H. Scalable Recognition with a Vocabulary Tree[J]. 
//             Proc Cvpr, 2006, 2(10):2161-2168.
// [ICCV 2009] Agarwal S , Snavely N , Simon I , et al. Building Rome in a day[C]// 
//             IEEE International Conference on Computer Vision. IEEE, 2009.
// [ECCV 2016] Shen T, Zhu S, Fang T, et al. Graph-Based Consistent Matching for Structure-from-Motion[C]//
//             European Conference on Computer Vision. 2016.
// [3DV 2017] Cui Q, Fragoso V, Sweeney C, et al. GraphMatch: Efficient Large-Scale Graph 
//            Construction for Structure from Motion[J]. 2017.

#ifndef GRAPH_VIEWING_GRAPH_H
#define GRAPH_VIEWING_GRAPH_H

#include "graph.h"
#include "matching/matcher.h"

#include "Eigen/Core"
#include "Eigen/Dense"

#define __DEBUG__

namespace GraphSfM {
namespace graph {

struct QueryExpansionOption
{
    QueryExpansionOption(const size_t x, const size_t y, const size_t l)
    {
        k1 = x;
        k2 = y;
        expansion_level = l;
    }

    size_t k1;
    size_t k2;
    size_t expansion_level;
};

struct ViewingGraphOption 
{
    ViewingGraphOption(const size_t srt = 20, const size_t mit = 20, 
                       const float dct = 2, const size_t cwn = 30, 
                       const size_t ept = 3, const size_t cn = 2,
                       const size_t rsn = 20, const bool lcce = true) 
    {
        sr_thresh = srt;
        rs_number = rsn;
        mi_thresh = mit;
        dc_thresh = dct;
        cw_number = cwn;
        exp_times = ept;
        component_number = cn;
        loop_consistency_enable = lcce;
    }

    ViewingGraphOption(const ViewingGraphOption& vg_option) 
    {
        sr_thresh = vg_option.sr_thresh;
        rs_number = vg_option.rs_number;
        mi_thresh = vg_option.mi_thresh;
        dc_thresh = vg_option.dc_thresh;
        cw_number = vg_option.cw_number;
        exp_times = vg_option.exp_times;
        component_number = vg_option.component_number;
    }

    size_t sr_thresh;               // singleton node rejection threshold
    size_t rs_number;               // upper bound to retain singleton nodes
    size_t mi_thresh;               // match inlier threshold
    float dc_thresh;                // discrepency threshold
    size_t cw_number;               // community-wise match number
    size_t exp_times;               // graph expansion times
    size_t component_number;        // the number of components that used in graph-cut
    bool loop_consistency_enable;   // whether to check loop consistency
};

// Indicate use rotation matrix or homograph matrix to verify loop consistency
enum GeometricRelationType { ROTATION, HOMOGRAPHY, FUNDAMENTAL };

struct VGEdge : Edge
{
    /* Matrix that used for checking the loop consistency.
     * A relative rotation matrix may be used in general case.
     * While a homography matrix should be used for coplanar scenes,
     * as there is little rotation for checking loop consistency.
     */  
    Eigen::Matrix3d M;
    GeometricRelationType mat_type;
    
    VGEdge() { }

    VGEdge(size_t src, size_t dst, float weight) : Edge(src, dst, weight) { }

    VGEdge(size_t src, size_t dst, float weight, Eigen::Matrix3d mat, GeometricRelationType type) 
    : Edge(src, dst, weight)
    {
        M = mat;
        mat_type = type;
    }

    VGEdge(const VGEdge& edge)
    {
        src = edge.src;
        dst = edge.dst;
        weight = edge.weight;
        M = edge.M;
    }
};

/*
 * Triplet that used for checking weak loop consistency
 * if edges.size() == 2, then indicates a weak triplet
 * if edges.size() == 3, then indicates a strong triplet
 * after passing the loop consistency checking, weak triplet becomes strong triplet
 */ 
struct Triplet 
{
    size_t joint_node;          // the node that connect the two edges in a weak triplet
    size_t left_node;
    size_t right_node;
    std::vector<VGEdge> edges;

    Triplet() {}
    Triplet(const size_t& jn, const size_t& ln, const size_t& rn, const std::vector<VGEdge>& es)
    {
        joint_node = jn;
        left_node = ln;
        right_node = rn;
        edges.assign(es.begin(), es.end());
    }
};

typedef std::unordered_map<size_t, VGEdge> EdgeMap;
class ViewingGraph 
{
private:
    Graph<ImageNode, Edge> _graph;
    Graph<ImageNode, VGEdge> _view_graph;
    ViewingGraphOption _vg_option;
    matching::Matcher _matcher;
    std::unordered_map<size_t, std::unordered_map<size_t, bool>> _visited_edges;
    size_t _tried_match_times;

#ifdef __DEBUG__
    std::unordered_map<size_t, size_t> _valid_loops;
    std::unordered_map<size_t, size_t> _invalid_loops;
#endif

public:
    // constructors
    ViewingGraph();
    ViewingGraph(const std::string& matches_dir, const std::string& filename);
    ViewingGraph(const std::string& matches_dir, 
                 const std::string& filename, 
                 const Graph<ImageNode, Edge>& graph, 
                 const ViewingGraphOption& vg_option);

    // destructor
    ~ViewingGraph();

    // set functions
    bool SetInitialGraph(const Graph<ImageNode, Edge>& vg);
    bool SetVGOption(const ViewingGraphOption& vg_option);

    // get functions
    Graph<ImageNode, VGEdge> GetViewingGraph() const;
    ViewingGraphOption GetVGOption() const;
    size_t GetComponentNumber() const;

    // full matching
    void FullMatching();

    // query expansion
    void QueryExpansion();

    // algorithms for viewing graph building
    void OnlineMST();
    void StrongTripletsExpansion();
    void GraphReinforcement();

    // When singleton nodes appear after OnlineMST,
    // try to match it with nodes ranked by similarity score
    void RetainSingletonNodes();

    void ExportToDisk(const std::string& dir, const std::string& filename) const;

    void ShowStatistics() const;

private:
    // helper functions
    void SetVisitedStatus(const size_t& i, const size_t& j, bool flag);
    void ConstructMST(const std::vector<VGEdge>& mst_edges);
    std::vector<Triplet> CollectWeakTriplet() const;

    bool SatisfyLoopConsistency(const std::vector<Eigen::Matrix3d>& matrices) const;
    bool SatisfyMatchInlierCheck(const size_t& i, const size_t& j, VGEdge& vg_edge);

    std::vector<std::vector<size_t>> 
    ConstructComponents(const std::vector<size_t>& clusters) const;
    std::vector<std::vector<size_t>> 
    ConstructComponents(const std::unordered_map<int, int>& cluster_map) const;
    void RectifyMatrix(VGEdge& edge);
    void RectifyTripletDirection(Triplet& triplet);
    void RectifyPathDirection(std::vector<VGEdge>& path);

    std::vector<std::pair<size_t, size_t>> RankComponentWisePairs(
                                        const std::vector<size_t>& component1,
                                        const std::vector<size_t>& component2);
};

} // namespace graph
} // namespace GraphSfM


#endif