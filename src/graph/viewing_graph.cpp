#include <utility>
#include <cmath>

#include "viewing_graph.h"
#include "union_find.h"
#include "matching/matcher.h"

#include <glog/logging.h>

// #define USE_HOMOGRAPHY

namespace GraphSfM {
namespace graph {

ViewingGraph::ViewingGraph()
{
    _tried_match_times = 0;
}

ViewingGraph::ViewingGraph(const std::string& matches_dir, const std::string& filename)
    : ViewingGraph()
{
    _matcher.LoadRegionsFromFile(matches_dir, filename);
}

ViewingGraph::ViewingGraph(const std::string& matches_dir, 
                           const std::string& filename,
                           const Graph<ImageNode, Edge>& vg, 
                           const ViewingGraphOption& vg_option)
    : ViewingGraph()
{
    ViewingGraph(matches_dir, filename);
    SetInitialGraph(vg);
    SetVGOption(vg_option);
}

ViewingGraph::~ViewingGraph()
{
    // LOG(INFO) << "Total tried matches time: " << _tried_match_times;
    // LOG(INFO) << "Final survived matches pairs: " << _matcher.GetMatches().size();
    // LOG(INFO) << "Total matches number: " << _matcher.TotalMatchesNumber();
}

bool ViewingGraph::SetInitialGraph(const Graph<ImageNode, Edge>& vg)
{
    _graph = vg.Clone();
}

bool ViewingGraph::SetVGOption(const ViewingGraphOption& vg_option)
{
    _vg_option = vg_option;
}

Graph<ImageNode, VGEdge> ViewingGraph::GetViewingGraph() const
{
    return _view_graph;
}

ViewingGraphOption ViewingGraph::GetVGOption() const
{
    return _vg_option;
}

size_t ViewingGraph::GetComponentNumber() const
{
    return _view_graph.FindConnectedComponents();
}

void ViewingGraph::FullMatching()
{
    // TODO:
}

void ViewingGraph::QueryExpansion()
{
    // TODO:
}

void ViewingGraph::OnlineMST()
{
    UnionFind union_find(this->_graph.GetSize());
    std::priority_queue<Edge> edges = _graph.CollectEdges();
    
    std::vector<VGEdge> mst_edges;
    std::vector<size_t> ft(_graph.GetSize(), 0); // failure time
    
    while (!edges.empty()) {
        Edge edge = edges.top(); edges.pop();
        size_t i = union_find.FindRoot(edge.src);
        size_t j = union_find.FindRoot(edge.dst);
        
        if (i != j && 
            ft[i] <= _vg_option.sr_thresh && 
            ft[j] <= _vg_option.sr_thresh) {
            this->SetVisitedStatus(i, j, true);
            // Verify if pair (i, j) pass the match inlier check
            VGEdge vg_edge;
            if (SatisfyMatchInlierCheck(i, j, vg_edge)) {
                LOG(INFO) << "pair: (" << i << ", " << j << ") " 
                          << "satisfy match inlier threshold" << std::endl;
                union_find.Union(i, j);
                _matcher.CacheValidMatches(i, j);
                mst_edges.push_back(vg_edge);
            }
        }
        else { ft[i]++; ft[j]++; }
        _matcher.Clear();
    }
    LOG(INFO) << "[OnlineMST]-Total tried matching times: " << _tried_match_times;
    LOG(INFO) << "[OnlineMST]-Survived matching pairs: " << _matcher.GetMatches().size();
    LOG(INFO) << "[OnlienMST]-Total matches number: " << _matcher.TotalMatchesNumber();

    this->ConstructMST(mst_edges);
}

void ViewingGraph::SetVisitedStatus(const size_t& i, const size_t& j, bool flag)
{
    size_t lo = std::min(i, j), hi = std::max(i, j);
    _visited_edges[lo][hi] = flag;
}

void ViewingGraph::ConstructMST(const std::vector<VGEdge>& mst_edges)
{
    auto nodes = _graph.GetNodes();

    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        _view_graph.AddNode(it->second);
    }
    for (auto edge : mst_edges) { _view_graph.AddEdge(edge); }

    // initialize _visited_edges
    std::vector<ImageNode> sequential_nodes = _view_graph.SerializeNodes();
    for (size_t i = 0; i < sequential_nodes.size(); i++) {
        for (size_t j = i + 1; j < sequential_nodes.size(); j++) {
            _visited_edges[i][j] = false;
        }
    }
}

void ViewingGraph::StrongTripletsExpansion()
{
    for (int i = 0; i < _vg_option.exp_times; i++) {
        std::vector<Triplet> weak_triplets = this->CollectWeakTriplet();
        // #pragma omp parallel 
        // {
            // #pragma omp for
            for (int k = 0; k < weak_triplets.size(); k++) {
                Triplet weak_triplet = weak_triplets[k];
                size_t l = weak_triplet.left_node;
                size_t r = weak_triplet.right_node;
                this->SetVisitedStatus(l, r, true);

                // Verify if pair (i, j) pass the match inlier check
                VGEdge vg_edge;
                if (SatisfyMatchInlierCheck(l, r, vg_edge)) {
                    LOG(INFO) << "pair: (" << l << ", " << r << ") " 
                              << "satisfy match inlier check";
                    if (_vg_option.loop_consistency_enable) {
                        weak_triplet.edges.push_back(vg_edge);
                        this->RectifyTripletDirection(weak_triplet);

                        std::vector<Eigen::Matrix3d> matrices;
                        for (auto edge : weak_triplet.edges) { matrices.push_back(edge.M); }

                        // Verify if weak_triplet pass the loop consistency check
                        if (SatisfyLoopConsistency(matrices)) { 
                            LOG(INFO) << "pair: (" << l << ", " << r << ") " 
                                      << "satisfy loop-consistency threshold";
                            _matcher.CacheValidMatches(l, r);
                            _view_graph.AddEdge(vg_edge); 
                        // #ifdef __DEBUG__
                            _valid_loops[3]++;
                        // #endif
                        } 
                    // #ifdef __DEBUG__ 
                        else { _invalid_loops[3]++; } 
                    // #endif
                    } else {
                        _matcher.CacheValidMatches(l, r);
                        _view_graph.AddEdge(vg_edge); 
                    }
                } // end match inlier contidition check
                _matcher.Clear();
            } // end triplet traverse
        // }
    }
    LOG(INFO) << "[TripletsExpansion]-Total tried matching times: " << _tried_match_times;
    LOG(INFO) << "[TripletsExpansion]-Survived matching pairs: " << _matcher.GetMatches().size();
    LOG(INFO) << "[TripletsExpansion]-Total matches number: " << _matcher.TotalMatchesNumber();
}

void ViewingGraph::GraphReinforcement()
{
    size_t cc_num = _view_graph.FindConnectedComponents();
    LOG(INFO) << "connected components number is: " << cc_num << std::endl;
    _vg_option.component_number = std::max(2 * cc_num, _vg_option.component_number) ;

    std::unordered_map<int, int> cluster_map = 
         _view_graph.NormalizedCut(_vg_option.component_number);

    // Generate components (a component is a view cluster)
    std::vector<std::vector<size_t>> components = this->ConstructComponents(cluster_map);

    LOG(INFO) << "components number is: " << components.size() << std::endl;

    // validate putative matches between component i and component j
    int component_num = components.size();
    // #pragma omp parallel
    // {   
        // #pragma omp for
        for (int i = 0; i < component_num; i++) {
            for (int j = i + 1; j < component_num; j++) {
                std::vector<std::pair<size_t, size_t>> ranks = 
                            this->RankComponentWisePairs(components[i], components[j]);
                for (auto rank : ranks) {
                    size_t l = rank.first, r = rank.second;
                    VGEdge vg_edge;

                    size_t lo = std::min(l, r), hi = std::max(l, r);
                    if (_visited_edges[lo][hi]) { continue; }
                    else { _visited_edges[lo][hi] = true; }

                    if (SatisfyMatchInlierCheck(l, r, vg_edge)) {
                        LOG(INFO) << "pair: (" << l << ", " << r << ") " 
                                  << "satisfy match inlier threshold";
                        if (_vg_option.loop_consistency_enable) {
                            std::vector<VGEdge> path = _view_graph.ShortestPath(l, r);
                            if (path.empty()) {
                                _matcher.CacheValidMatches(i, j);
                                _view_graph.AddEdge(vg_edge);
                            } else {
                                // this->RectifyMatrix(vg_edge);
                                path.push_back(vg_edge);
                                this->RectifyPathDirection(path);
                                std::vector<Eigen::Matrix3d> matrices;
                                for (auto edge : path) { matrices.push_back(edge.M); }

                                if (SatisfyLoopConsistency(matrices)) { 
                                    LOG(INFO) << "pair: (" << l << ", " << r << ") " 
                                              << "loop consistency check";
                                    _matcher.CacheValidMatches(l, r);
                                    _view_graph.AddEdge(vg_edge); 
                                // #ifdef __DEBUG__
                                    _valid_loops[matrices.size()]++;
                                //  #endif
                                } 
                            // #ifdef __DEBUG__ 
                                else { _invalid_loops[matrices.size()]++; }
                            // #endif
                            }
                        } else {
                            _matcher.CacheValidMatches(l, r);
                            _view_graph.AddEdge(vg_edge);
                        }
                    }
                    _matcher.Clear();
                }
            } // end inner j-loop
        } // end outer i-loop
    // }

    LOG(INFO) << "[GraphReinforcement]-Total tried matching times: " << _tried_match_times;
    LOG(INFO) << "[GraphReinforcement]-Survived matching pairs: " << _matcher.GetMatches().size();
    LOG(INFO) << "[GraphReinforcement]-Total matches number: " << _matcher.TotalMatchesNumber();
}

void ViewingGraph::RetainSingletonNodes()
{
    std::vector<ImageNode> singleton_nodes = _view_graph.FindSingletonNodes();
    LOG(INFO) << "After OnlineMST, there are " << singleton_nodes.size()
              << " singleton nodes" << std::endl;
    std::vector<std::unordered_map<size_t, Edge>> edges = _graph.SerializeEdges();
    for (auto single_node : singleton_nodes) {
        size_t i = single_node.id;
        auto em = edges[i];
        int retain_time = 0;
        for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
            size_t j = em_it->second.dst;
            size_t lo = std::min(i, j), hi = std::max(i, j);
            if (!_visited_edges[lo][hi] && (retain_time++ < _vg_option.rs_number)) {
                _visited_edges[lo][hi] = true;
                // Verify if pair (i, j) pass the match inlier check
                VGEdge vg_edge;
                if (SatisfyMatchInlierCheck(i, j, vg_edge)) {
                    LOG(INFO) << "pair: (" << i << ", " << j << ") " 
                              << "satisfy match inlier threshold" << std::endl;
                    _matcher.CacheValidMatches(i, j);
                    _view_graph.AddEdge(vg_edge); 
                }
                _matcher.Clear();
            }
        }
    }
    LOG(INFO) << "[RetainSingletonNodes]-Total tried matching times: " << _tried_match_times;
    LOG(INFO) << "[RetainSingletonNodes]-Survived matching pairs: " << _matcher.GetMatches().size();
    LOG(INFO) << "[RetainSingletonNodes]-Total matches number: " << _matcher.TotalMatchesNumber();
}

void ViewingGraph::ExportToDisk(const std::string& dir, const std::string& filename) const
{
    _matcher.ExportMatches(dir, filename);
}

void ViewingGraph::ShowStatistics() const
{
    for (auto it = _valid_loops.begin(); it != _valid_loops.end(); ++it) {
        LOG(INFO) << "Valid " << it->first << "-len loops: " << it->second;
    }

    for (auto it = _invalid_loops.begin(); it != _invalid_loops.end(); ++it) {
        LOG(INFO) << "Invalid " << it->first << "-len loops: " << it->second;
    }
}

std::vector<Triplet> ViewingGraph::CollectWeakTriplet() const
{
    std::vector<Triplet> weak_triplets;
    std::unordered_map<size_t, ImageNode> nodes = _view_graph.GetNodes();
    std::unordered_map<size_t, EdgeMap> edges = _view_graph.GetEdges();

    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        size_t idx = it->first;
        EdgeMap linked_edges = edges[idx];
        if (linked_edges.size() < 2) continue;
        
        std::vector<VGEdge> vec_edges;
        for (auto it = linked_edges.begin(); it != linked_edges.end(); ++it) {
            vec_edges.push_back(it->second);
        }

        for (int i = 0; i < vec_edges.size(); i++) {
            for (int j = i + 1; j < vec_edges.size(); j++) {
                size_t l = vec_edges[i].dst, r = vec_edges[j].dst;
                size_t lo = std::min(l, r), hi = std::max(l, r);

                // weak triplet must not be a loop
                if (edges[lo].find(hi) == edges[lo].end()  && 
                    !_visited_edges.at(lo).at(hi)) {
                    std::vector<VGEdge> tmp_edges = { vec_edges[i], vec_edges[j] };
                    weak_triplets.push_back(Triplet(idx, l, r, tmp_edges));
                }
            }
        }
    }

    return weak_triplets;
}

bool ViewingGraph::SatisfyLoopConsistency(const std::vector<Eigen::Matrix3d>& matrices) const
{
    Eigen::Matrix3d chained_M = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    size_t len = matrices.size();

    for (auto mat : matrices) chained_M = chained_M * mat;

    float trace = chained_M.trace();
    float cost = acos((trace - 1.0) / 2.0);
    LOG(INFO) << len << "-Loop-Consistency discrepency: " << cost << std::endl;

    // if (cost < _vg_option.dc_thresh / std::sqrt(len)) return true;
    if (cost < _vg_option.dc_thresh) return true;
    return false;
}

bool ViewingGraph::SatisfyMatchInlierCheck(const size_t& i, const size_t& j, VGEdge& vg_edge)
{
    const std::string filename_l = _graph.GetNode(i).img_path;
    const std::string filename_r = _graph.GetNode(j).img_path;
    
    _matcher.LoadPairwiseRegions(i, j, filename_l, filename_r);
    _matcher.InitialMatching();

    std::vector<uint32_t> vec_inliers;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    _matcher.EssentialFiltering(i, j, R, vec_inliers);

    // Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    // _matcher.FundamentalFiltering(F, vec_inliers);
    // Eigen::Matrix3d K_l, K_r;
    // if (!_matcher.ReadIntrinsic(i, K_l)) {
        // std::cerr << "Cannot read intrinsic parameters of view " << i << std::endl;
        // return;
    // }
    // if (!_matcher.ReadIntrinsic(j, K_r)) {
        // std::cerr << "Cannot read intrinsic parameters of view " << j << std::endl;
        // return;
    // }

    // Eigen::Matrix3d E = K_r.transpose() * F * K_l;
    _tried_match_times++;
    LOG(INFO) << "inliers number is: " << vec_inliers.size() << std::endl;

#ifdef USE_HOMOGRAPHY
    // TODO: compute angle according to R
    double angle;
    double thresh;

    if (angle < thresh) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
        _matcher.HomographyFiltering(H, vec_inliers);
        vg_edge.M = H;
        vg_edge.mat_type = HOMOGRAPHY;
    } else {
        vg_edge.M = R;
        vg_edge.mat_type = ROTATION;
    }
#else
    vg_edge.M = R;
    vg_edge.mat_type = ROTATION;
#endif

    vg_edge.src = i;
    vg_edge.dst = j;
    vg_edge.weight = vec_inliers.size();

    return vec_inliers.size() >= _vg_option.mi_thresh;
}

std::vector<std::vector<size_t>> 
ViewingGraph::ConstructComponents(const std::vector<size_t>& clusters) const
{
    std::vector<std::vector<size_t>> components;

    for(int i = 0; i < _vg_option.component_number; i++) {
        std::vector<size_t> component;
        
        for(int j = 0; j < clusters.size(); j++) {
            if(clusters[j] == i) { component.push_back(j); }
        }
        components.push_back(component);
    }
    return components;
}

std::vector<std::vector<size_t>> 
ViewingGraph::ConstructComponents(const std::unordered_map<int, int>& cluster_map) const 
{
    std::vector<std::vector<size_t>> components(_vg_option.component_number);

    for (auto it = cluster_map.begin(); it != cluster_map.end(); ++it) {
        components[it->second].push_back(it->first);
    }
    return components;
}

void ViewingGraph::RectifyMatrix(VGEdge& edge)
{
    if (edge.mat_type == ROTATION) { 
        edge.M = (edge.M).transpose(); 
    } else { 
        edge.M = (edge.M).inverse(); 
    }
}

void ViewingGraph::RectifyTripletDirection(Triplet& triplet)
{
    size_t l = triplet.left_node;
    size_t r = triplet.right_node;
    size_t k = triplet.joint_node;
    std::vector<VGEdge>& edges = triplet.edges;

    for (auto& edge : edges) {
        if ((edge.src == k && edge.dst == r) ||
            (edge.src == l && edge.dst == k)) {
            this->RectifyMatrix(edge);
        }
    }
}

void ViewingGraph::RectifyPathDirection(std::vector<VGEdge>& path)
{
    for (auto& edge : path) {
        if (!_view_graph.HasEdge(edge.src, edge.dst)) {
            this->RectifyMatrix(edge);
        }
    }
}

std::vector<std::pair<size_t, size_t>> ViewingGraph::RankComponentWisePairs(
                                          const std::vector<size_t>& component1,
                                          const std::vector<size_t>& component2)
{
    std::priority_queue<Edge> edges;
    for (auto i : component1) {
        for (auto j : component2) {
            // if (_graph.HasEdge(i, j)) { edges.push(_graph.GetEdge(i, j)); }
            // else if (_graph.HasEdge(j, i)) { edges.push(_graph.GetEdge(j, i)); }
            size_t lo = std::min(i, j), hi = std::max(i, j);
            if (_graph.HasEdge(lo, hi) && !_visited_edges[lo][hi]) { 
                edges.push(_graph.GetEdge(lo, hi)); 
            }
            // if (edges.size() > _vg_option.cw_number) break;
        }
        // if (edges.size() > _vg_option.cw_number) break;
    }

    std::vector<std::pair<size_t, size_t>> ranked_pairs;
    size_t sum = 0;
    while (!edges.empty() && (sum++ <= _vg_option.cw_number)) {
        Edge edge = edges.top(); edges.pop();
        ranked_pairs.push_back(std::make_pair(edge.src, edge.dst));
    }
    return ranked_pairs;
}

} // namespace graph
} // namespace GraphSfM