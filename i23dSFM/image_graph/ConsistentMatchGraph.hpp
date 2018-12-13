#ifndef CONSISTENT_MATCH_H
#define CONSISTENT_MATCH_H

#include <queue>

#include "MatchGraph.hpp"

using namespace std;

namespace i23dSFM{

// Algorithm used in this class comes from
// "Shen T, Zhu S, Fang T, et al. Graph-Based Consistent Matching for Structure-from-Motion[M]// 
// Computer Vision – ECCV 2016. Springer International Publishing, 2016:139-155."
class ConsistentMatchGraph
{
private:
    MatchGraph _tripletGraph;   // a match graph after triplet expansion
    MatchGraph _finalGraph;     // final match graph after component merging
    size_t** _generalGraph;     // a general adjacent matrix

    /**
     * @brief Construct nodes of match graph from other image graph
     * @return True if construct succeed
     */  
    bool MakeNode(const ImageGraph g);
    /**
     * @brief Order edge by weight
     * @param image graph with edge weight re-computed by qudratic mean edge
     * @return a priority queue ordered by edge weight
     */
    priority_queue<LinkEdge> OrderEdge(const ImageGraph g) const;
    /** TODO
     * @brief
     * 
     */
    void ComputeCommunityStructure();
    /** TODO
     * @brief
     * 
     */
    priority_queue<LinkEdge> CreateCandidateMatchSet(); 
    /** TODO
     * @brief
     * 
     */
    // vector<size_t> FindShortestPath(size_t src, size_t dst); 
    /**
     * @brief Compute discrepancy angle
     * @param R: relative rotation matrix
     */ 
    double ComputeDiscrepancy(Eigen::Matrix3d R);

public:
    // constructor
    // ConsistentMatchGraph();
    /**
     * @brief Get triplet match graph (a minimum spanning tree) 
     */
    ImageGraph GetTripletGraph() const;
    /**
     * @brief Get final match graph 
     */
    ImageGraph GetFinalGraph() const;
    /**
     * @brief Online Minimum Spanning Tree algorithm for match graph initialization
     * @param g: original image graph generated by vocabulary tree index
     * @param rejectThresh: singleton rejection threshold
     * @param inlierThresh: match inlier threshold
     */
    void OnlineMST(const ImageGraph g, size_t rejectThresh = 20, size_t inlierThresh = 40);
    /**
     * brief Graph expansion by strong triplets
     * @param discreThresh: discrepancy threshold, metric is degree
     */
    void StrongTripletExpansion(const ImageGraph g, double discreThresh = 2.0); 
    /**
     * @brief Component Merging Algorithm
     * @param communityScale: community-wise match number
     * @param loopDiscreThresh: loop discrepancy threshold
     */
    void ComponentMerging(size_t communityScale, double loopDiscreThresh); 


};
    
} // namespace i23dSFM

#endif