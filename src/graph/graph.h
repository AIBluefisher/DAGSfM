#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <string>
#include <queue>
#include <unordered_map>
#include <vector>
#include <utility>
// #include <fstream>

// #include "union_find.h"
// #include "log/log.h"

// #include "stlplus3/stlplus.h"
// extern "C" {
//   #include "graclus/graclus/Graclus.h"
// }

namespace GraphSfM {
namespace graph {

struct Node 
{
    int id;
    // std::string img_path;

    Node() { id = -1; }

    Node(size_t idx)
    {
        id = idx;
    }

    // Node(size_t idx, const std::string& path)
    // {
    //     id = idx;
    //     img_path = path;
    // }

    Node(const Node& node) 
    {
        id = node.id;
        // img_path = node.img_path;
    }

    bool operator==(const Node& node)
    {
        return id == node.id;
    }

    // sort in ascending order
    static bool CompareById(const Node& node1, const Node& node2)
    {
        return node1.id < node2.id;
    }
};

struct ImageNode : Node
{
    std::string img_path;

    ImageNode()
    { 
        id = -1;
        img_path = "";
    }

    ImageNode(const size_t& idx, const std::string& path = "")
    : Node(idx)
    {
        img_path = path;
    }

    ImageNode(const ImageNode& img_node)
    {
        id = img_node.id;
        img_path = img_node.img_path;
    }
};

struct Edge 
{
    size_t src;
    size_t dst;
    float weight;

    Edge() {}

    Edge(size_t i, size_t j)
    {
        src = i;
        dst = j;
        weight = 0.0f;
    }

    Edge(size_t i, size_t j, float w)
    {
        src = i;
        dst = j;
        weight = w;
    }

    Edge(const Edge& edge)
    {
        src = edge.src;
        dst = edge.dst;
        weight = edge.weight;
    }

    // sort in descending order
    friend bool operator < (const Edge& edge1, const Edge& edge2)
    {
        return edge1.weight < edge2.weight;
    }
};


// T1: type of node, T2: type of edge
template <typename T1, typename T2>
class Graph 
{
typedef std::unordered_map<size_t, T2> EdgeMap; // size_t: dst

private:
    size_t _size;
    std::unordered_map<size_t, T1> _nodes;
    std::unordered_map<size_t, EdgeMap> _edges; // size_t: src
    // degree of nodes: node_id, degree
    std::unordered_map<size_t, size_t> _degrees;
    std::unordered_map<size_t, size_t> _out_degrees;
    std::unordered_map<size_t, size_t> _in_degrees;

public:
    // constructors
    Graph();
    Graph(size_t n);
    Graph(const Graph<T1, T2>& graph);

    // Clone operation
    Graph<T1, T2> Clone() const;

    // Node operation
    T1 GetNode(size_t idx) const;
    std::unordered_map<size_t, T1> GetNodes() const;
    size_t GetNodesNum() const;
    bool HasNode(const size_t& idx) const;
    bool AddNode(const T1& node);
    bool DeleteNode(const size_t& idx);
    bool RemoveSingletonNodes();
    std::vector<T1> FindSingletonNodes();

    // Edge operation
    std::unordered_map<size_t, EdgeMap> GetEdges() const;
    T2 GetEdge(size_t src, size_t dst) const;
    size_t GetEdgesNum() const;
    bool HasEdge(const size_t& src, const size_t& dst) const;
    bool AddEdge(const T2& edge);
    bool AddUEdge(const T2& edge, const T2& rev_edge);
    bool AlterEdge(const T2& edge);
    bool DeleteEdge(const size_t& src, const size_t& dst);
    std::priority_queue<T2> CollectEdges() const;
    T2 FindConnectedEdge(const int& idx) const;

    // graph size (equals to size of nodes)
    size_t GetSize() const;

    // Degree operation
    void CountDegrees();
    std::unordered_map<size_t, size_t> GetDegrees() const;
    void CountOutDegrees();
    std::unordered_map<size_t, size_t> GetOutDegrees() const;
    void CountInDegrees();
    std::unordered_map<size_t, size_t> GetInDegrees() const;

    int FindLeafNode(const std::unordered_map<size_t, size_t>& degrees) const;

    // Minimum Spanning Tree (MST) algorithm
    std::vector<T2> Kruskal() const;

    // breadth-first-search algorithm
    std::vector<T2> ShortestPath(const size_t& src, const size_t& dst) const;

    // Graph-cut algorithm
    std::unordered_map<int, int> NormalizedCut(const size_t cluster_num) const;

    // graph information presentation
    void ShowInfo() const;
    void ShowInfo(const std::string& filename) const;
    // void GraphVisual() const;

    size_t FindConnectedComponents() const;

// private:
    // For convenience of normalized-cut
    std::vector<T1> SerializeNodes() const;
    std::vector<EdgeMap> SerializeEdges() const;

    // Update index of node
    void UpdateGraph();
};

} // namespace graph
} // namespace GraphSfM

#include "graph.inl"

#endif