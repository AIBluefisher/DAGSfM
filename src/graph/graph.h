// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace DAGSfM {
namespace graph {

struct Node {
  int id;

  Node() { id = -1; }

  Node(size_t idx) { id = idx; }

  Node(const Node& node) { id = node.id; }

  bool operator==(const Node& node) { return id == node.id; }

  // sort in ascending order
  static bool CompareById(const Node& node1, const Node& node2) {
    return node1.id < node2.id;
  }
};

struct ImageNode : Node {
  std::string img_path;

  ImageNode() {
    id = -1;
    img_path = "";
  }

  ImageNode(const size_t& idx, const std::string& path = "") : Node(idx) {
    img_path = path;
  }

  ImageNode(const ImageNode& img_node) {
    id = img_node.id;
    img_path = img_node.img_path;
  }
};

struct Edge {
  size_t src;
  size_t dst;
  float weight;

  Edge() {}

  Edge(size_t i, size_t j) {
    src = i;
    dst = j;
    weight = 0.0f;
  }

  Edge(size_t i, size_t j, float w) {
    src = i;
    dst = j;
    weight = w;
  }

  Edge(const Edge& edge) {
    src = edge.src;
    dst = edge.dst;
    weight = edge.weight;
  }

  // sort in descending order
  friend bool operator<(const Edge& edge1, const Edge& edge2) {
    return edge1.weight < edge2.weight;
  }
};

// NodeType: type of node, EdgeType: type of edge
template <typename NodeType, typename EdgeType>
class Graph {
 public:
  typedef std::unordered_map<size_t, EdgeType> EdgeMap;  // size_t: dst
  // constructors
  Graph();
  Graph(size_t n);
  Graph(const Graph<NodeType, EdgeType>& graph);

  // Clone operation
  Graph<NodeType, EdgeType> Clone() const;

  // Node operation
  NodeType GetNode(size_t idx) const;
  const std::unordered_map<size_t, NodeType>& GetNodes() const;
  size_t GetNodesNum() const;
  bool HasNode(const size_t& idx) const;
  bool AddNode(const NodeType& node);
  bool DeleteNode(const size_t& idx);
  bool RemoveSingletonNodes();
  std::vector<NodeType> FindSingletonNodes();

  // Edge operation
  const std::unordered_map<size_t, EdgeMap>& GetEdges() const;
  EdgeType GetEdge(size_t src, size_t dst) const;
  size_t GetEdgesNum() const;
  bool HasEdge(const size_t& src, const size_t& dst) const;
  bool AddEdge(const EdgeType& edge);
  bool AddUEdge(const EdgeType& edge, const EdgeType& rev_edge);
  bool AlterEdge(const EdgeType& edge);
  bool DeleteEdge(const size_t& src, const size_t& dst);
  std::priority_queue<EdgeType> CollectEdges() const;
  EdgeType FindConnectedEdge(const int& idx) const;

  // graph size (equals to size of nodes)
  size_t GetSize() const;

  // Degree operation
  void CountDegrees();
  const std::unordered_map<size_t, size_t>& GetDegrees() const;
  void CountOutDegrees();
  const std::unordered_map<size_t, size_t>& GetOutDegrees() const;
  void CountInDegrees();
  const std::unordered_map<size_t, size_t>& GetInDegrees() const;

  int FindLeafNode(const std::unordered_map<size_t, size_t>& degrees) const;

  // Minimum Spanning Tree (MST) algorithm
  std::vector<EdgeType> Kruskal() const;

  // breadth-first-search algorithm
  std::vector<EdgeType> ShortestPath(const size_t& src,
                                     const size_t& dst) const;

  // Graph-cut algorithm
  std::unordered_map<int, int> NormalizedCut(const size_t cluster_num) const;

  // graph information presentation
  void ShowInfo() const;
  void ShowInfo(const std::string& filename) const;
  // void GraphVisual() const;

  Graph<NodeType, EdgeType> ExtractLargestCC() const;

  size_t FindConnectedComponents() const;

  std::vector<NodeType> SerializeNodes() const;
  std::vector<EdgeMap> SerializeEdges() const;

  // Update index of node
  void UpdateGraph();

 private:
  size_t _size;
  std::unordered_map<size_t, NodeType> nodes_;
  std::unordered_map<size_t, EdgeMap> edges_;  // size_t: src
  // degree of nodes: node_id, degree
  std::unordered_map<size_t, size_t> degrees_;
  std::unordered_map<size_t, size_t> out_degrees_;
  std::unordered_map<size_t, size_t> in_degrees_;
};

}  // namespace graph
}  // namespace DAGSfM

#include "graph.inl"

#endif