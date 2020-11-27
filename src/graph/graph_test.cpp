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

#include "graph.h"

#include <tuple>
#include <vector>

#include "gtest/gtest.h"

using namespace std;
using namespace DAGSfM;
using namespace DAGSfM::graph;

const vector<pair<size_t, size_t>> directed_edges = {
    {0, 1}, {0, 2}, {0, 3}, {0, 7}, {1, 0}, {1, 3}, {1, 6}, {1, 7},
    {2, 0}, {2, 3}, {2, 4}, {3, 0}, {3, 1}, {3, 2}, {3, 4}, {3, 5},
    {4, 2}, {4, 3}, {4, 5}, {5, 3}, {5, 4}, {5, 6}, {5, 7}, {6, 1},
    {6, 5}, {6, 7}, {7, 0}, {7, 1}, {7, 5}, {7, 6}};

const vector<pair<size_t, size_t>> undirected_edges = {
    {0, 1}, {0, 2}, {0, 3}, {0, 7}, {1, 3}, {1, 6}, {1, 7}, {2, 3},
    {2, 4}, {3, 4}, {3, 5}, {4, 5}, {5, 6}, {5, 7}, {6, 7}, {7, 8}};

const vector<vector<float>> weighted_edges = {
    {0, 1, 6}, {0, 2, 1}, {0, 3, 5}, {1, 2, 5}, {1, 4, 3},
    {2, 3, 5}, {2, 4, 6}, {2, 5, 4}, {3, 5, 2}, {4, 5, 6}};

TEST(GRAPH_TEST, TEST_ADDNODE) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  EXPECT_EQ(graph.GetNodes().size(), n);
  EXPECT_EQ(graph.GetSize(), n);
}

TEST(GRAPH_TEST, TEST_HASNODE) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (int i = 0; i < n; i++) {
    EXPECT_EQ(graph.HasNode(i), true);
  }

  for (int i = n; i < 2 * n; i++) {
    EXPECT_NE(graph.HasNode(i), true);
  }
}

TEST(GRAPH_TEST, TEST_DELETENODE) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  graph.DeleteNode(-1);
  graph.DeleteNode(1);
  graph.DeleteNode(6);
  graph.DeleteNode(8);

  EXPECT_EQ(graph.HasNode(1), false);
  EXPECT_EQ(graph.HasNode(6), false);
  EXPECT_EQ(graph.HasNode(-1), false);
  EXPECT_EQ(graph.HasNode(8), false);
}

TEST(GRAPH_TEST, TEST_CLONE) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : directed_edges) {
    Edge edge(e.first, e.second);
    graph.AddEdge(edge);
  }

  Graph<Node, Edge> cloned_graph = graph.Clone();
  EXPECT_EQ(graph.GetNodesNum(), cloned_graph.GetNodesNum());
  EXPECT_EQ(graph.GetEdgesNum(), cloned_graph.GetEdgesNum());
  cloned_graph.ShowInfo();
}

TEST(DIRECTED_GRAPH_TEST, TEST_ADDEDGE) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : directed_edges) {
    Edge edge(e.first, e.second);
    graph.AddEdge(edge);
  }

  ASSERT_EQ(graph.GetEdgesNum(), directed_edges.size());

  for (auto e : directed_edges) {
    EXPECT_EQ(graph.HasEdge(e.first, e.second), true);
  }
}

TEST(DIRECTED_GRAPH_TEST, TEST_DELETEEDGE) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : directed_edges) {
    Edge edge(e.first, e.second);
    graph.AddEdge(edge);
  }

  const vector<pair<size_t, size_t>> deleted_edges = {
      {0, 1}, {0, 3}, {3, 4}, {3, 7}, {7, 1}, {7, 4}, {8, 9}, {8, 10}};

  for (auto de : deleted_edges) {
    graph.DeleteEdge(de.first, de.second);
    EXPECT_EQ(graph.HasEdge(de.first, de.second), false);
  }
}

TEST(DIRECTED_GRAPH_TEST, TEST_COUNTDEGREES) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : directed_edges) {
    Edge edge(e.first, e.second);
    graph.AddEdge(edge);
  }

  unordered_map<size_t, size_t> degrees;

  // for directed graph, degree = out-degree + in-degree
  graph.CountInDegrees();
  graph.CountOutDegrees();
  graph.CountDegrees();
  degrees = graph.GetDegrees();

  EXPECT_EQ(degrees[0], 8);
  EXPECT_EQ(degrees[1], 8);
  EXPECT_EQ(degrees[2], 6);
  EXPECT_EQ(degrees[3], 10);
  EXPECT_EQ(degrees[4], 6);
  EXPECT_EQ(degrees[5], 8);
  EXPECT_EQ(degrees[6], 6);
  EXPECT_EQ(degrees[7], 8);

  const vector<pair<size_t, size_t>> deleted_edges = {
      {0, 1}, {0, 3}, {3, 4}, {3, 7}, {7, 1}, {7, 4}, {8, 9}, {8, 10}};

  for (auto de : deleted_edges) {
    graph.DeleteEdge(de.first, de.second);
    graph.DeleteEdge(de.second, de.first);
    ASSERT_EQ(graph.HasEdge(de.first, de.second), false);
    ASSERT_EQ(graph.HasEdge(de.second, de.first), false);
  }

  graph.CountInDegrees();
  graph.CountOutDegrees();
  graph.CountDegrees();
  degrees = graph.GetDegrees();

  EXPECT_EQ(degrees[0], 4);
  EXPECT_EQ(degrees[1], 4);
  EXPECT_EQ(degrees[2], 6);
  EXPECT_EQ(degrees[3], 6);
  EXPECT_EQ(degrees[4], 4);
  EXPECT_EQ(degrees[5], 8);
  EXPECT_EQ(degrees[6], 6);
  EXPECT_EQ(degrees[7], 6);
}

TEST(GRAPH_TEST, TEST_SERIALIZENODES) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  std::vector<Node> sequential_nodes = graph.SerializeNodes();
  for (uint i = 0; i < sequential_nodes.size(); i++) {
    EXPECT_EQ(sequential_nodes[i].id, i);
  }
}

TEST(GRAPH_TEST, TEST_SERIALIZEEDGES) {
  // TODO:
}

TEST(UNDIRECTED_GRAPH_TEST, TEST_ADDEDGE) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : undirected_edges) {
    graph.AddEdge(Edge(e.first, e.second));
    graph.AddEdge(Edge(e.second, e.first));
  }

  ASSERT_EQ(graph.GetEdgesNum(), 2 * undirected_edges.size());

  for (auto e : directed_edges) {
    EXPECT_EQ(graph.HasEdge(e.first, e.second), true);
    EXPECT_EQ(graph.HasEdge(e.second, e.first), true);
  }
}

TEST(UNDIRECTED_GRAPH_TEST, TEST_DELETEEDGE) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : undirected_edges) {
    graph.AddEdge(Edge(e.first, e.second));
    graph.AddEdge(Edge(e.second, e.first));
  }

  const vector<pair<size_t, size_t>> deleted_edges = {
      {0, 1}, {0, 3}, {3, 4}, {3, 7}, {7, 1}, {7, 4}, {8, 9}, {8, 10}};

  for (auto de : deleted_edges) {
    graph.DeleteEdge(de.first, de.second);
    graph.DeleteEdge(de.second, de.first);
    EXPECT_EQ(graph.HasEdge(de.first, de.second), false);
    EXPECT_EQ(graph.HasEdge(de.second, de.first), false);
  }
}

TEST(UNDIRECTED_GRAPH_TEST, TEST_COUNTDEGREES) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : undirected_edges) {
    graph.AddEdge(Edge(e.first, e.second));
    graph.AddEdge(Edge(e.second, e.first));
  }

  unordered_map<size_t, size_t> degrees;

  // for undirected graph, the degree of node equals to the out-degree of node
  graph.CountOutDegrees();
  graph.CountInDegrees();
  graph.CountDegrees();
  degrees = graph.GetDegrees();

  EXPECT_EQ(degrees[0], 8);
  EXPECT_EQ(degrees[1], 8);
  EXPECT_EQ(degrees[2], 6);
  EXPECT_EQ(degrees[3], 10);
  EXPECT_EQ(degrees[4], 6);
  EXPECT_EQ(degrees[5], 8);
  EXPECT_EQ(degrees[6], 6);
  EXPECT_EQ(degrees[7], 10);
  EXPECT_EQ(degrees[8], 2);

  const vector<pair<size_t, size_t>> deleted_edges = {
      {0, 1}, {0, 3}, {3, 4}, {3, 7}, {7, 1}, {7, 4}, {8, 9}, {8, 10}};

  for (auto de : deleted_edges) {
    graph.DeleteEdge(de.first, de.second);
    graph.DeleteEdge(de.second, de.first);
    ASSERT_EQ(graph.HasEdge(de.first, de.second), false);
    ASSERT_EQ(graph.HasEdge(de.second, de.first), false);
  }

  graph.CountOutDegrees();
  graph.CountInDegrees();
  graph.CountDegrees();
  degrees = graph.GetDegrees();

  EXPECT_EQ(degrees[0], 4);
  EXPECT_EQ(degrees[1], 4);
  EXPECT_EQ(degrees[2], 6);
  EXPECT_EQ(degrees[3], 6);
  EXPECT_EQ(degrees[4], 4);
  EXPECT_EQ(degrees[5], 8);
  EXPECT_EQ(degrees[6], 6);
  EXPECT_EQ(degrees[7], 8);
  EXPECT_EQ(degrees[8], 2);
}

TEST(UNDIRECTED_GRAPH_TEST, TEST_KRUSKAL) {
  Graph<Node, Edge> graph;
  const int n = 6;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : weighted_edges) {
    graph.AddEdge(Edge((size_t)e[0], (size_t)e[1], e[2]));
    // graph.AddEdge(Edge((size_t)e[1], (size_t)e[0], e[2]));
  }

  vector<Edge> mst_edges = graph.Kruskal();
  vector<pair<size_t, size_t>> gt_edges = {
      {0, 2}, {1, 2}, {1, 4}, {2, 5}, {3, 5}};

  ASSERT_EQ(mst_edges.size(), graph.GetNodesNum() - 1);
  for (auto gt : gt_edges) {
    bool find = false;
    for (auto mst_edge : mst_edges) {
      if (((mst_edge.src == gt.first) && (mst_edge.dst == gt.second)) ||
          ((mst_edge.src == gt.second) && (mst_edge.dst == gt.first))) {
        find = true;
      }
    }
    EXPECT_EQ(find, true);
  }
}

TEST(UNDIRECTED_GRAPH_TEST, TEST_SHORTESTPATH) {
  Graph<Node, Edge> graph;
  const int n = 6;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : weighted_edges) {
    graph.AddEdge(Edge((size_t)e[0], (size_t)e[1], e[2]));
    graph.AddEdge(Edge((size_t)e[1], (size_t)e[0], e[2]));
  }

  vector<Edge> mst_edges = graph.Kruskal();

  Graph<Node, Edge> mst;
  for (auto mst_edge : mst_edges) {
    Edge rev_edge(mst_edge.dst, mst_edge.src, mst_edge.weight);
    mst.AddUEdge(mst_edge, rev_edge);
  }
  mst.ShowInfo();

  vector<Edge> path0 = mst.ShortestPath(0, 1);
  vector<Edge> path1 = mst.ShortestPath(0, 4);
  vector<Edge> path2 = mst.ShortestPath(4, 3);

  ASSERT_EQ(path0.size(), 2);
  ASSERT_EQ(path1.size(), 3);
  ASSERT_EQ(path2.size(), 4);
}

TEST(UNDIRECTED_GRAPH_TEST, TEST_FINDCONNECTEDCOMPONENTS) {
  Graph<Node, Edge> graph;
  const int n = 8;
  for (int i = 0; i < n; i++) {
    Node node(i);
    graph.AddNode(node);
  }

  for (auto e : undirected_edges) {
    graph.AddEdge(Edge(e.first, e.second));
    graph.AddEdge(Edge(e.second, e.first));
  }

  size_t cc_num = graph.FindConnectedComponents().size();
  EXPECT_EQ(cc_num, 1);

  graph.AddUEdge(Edge(1, 8), Edge(8, 1));
  graph.AddUEdge(Edge(7, 8), Edge(8, 7));
  graph.AddUEdge(Edge(8, 9), Edge(9, 8));
  graph.AddUEdge(Edge(8, 10), Edge(10, 8));
  graph.AddUEdge(Edge(8, 11), Edge(11, 8));
  graph.AddUEdge(Edge(9, 10), Edge(10, 9));
  graph.AddUEdge(Edge(9, 11), Edge(11, 9));
  graph.AddUEdge(Edge(10, 11), Edge(11, 10));

  cc_num = graph.FindConnectedComponents().size();
  EXPECT_EQ(cc_num, 1);

  graph.DeleteEdge(1, 8);
  graph.DeleteEdge(8, 1);
  graph.DeleteEdge(7, 8);
  graph.DeleteEdge(8, 7);

  cc_num = graph.FindConnectedComponents().size();
  EXPECT_EQ(cc_num, 2);
}
