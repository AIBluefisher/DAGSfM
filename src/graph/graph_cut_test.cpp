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

#include "graph_cut.h"

#include "gtest/gtest.h"

using namespace DAGSfM;
using namespace graph;

TEST(GRAPH_CUT_TEST, TestComputeMinGraphCutStoerWagner) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 4}, {0, 1}, {0, 6}, {0, 7}, {0, 5},
      {0, 2}, {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}, {3, 4}};
  const std::vector<int> weights = {0, 3, 1, 3,  1, 2, 6, 1,
                                    8, 1, 1, 80, 2, 1, 1, 4};
  int cut_weight;
  std::vector<char> cut_labels;
  ComputeMinGraphCutStoerWagner(edges, weights, &cut_weight, &cut_labels);
  EXPECT_EQ(cut_weight, 7);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label, 0);
    EXPECT_LT(label, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeMinGraphCutStoerWagnerDuplicateEdge) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 4}, {0, 1}, {0, 6}, {0, 7}, {0, 5}, {0, 2},
      {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}, {3, 4}, {3, 4}};
  const std::vector<int> weights = {0, 3, 1,  3, 1, 2, 6, 1, 8,
                                    1, 1, 80, 2, 1, 1, 4, 4};
  int cut_weight;
  std::vector<char> cut_labels;
  ComputeMinGraphCutStoerWagner(edges, weights, &cut_weight, &cut_labels);
  EXPECT_EQ(cut_weight, 7);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label, 0);
    EXPECT_LT(label, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeMinGraphCutStoerWagnerMissingVertex) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 1}, {0, 6}, {0, 7}, {0, 5},
      {0, 2}, {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}};
  const std::vector<int> weights = {0, 3, 1, 3, 1, 2, 6, 1, 8, 1, 1, 80, 2, 1};
  int cut_weight;
  std::vector<char> cut_labels;
  ComputeMinGraphCutStoerWagner(edges, weights, &cut_weight, &cut_labels);
  EXPECT_EQ(cut_weight, 2);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label, 0);
    EXPECT_LT(label, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeMinGraphCutStoerWagnerDisconnected) {
  const std::vector<std::pair<int, int>> edges = {{0, 1}, {1, 2}, {3, 4}};
  const std::vector<int> weights = {1, 3, 1};
  int cut_weight;
  std::vector<char> cut_labels;
  ComputeMinGraphCutStoerWagner(edges, weights, &cut_weight, &cut_labels);
  EXPECT_EQ(cut_weight, 0);
  EXPECT_EQ(cut_labels.size(), 5);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label, 0);
    EXPECT_LT(label, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeNormalizedMinGraphCut) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 4}, {0, 1}, {0, 6}, {0, 7}, {0, 5},
      {0, 2}, {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}, {3, 4}};
  const std::vector<int> weights = {0, 3, 1, 3,  1, 2, 6, 1,
                                    8, 1, 1, 80, 2, 1, 1, 4};
  const auto cut_labels = ComputeNormalizedMinGraphCut(edges, weights, 2);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label.second, 0);
    EXPECT_LT(label.second, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeNormalizedMinGraphCutDuplicateEdge) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 4}, {0, 1}, {0, 6}, {0, 7}, {0, 5}, {0, 2},
      {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}, {3, 4}, {3, 4}};
  const std::vector<int> weights = {0, 3, 1,  3, 1, 2, 6, 1, 8,
                                    1, 1, 80, 2, 1, 1, 4, 4};
  const auto cut_labels = ComputeNormalizedMinGraphCut(edges, weights, 2);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label.second, 0);
    EXPECT_LT(label.second, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeNormalizedMinGraphCutMissingVertex) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 1}, {0, 6}, {0, 7}, {0, 5},
      {0, 2}, {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}};
  const std::vector<int> weights = {0, 3, 1, 3, 1, 2, 6, 1, 8, 1, 1, 80, 2, 1};
  const auto cut_labels = ComputeNormalizedMinGraphCut(edges, weights, 2);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label.second, 0);
    EXPECT_LT(label.second, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeNormalizedMinGraphCutDisconnected) {
  const std::vector<std::pair<int, int>> edges = {{0, 1}, {1, 2}, {3, 4}};
  const std::vector<int> weights = {1, 3, 1};
  const auto cut_labels = ComputeNormalizedMinGraphCut(edges, weights, 2);
  EXPECT_EQ(cut_labels.size(), 5);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label.second, 0);
    EXPECT_LT(label.second, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestMinSTGraphCut1) {
  MinSTGraphCut<int, int> graph(2);
  EXPECT_EQ(graph.NumNodes(), 2);
  EXPECT_EQ(graph.NumEdges(), 0);
  graph.AddNode(0, 5, 1);
  graph.AddNode(1, 2, 6);
  graph.AddEdge(0, 1, 3, 4);
  EXPECT_EQ(graph.NumEdges(), 10);
  EXPECT_EQ(graph.Compute(), 6);
  EXPECT_EQ(graph.IsConnectedToSource(0), true);
  EXPECT_EQ(graph.IsConnectedToSink(1), true);
}

TEST(GRAPH_CUT_TEST, TestMinSTGraphCut2) {
  MinSTGraphCut<int, int> graph(2);
  graph.AddNode(0, 1, 5);
  graph.AddNode(1, 2, 6);
  graph.AddEdge(0, 1, 3, 4);
  EXPECT_EQ(graph.NumEdges(), 10);
  EXPECT_EQ(graph.Compute(), 3);
  EXPECT_EQ(graph.IsConnectedToSink(0), true);
  EXPECT_EQ(graph.IsConnectedToSink(1), true);
}

TEST(GRAPH_CUT_TEST, TestMinSTGraphCut3) {
  MinSTGraphCut<int, int> graph(3);
  graph.AddNode(0, 6, 4);
  graph.AddNode(2, 3, 6);
  graph.AddEdge(0, 1, 2, 4);
  graph.AddEdge(1, 2, 3, 5);
  EXPECT_EQ(graph.NumEdges(), 12);
  EXPECT_EQ(graph.Compute(), 9);
  EXPECT_EQ(graph.IsConnectedToSource(0), true);
  EXPECT_EQ(graph.IsConnectedToSink(1), true);
  EXPECT_EQ(graph.IsConnectedToSink(2), true);
}
