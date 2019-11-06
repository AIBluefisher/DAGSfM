#include "gtest/gtest.h"

#include "graph_cut.h"

using namespace GraphSfM;
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
