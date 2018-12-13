#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include "vocab_tree/vot_pipeline.h"
#include "gtest/gtest.h"
#include "graph/image_graph.h"
#include "utils/data_types.h"

// -------------------------------------------------------- //
// 															//
//					test image_graph.h 						//
// 															//
// -------------------------------------------------------- //
TEST(ImageGraph, ImageNode)
{
	vot::ImageNode a(0, "123.jpg", "123.sift");	// constructor use image_name and sift_name
	vot::ImageNode b(a);		// copy constructor
	EXPECT_EQ("123.jpg", b.image_name);
	EXPECT_EQ("123.sift", b.sift_name);
	vot::ImageNode c;
	vot::ImageGraph image_graph(0);
	image_graph.addNode(a);
	image_graph.addNode(b);
	image_graph.addNode(c);
	image_graph.addNode();
	EXPECT_EQ(4, image_graph.nodeNum());
	EXPECT_EQ(4, image_graph.numConnectedComponents());
}

TEST(ImageGraph, numConnectedComponents)
{
	// case #1
	vot::ImageGraph g(6);
	g.addEdge(0, 1, 0);
	g.addEdge(1, 2, 0);
	g.addEdge(2, 3, 0);
	g.addEdge(3, 4, 0);
	g.addEdge(4, 5, 0);
	EXPECT_EQ(1, g.numConnectedComponents());		

	// case #2
	vot::ImageGraph q(6);
	q.addEdge(0, 1, 0);
	q.addEdge(1, 2, 0);
	q.addEdge(2, 3, 0);
	q.addEdge(4, 5, 0);
	EXPECT_EQ(2, q.numConnectedComponents());		

	// case #3: {{0,1,2,3,4}, {5}, {6,7}, {8,9}}
	vot::ImageGraph double_link(10);
	double_link.addEdge(0, 1, 3);
	double_link.addEdge(1, 0, 3);
	EXPECT_EQ(1, double_link.adjListSize(1));
	EXPECT_EQ(1, double_link.adjListSize(0));
	double_link.addEdge(2, 3, 3);
	double_link.addEdge(3, 4, 3);
	double_link.addEdge(8, 9, 3);
	double_link.addEdge(9, 8, 3);
	double_link.addEdge(9, 8, 120);		// this add won't do anything to the graph
	double_link.addEdge(1, 6, 100);
	double_link.addEdge(6, 7, 3);
	double_link.addEdge(7, 6, 3);
	EXPECT_EQ(4, double_link.numConnectedComponents());
	// the number of CC of size greater or equal than 2
	EXPECT_EQ(3, double_link.numConnectedComponents(2));

	// test size function
	EXPECT_EQ(10, double_link.nodeNum());
	EXPECT_EQ(6, q.nodeNum());
	EXPECT_EQ(6, g.nodeNum());

	// case #4: undirected graph
	vot::ImageGraph mc(10);
	EXPECT_EQ(10, mc.nodeNum());
	EXPECT_EQ(10, mc.numConnectedComponents());
	EXPECT_EQ(0, mc.numConnectedComponents(2));
	mc.addEdgeu(0, 5);
	mc.addEdgeu(5, 0);
	mc.addEdgeu(5, 6);
	mc.addEdgeu(6, 7);
	mc.addEdgeu(7, 8);
	mc.addEdgeu(1, 2);
	mc.addEdgeu(2, 3);
	EXPECT_EQ(4, mc.numConnectedComponents());
	mc.addEdgeu(2, 4);
	EXPECT_EQ(3, mc.numConnectedComponents());
	mc.addEdgeu(9, 3);
	mc.addEdgeu(3, 9);
	EXPECT_EQ(2, mc.numConnectedComponents());
}

TEST(ImageGraph, AddRepeatedEdge)
{
    vot::ImageGraph g(10);
    g.addEdge(0, 1, 0);
    g.addEdge(0, 1, 1);
    g.addEdge(0, 1, 100);
    // also test the other direcition; this means to protect double-add
    g.addEdge(1, 0, 0.0);
    g.addEdge(1, 0, 1);
    g.addEdge(1, 0, 200);
    EXPECT_EQ(1, g.adjListSize(0));
    EXPECT_EQ(1, g.adjListSize(1));
}

bool IsXInVector(int x, std::vector<int> &vec)
{
	for(int i = 0; i < vec.size(); i++)
	{
		if(x == vec[i])
			return true;
	}
	return false;
}

TEST(ImageGraph, KargerCutSimpleTest)
{
	// test Karger's Cut, assume the graph is a connected component, undirected
	// Case 1: {{0, 1, 2}, {3, 4, 5}}
	const int node_num = 6;
	vot::ImageGraph g(node_num);
	g.addEdgeu(0, 1, 1);
	g.addEdgeu(1, 2, 1);
	g.addEdgeu(0, 2, 1);
	g.addEdgeu(2, 3, 1);
	g.addEdgeu(3, 4, 1);
	g.addEdgeu(4, 5, 1);
	g.addEdgeu(3, 5, 1);
	std::vector<std::vector<int> > two_parts;
	g.kargerCut(two_parts);
	EXPECT_EQ(2, two_parts.size());
	EXPECT_EQ(3, two_parts[0].size());
	EXPECT_EQ(3, two_parts[1].size());
	bool flag = true;
	for(int i = 0; i < node_num; i++)
	{
		if(!IsXInVector(i, two_parts[0]) && !IsXInVector(i, two_parts[1]))
			flag = false;
	}
	if(!IsXInVector(0, two_parts[0]))	flag = false;
	if(!IsXInVector(1, two_parts[0]))	flag = false;
	if(!IsXInVector(2, two_parts[0]))	flag = false;
	if(!IsXInVector(3, two_parts[1]))	flag = false;
	if(!IsXInVector(4, two_parts[1]))	flag = false;
	if(!IsXInVector(5, two_parts[1]))	flag = false;
	EXPECT_EQ(true, flag);
}

TEST(ImageGraph, QueryExpansion)
{
	const int node_num = 3;
	vot::ImageGraph g(node_num);
	vot::LinkEdge e1(0, 1, 1, 1000, 1000);
	vot::LinkEdge e2(1, 2, 1, 1000, 1000);
	g.addEdgeu(e1);
	g.addEdgeu(e2);

	// query expansion
	std::vector<std::vector<vot::LinkEdge> > expansion_lists;
	g.queryExpansion(expansion_lists, 2, 100);
	EXPECT_EQ(1, expansion_lists[0].size());
	EXPECT_EQ(0, expansion_lists[1].size());
	EXPECT_EQ(1, expansion_lists[2].size());
	EXPECT_EQ(2, expansion_lists[0][0].dst);
	EXPECT_EQ(0, expansion_lists[2][0].dst);

	// add one node and redo the query expansion
	g.addNode();
	vot::LinkEdge e3(1, 3, 1, 1000, 200);
	g.addEdgeu(e3);
	g.queryExpansion(expansion_lists, 2, 100);
	EXPECT_EQ(2, expansion_lists[0].size());
	EXPECT_EQ(2, expansion_lists[2].size());
	EXPECT_EQ(2, expansion_lists[3].size());

	// add one node and test 'level'
	g.addNode();
	vot::LinkEdge e4(3, 4, 1, 1000, 100);	
	g.addEdgeu(e4);
	g.queryExpansion(expansion_lists, 2, 100);
	EXPECT_EQ(2, expansion_lists[0].size());
	EXPECT_EQ(1, expansion_lists[1].size());
	EXPECT_EQ(2, expansion_lists[2].size());
	EXPECT_EQ(2, expansion_lists[3].size());
	EXPECT_EQ(1, expansion_lists[4].size());
	g.queryExpansion(expansion_lists, 3, 100);
	EXPECT_EQ(3, expansion_lists[0].size());
	EXPECT_EQ(1, expansion_lists[1].size());
	EXPECT_EQ(3, expansion_lists[2].size());
	EXPECT_EQ(2, expansion_lists[3].size());
	EXPECT_EQ(3, expansion_lists[4].size());
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
