/*
Copyright (c) 2015, Tianwei Shen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of libvot nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/** \file image_graph.h
 *	\brief a simple image graph
 */
#ifndef VOT_IMAGEGRAPH_H
#define VOT_IMAGEGRAPH_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <cassert>
#include <unordered_map>

namespace vot
{
/**
 * @brief edge struct used in image graph class
 */
struct LinkEdge
{
	size_t src;
	size_t dst;
	float score;
	int p_match;
	int g_match;

	LinkEdge(int src_ = -1, int dst_ = -1, float score_ = 0.0, int p_match_ = 0, int g_match_ = 0):
	    src(src_), dst(dst_), score(score_), p_match(p_match_), g_match(g_match_) {}

	//! Copy constructor
	LinkEdge(const LinkEdge &e)
	{
		src = e.src;
		dst = e.dst;
		score = e.score;
		p_match = e.p_match;
		g_match = e.g_match;
	}
};

/**
 * @brief the image node used in image graph class
 */
struct ImageNode
{
	int idx;                    //!< the optional original index (maybe in the image_list)
	std::string image_name;     //!< the image name
	std::string sift_name;      //!< the sift name
	ImageNode(int idx_ = -1, const std::string &iname = "", const std::string &sname = ""): idx(idx_), image_name(iname), sift_name(sname) {}

	//! Copy constructor
	ImageNode(const ImageNode & node)
	{
		idx = node.idx;
		image_name = node.image_name;
		sift_name = node.sift_name;
	}
};

/**
 * @brief Image graph class
 */
class ImageGraph
{
	typedef std::unordered_map<int, LinkEdge> EdgeMap;
	typedef std::vector<std::vector<LinkEdge> > Edge2dArray;
public:
	//! Brief construct ananymous image graph without filenames
	ImageGraph(int size);
	//! Brief construct a image graph with filenames
	ImageGraph(const std::vector<std::string> &image_filenames, const std::vector<std::string> &sift_filenames);
	void addNode();
	void addNode(const vot::ImageNode &n);
	//! Brief add one-way edge
	void addEdge(int src, int dst, double score = 0.0);
	void addEdge(const vot::LinkEdge &n);
	//! Brief add undirected edge
	void addEdgeu(int src, int dst, double score = 0.0);
	void addEdgeu(const vot::LinkEdge &n);
	//! Brief compute the number of connected components in a undirected graph (edge (i,j) and edge (j,i) are both in the graph)
	int numConnectedComponents(int threshold = 0);
	bool kargerCut(std::vector<std::vector<int> > &global_min_cut);
	//! Brief Remove the singleton node from the graph
	bool consolidate(int k);
	//! Brief Query expansion and its sub-routine
	bool queryExpansion(Edge2dArray &expansion_lists, int level, int inlier_threshold = 150);
	bool queryExpansionSub(int src, int tgt, double score, Edge2dArray &expansion_lists, bool **visit_mat, int level, int inlier_threshold);
	//! Brief output the undirected visualization code for graphviz
	bool graphvizu(std::string gv_filename, std::string graph_name);
	//! Brief output the information
	void showInfo();
	int adjListSize(int idx);
	int nodeNum();

private:
	int size_;										//!< the total number of nodes in the graph
	std::vector<ImageNode> nodes_;                  //!< this stores the nodes information
	std::vector<EdgeMap> adj_maps_;					//!< find the edge index by adj_maps_[src][dst]
};
}	// end of namespace vot

#endif	// VOT_IMAGEGRAPH_H
