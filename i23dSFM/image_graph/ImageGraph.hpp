/*
Copyright (c) 2018, Yu Chen
All rights reserved.

*/

/** \file image_graph.h
 *	\brief a simple image graph
 */
#ifndef IMAGEGRAPH_H
#define IMAGEGRAPH_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <cassert>
#include <unordered_map>

namespace i23dSFM
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

		friend bool operator < (LinkEdge a, LinkEdge b)
		{
			return a.score > b.score;
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

	struct PathNode
	{
		int idx;
		PathNode* parent;
		PathNode(int idx_, PathNode* parent_)
		{
			idx = idx_;
			parent = parent_;
		}
	};

	typedef std::unordered_map<int, LinkEdge> EdgeMap;
	typedef std::vector<std::vector<LinkEdge> > Edge2dArray;

	/**
	 * @brief Image graph class
	 */
	class ImageGraph
	{
	public:
		ImageGraph();
		//! Brief construct ananymous image graph without filenames
		ImageGraph(int size);
		//! Brief construct a image graph with filenames
		ImageGraph(const std::vector<std::string> &image_filenames, const std::vector<std::string> &sift_filenames);
		void AddNode();
		void AddNode(const i23dSFM::ImageNode &n);
		//! Brief add one-way edge
		void AddEdge(int src, int dst, double score = 0.0);
		void AddEdge(const i23dSFM::LinkEdge &n);
		//! Brief add undirected edge
		void AddEdgeu(int src, int dst, double score = 0.0);
		void AddEdgeu(const i23dSFM::LinkEdge &n);
		//! Brief compute the number of connected components in a undirected graph (edge (i,j) and edge (j,i) are both in the graph)
		int NumConnectedComponents(int threshold = 0);
		bool KargerCut(std::vector<std::vector<int> > &global_min_cut);
		//! Brief Remove the singleton node from the graph
		bool Consolidate(int k);
		//! Brief Query expansion and its sub-routine
		bool QueryExpansion(Edge2dArray &expansion_lists, int level, int inlier_threshold = 150);
		bool QueryExpansionSub(int src, int tgt, double score, Edge2dArray &expansion_lists, bool **visit_mat, int level, int inlier_threshold);
		//! Brief output the undirected visualization code for graphviz
		bool Graphvizu(std::string gv_filename, std::string graph_name);
		//! Brief output the information
		void ShowInfo();
		void ShowInfo(std::string filename);
		int AdjListSize(int idx);
		int NodeNum();
		int GetEdgeSize() const;
		int GetNodeSize() const;
		ImageNode GetNode(int idx) const;
		std::vector<ImageNode> GetImageNode() const;
		std::vector<EdgeMap> GetEdgeMap() const;
		std::vector<size_t> ShortestPath(size_t src, size_t dst) const;
		int Map2CurrentIdx(int idx);

	private:
		int size_;										//!< the total number of nodes in the graph
		std::vector<ImageNode> nodes_;                  //!< this stores the nodes information
		std::vector<EdgeMap> adj_maps_;					//!< find the edge index by adj_maps_[src][dst]
	};
}	// end of namespace i23dSFM

#endif	// i23dSFM_IMAGEGRAPH_H
