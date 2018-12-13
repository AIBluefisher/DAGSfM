/*
Copyright (c) 2018, Yu Chen
All rights reserved.

*/

/** \file image_graph.h
 *	\brief a simple image graph
 */
#ifndef CLUSTER_GRAPH_H
#define CLUSTER_GRAPH_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <cassert>
#include <limits>
#include <unordered_map>
#include <queue>

#include "Eigen/Core"
#include "i23dSFM/sfm/sfm.hpp"


using namespace Eigen;
using namespace i23dSFM;
using namespace i23dSFM::sfm;

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
		Mat3 R;
		Vector3d t;
		double scale;
        // unordered_map<geometry::Pose3, vector<Vector3d>> src_observations;
        // unordered_map<geometry::Pose3, vector<Vector3d>> dst_observations;
		vector<pair<int, geometry::Pose3>> src_poses;
		vector<pair<int, geometry::Pose3>> dst_poses;
		vector<Vector3d> src_observations;
		vector<Vector3d> dst_observations;
		LinkEdge(int src_ = -1, int dst_ = -1, float score_ = 0.0):
			src(src_), dst(dst_), score(score_) {}

		//! Copy constructor
		LinkEdge(const LinkEdge &e)
		{
			src = e.src;
			dst = e.dst;
			score = e.score;
			src_poses = e.src_poses;
			dst_poses = e.dst_poses;
            src_observations  = e.src_observations;
            dst_observations = e.dst_observations;
		}

		bool operator == (const LinkEdge& a)
		{
			return a.src == src && a.dst == dst;
		}

		friend bool operator < (LinkEdge a, LinkEdge b)
		{
			return a.score > b.score;
		}

		vector<Vector3d> GetSrcCameraCenter()
		{	cout << "src_poses size: " << src_poses.size() << endl;
			vector<Vector3d> centers;
			for(auto p : src_poses) {
				centers.push_back(p.second.center());
			}
			return centers;
		}

		vector<Vector3d> GetDstCameraCenter()
		{	cout << "dst_poses size: " << dst_poses.size() << endl;
			vector<Vector3d> centers;
			for(auto p : dst_poses) {
				centers.push_back(p.second.center());
			}
			return centers;
		}
	};

	/**
	 * @brief the image node used in image graph class
	 */
	struct ImageNode
	{
		int idx;                                //!< the optional original index (maybe in the image_list)
		std::string sfm_data_path;     //!< the image name
		ImageNode(int idx_ = -1, const std::string &path = ""): idx(idx_), sfm_data_path(path) {}

		//! Copy constructor
		ImageNode(const ImageNode & node)
		{
			idx = node.idx;
			sfm_data_path = node.sfm_data_path;
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
	class ClusterGraph
	{
	public:
		ClusterGraph();
		//! Brief construct ananymous image graph without filenames
		ClusterGraph(int size);
		//! Brief construct a image graph with filenames
		ClusterGraph(const std::vector<std::string>& sfm_data_filenames);

		// node operation
		void AddNode();
		void AddNode(const i23dSFM::ImageNode &n);
		bool DeleteNode(ImageNode& node);
		bool DeleteNode(size_t index);
		bool UpdateNode(size_t index, string sfm_data_path);
		ImageNode GetNode(int idx) const;
		std::vector<ImageNode> GetImageNode() const;
		int GetNodeSize() const;
		int NodeNum();
		size_t FindConnectedNode(size_t src, size_t dst);

		// edge operation
		//! Brief add one-way edge
		void AddEdge(int src, int dst, double score = 0.0);
		void AddEdge(const i23dSFM::LinkEdge &n);
		//! Brief add undirected edge
		void AddEdgeu(int src, int dst, double score = 0.0);
		void AddEdgeu(const i23dSFM::LinkEdge &n);
		bool RemoveEdge(size_t src, size_t dst);
		int GetEdgeSize() const;
		int AdjListSize(int idx);
		std::vector<EdgeMap> GetEdgeMap() const;
		std::vector<EdgeMap>& GetEdges();

		//! Brief compute the number of connected components in a undirected graph (edge (i,j) and edge (j,i) are both in the graph)
		int NumConnectedComponents(int threshold = 0);
		//! Brief output the undirected visualization code for graphviz
		bool Graphvizu(std::string gv_filename, std::string graph_name);
		//! Brief output the information
		void ShowInfo();

		std::vector<size_t> ShortestPath(size_t src, size_t dst) const;
		int Map2CurrentIdx(int idx);
		vector<LinkEdge> Prim() const;
		vector<LinkEdge> Kruskal() const;

		// MST operation
		unordered_map<int, int> ComputeDegree() const;
		int FindLeafNode(const unordered_map<int, int>& degree) const;
		LinkEdge FindConnectedEdge(const int idx) const;

	private:
		int size_;										//!< the total number of nodes in the graph
		std::vector<ImageNode> nodes_;                  //!< this stores the nodes information
		std::vector<EdgeMap> adj_maps_;					//!< find the edge index by adj_maps_[src][dst]
	};
}	// end of namespace i23dSFM

#endif	// i23dSFM_ClusterGraph_H
