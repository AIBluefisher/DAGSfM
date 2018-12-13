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

/** \file image_graph.cpp
 *	\brief a simple image graph implementation
 */
#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <algorithm>
#include "image_graph.h"
#include "../utils/data_structures.h"

using std::cout;
using std::endl;

namespace vot
{
ImageGraph::ImageGraph(int size)
{
	size_ = size;
	adj_maps_.resize(size_);
	nodes_.resize(size_);
	for (int i = 0; i < size_; i++)
		nodes_[i] = ImageNode();
}

ImageGraph::ImageGraph(const std::vector<std::string> &image_filenames, const std::vector<std::string> &sift_filenames)
{
	assert(sift_filenames.size() == image_filenames.size());
	size_ = image_filenames.size();
	adj_maps_.resize(size_);
	nodes_.resize(size_);
	for (int i = 0; i < size_; i++)
		nodes_[i] = ImageNode(i, image_filenames[i], sift_filenames[i]);
}

void ImageGraph::addNode()
{
	nodes_.push_back(vot::ImageNode());
	adj_maps_.push_back(EdgeMap());
	size_ = nodes_.size();
}

void ImageGraph::addNode(const vot::ImageNode &n)
{
	nodes_.push_back(vot::ImageNode(n));
	adj_maps_.push_back(EdgeMap());
	size_ = nodes_.size();
}

void ImageGraph::addEdge(int src, int dst, double score)
{
	EdgeMap::iterator it = adj_maps_[src].find(dst);
	if (it == adj_maps_[src].end()) {
		adj_maps_[src].insert(std::make_pair(dst, vot::LinkEdge(src, dst, score)));
	}
}

void ImageGraph::addEdge(const vot::LinkEdge &n)
{
	int src = n.src, dst = n.dst;
	EdgeMap::iterator it = adj_maps_[src].find(dst);
	if (it == adj_maps_[src].end()) {
		adj_maps_[src].insert(std::make_pair(dst, n));
	}
}

void ImageGraph::addEdgeu(int src, int dst, double score)
{
	addEdge(src, dst, score);
	addEdge(dst, src, score);
}

void ImageGraph::addEdgeu(const vot::LinkEdge &n)
{
	addEdge(n);
	vot::LinkEdge n_inv(n.dst, n.src, n.score, n.p_match, n.g_match);
	addEdge(n_inv);
}

int ImageGraph::numConnectedComponents(int threshold)
{
	std::vector<bool> is_visited(size_, false);

	size_t numCC = 0;
	for (int i = 0; i < size_; i++) {
		if (!is_visited[i]) {
			numCC++;
			std::queue<int> index_queue;
			is_visited[i] = true;
			index_queue.push(i);
			int component_size = 1;
			while (!index_queue.empty()) {
				int curr = index_queue.front();
				index_queue.pop();
				for (EdgeMap::iterator it = adj_maps_[curr].begin(); it != adj_maps_[curr].end(); it++) {
					if (!is_visited[it->second.dst]) {
						is_visited[it->second.dst] = true;
						component_size++;
						index_queue.push(it->second.dst);
					}
				}
			}
			if (component_size < threshold && threshold != 0)
				numCC--;
		}
	}
	return numCC;
}

bool ImageGraph::kargerCut(std::vector<std::vector<int> > &global_min_cut)
{
	std::vector<int> global_min_cut1, global_min_cut2;
	// randomly select an edge
	std::vector<std::pair<int, int> > edges;
	for (int i = 0; i < size_; i++)
		for (EdgeMap::iterator it = adj_maps_[i].begin(); it != adj_maps_[i].end(); it++)
			edges.push_back(std::pair<int, int>(it->second.src, it->second.dst));

	int iter_num = size_ * log(size_);
	int min_cut = edges.size();
	for (int iter = 0; iter < iter_num; iter++) {
		std::random_shuffle(edges.begin(), edges.end());
		tw::UnionFind vertex_union(size_);
		unsigned int edge_iter = 0;
		int operation_count = 0;
		while (edge_iter < edges.size() && operation_count < size_-2) {
			int src = edges[edge_iter].first;
			int dst = edges[edge_iter].second;
			if (vertex_union.UnionSet(src, dst)) //contention operation
				operation_count++;
			edge_iter++;
		}
		std::vector<int> temp_parts[2];
		for (int i = 0; i < size_; i++) {
			if(vertex_union.Find(i) == vertex_union.Find(0))
				temp_parts[0].push_back(i);
			else
				temp_parts[1].push_back(i);
		}
		int temp_cut = 0;
		for (int i = 0; i < size_; i++) {
			for (EdgeMap::iterator it = adj_maps_[i].begin(); it != adj_maps_[i].end(); it++) {
				if (vertex_union.Find(it->second.src) != vertex_union.Find(it->second.dst))
					temp_cut++;
			}
		}
		if (min_cut > temp_cut) {
			min_cut = temp_cut;
			global_min_cut1 = temp_parts[0];
			global_min_cut2 = temp_parts[1];
		}
	}
	global_min_cut.clear();
	global_min_cut.push_back(global_min_cut1);
	global_min_cut.push_back(global_min_cut2);

	return true;
}

bool ImageGraph::consolidate(int k)
{
	return true;
}

bool ImageGraph::queryExpansionSub(int src, int tgt,
                                   double score, Edge2dArray &expansion_lists, bool **visit_mat,
                                   int level, int inlier_threshold)
{
	if (level < 1) {return false;}
	for (EdgeMap::iterator it = adj_maps_[tgt].begin(); it != adj_maps_[tgt].end(); it++) {
		vot::LinkEdge temp(src, it->second.dst, score * it->second.score);
		if (temp.src != temp.dst &&
		    !visit_mat[src][temp.dst] &&
		    it->second.g_match >= inlier_threshold)
		{
			expansion_lists[src].push_back(temp);
			visit_mat[src][temp.dst] = true;
			queryExpansionSub(src, temp.dst, temp.score, expansion_lists, visit_mat, level-1, inlier_threshold);
		}
	}
	return true;
}

bool ImageGraph::queryExpansion(Edge2dArray &expansion_lists, int level, int inlier_threshold)
{
	const int MAX_LEVEL = 5;
	if (level < 1 || level > MAX_LEVEL) {
		std::cout << "[QueryExpansion] Error: exceed the maximum expansion level (5)\n";
		return false;
	}

	expansion_lists.clear();
	expansion_lists.resize(size_);
	bool **visit_mat = new bool* [size_];
	for (int i = 0; i < size_; i++) {
		visit_mat[i] = new bool [size_];
		memset(visit_mat[i], false, sizeof(bool) * size_);
	}

	// set the first layer connection
	for (int i = 0; i < size_; i++)
		for (EdgeMap::iterator it = adj_maps_[i].begin();
		     it != adj_maps_[i].end();
		     it++)
			visit_mat[i][it->second.dst] = true;

	// query expansion
	for (int i = 0; i < size_; i++) {
		for (EdgeMap::iterator it = adj_maps_[i].begin(); it != adj_maps_[i].end(); it++) {
			if (it->second.g_match >= inlier_threshold) {
				queryExpansionSub(i, it->second.dst, it->second.score,
				                  expansion_lists, visit_mat, level - 1,
				                  inlier_threshold);
			}
		}
	}

	// release the temporary memory for visit_mat
	for (int i = 0; i < size_; i++) {
		delete [] visit_mat[i];
	}
	delete [] visit_mat;
	return true;
}

void ImageGraph::showInfo()
{
	std::cout << "[ImageGraph] Node size: " << size_ << "\n";
	for (int i = 0; i < size_; i++) {
		for (EdgeMap::iterator it = adj_maps_[i].begin(); it != adj_maps_[i].end(); it++) {
			std::cout << it->second.src << " " << it->second.dst << " " << it->second.score << "\n";
		}
	}
}

bool ImageGraph::graphvizu(std::string gv_filename, std::string graph_name)
{
	std::ofstream fout;
	fout.open(gv_filename.c_str());
	if (fout.is_open()) {
		fout << "graph " << graph_name << "{\n";
		// node attributes
		fout << "\tnode [label=\"\", fontsize=5, shape=circle, margin=0, width=0.01];\n";

		// edge attributes
		for (int i = 0; i < size_; i++) {
			for (EdgeMap::iterator it = adj_maps_[i].begin(); it != adj_maps_[i].end(); it++) {
				if (it->second.src < it->second.dst) {
					fout << "\t" << it->second.src << " -- " << it->second.dst << " [penwidth=0.05];\n";
				}
			}
		}
		fout << "}\n";
	}
	else {
		std::cerr << "Fail to open " << gv_filename << " for writing\n";
		return false;
	}
	fout.close();
	return true;
}

int ImageGraph::adjListSize(int idx) { return adj_maps_[idx].size(); }
int ImageGraph::nodeNum() {return size_;}

}   // end of namespace vot
