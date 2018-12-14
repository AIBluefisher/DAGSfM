// BSD 3-Clause License

// Copyright (c) 2018, 陈煜
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <stack>

#include "ClusterGraph.hpp"
#include "UnionFind.hpp"

using std::cout;
using std::endl;

namespace i23dSFM
{
ClusterGraph::ClusterGraph()
{
	size_ = 0;
}

ClusterGraph::ClusterGraph(int size)
{
	size_ = size;
	adj_maps_.resize(size_);
	nodes_.resize(size_);
	for (int i = 0; i < size_; i++)
		nodes_[i] = ImageNode();
}

ClusterGraph::ClusterGraph(const std::vector<std::string>& sfm_data_filenames)
{
	size_ = sfm_data_filenames.size();
	adj_maps_.resize(size_);
	nodes_.resize(size_);
	for (int i = 0; i < size_; i++)
		nodes_[i] = ImageNode(i, sfm_data_filenames[i]);
}

void ClusterGraph::AddNode()
{
	nodes_.push_back(i23dSFM::ImageNode());
	adj_maps_.push_back(EdgeMap());
	size_ = nodes_.size();
}

void ClusterGraph::AddNode(const i23dSFM::ImageNode &n)
{
	nodes_.push_back(i23dSFM::ImageNode(n));
	adj_maps_.push_back(EdgeMap());
	size_ = nodes_.size();
}

void ClusterGraph::AddEdge(int src, int dst, double score)
{
	EdgeMap::iterator it = adj_maps_[src].find(dst);
	if (it == adj_maps_[src].end()) {
		adj_maps_[src].insert(std::make_pair(dst, i23dSFM::LinkEdge(src, dst, score)));
	}
}

void ClusterGraph::AddEdge(const i23dSFM::LinkEdge &n)
{
	int src = n.src, dst = n.dst;
	EdgeMap::iterator it = adj_maps_[src].find(dst);
	if (it == adj_maps_[src].end()) {
		adj_maps_[src].insert(std::make_pair(dst, n));
	}
}

void ClusterGraph::AddEdgeu(int src, int dst, double score)
{
	AddEdge(src, dst, score);
	AddEdge(dst, src, score);
}

void ClusterGraph::AddEdgeu(const i23dSFM::LinkEdge &n)
{
	AddEdge(n);
	i23dSFM::LinkEdge n_inv(n.dst, n.src, n.score);
	AddEdge(n_inv);
}

int ClusterGraph::NumConnectedComponents(int threshold)
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

void ClusterGraph::ShowInfo()
{
	std::cout << "[ClusterGraph] Node size: " << nodes_.size() << "\n";
	for(int i = 0; i < nodes_.size(); i++) cout << nodes_[i].idx << " ";
	std::cout << "\n[ClusterGraph] Edge size: " << GetEdgeSize() << endl;
	for(auto adjs : adj_maps_) {
		for(EdgeMap::iterator it = adjs.begin(); it != adjs.end(); it++) {
			std::cout << it->second.src << " " << it->second.dst << " " << it->second.score << "\n";
		}
	}
}

bool ClusterGraph::Graphvizu(std::string gv_filename, std::string graph_name)
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

std::vector<size_t> ClusterGraph::ShortestPath(size_t src, size_t dst) const
{
	std::vector<size_t> path;
	std::vector<bool> visited(this->size_, false);
	std::queue<PathNode*> qu;
	qu.push(new PathNode(src, NULL));

	while(!qu.empty())
	{
		PathNode* node = qu.front();
		if(node->idx == dst)
		{
			while(node->parent != NULL)
			{
				path.push_back(node->idx);
				node = node->parent;
			}
			path.push_back(node->idx);
			break;
		}
		EdgeMap edge = this->adj_maps_[node->idx];
		std::unordered_map<int, LinkEdge>::iterator ite;
		for(ite = edge.begin(); ite != edge.end(); ite++)
		{
			qu.push(new PathNode(ite->second.dst, node));
		}
	}
	return path;
}

ImageNode ClusterGraph::GetNode(int idx) const
{
	for(auto node : nodes_) {
		if(node.idx == idx) { return node; }
	}
	return ImageNode();
}

int ClusterGraph::AdjListSize(int idx) { return adj_maps_[idx].size(); }
int ClusterGraph::NodeNum() {return size_;}

int ClusterGraph::GetEdgeSize() const 
{ 
	int edge_num = 0;
	for (int i = 0; i < size_; i++) {
		EdgeMap edgeMap = adj_maps_[i];
		for (EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); it++) {
			edge_num++;
		}
	}
	return edge_num; 
}
int ClusterGraph::GetNodeSize() const { return nodes_.size(); }
std::vector<ImageNode> ClusterGraph::GetImageNode() const { return nodes_; }
std::vector<EdgeMap> ClusterGraph::GetEdgeMap() const { return adj_maps_; }
std::vector<EdgeMap>& ClusterGraph::GetEdges() { return adj_maps_; }

int ClusterGraph::Map2CurrentIdx(int idx)
{
	for(int i = 0; i < nodes_.size(); i++) {
		if(nodes_[i].idx == idx) return i;
	}
	return -1;
}

bool ClusterGraph::DeleteNode(ImageNode& node)
{
	bool find = false;
	for(auto it = nodes_.begin(); it != nodes_.end();) {
		if(it->idx == node.idx) {
			it = nodes_.erase(it);
			find = true;
			// break;
		}
		else it++;
	}
	if(!find) return false;

	// update edges
	// for(EdgeMap& em : adj_maps_) {
	// 	for(auto it = em.begin(); it != em.end(); it++) {
	// 		if(it->first == node.idx) em.erase(it);
	// 	}
	// }
	// int i = 0;
	// for(auto it = adj_maps_.begin(); it != adj_maps_.end();) {
	// 	if(i == node.idx) {
	// 		adj_maps_.erase(it);
	// 		break;
	// 	}
	// 	it++; i++;
	// }
	return true;
}

bool ClusterGraph::DeleteNode(size_t index)
{
	bool find = false;
	for(vector<ImageNode>::iterator it = nodes_.begin(); it != nodes_.end();) {
		if(it->idx == index) {
			it = nodes_.erase(it);
			find = true;
			// break;
		}
		else it++;
	}
	
	if(!find) return false;

	// delete edges
	// vector<pair<int, int>> edges_delete;
	// for(EdgeMap& em : adj_maps_) {
	// 	for(auto it = em.begin(); it != em.end(); it++) {
	// 		if(it->second.src == index || it->second.dst == index) {
	// 			edges_delete.push_back(make_pair(it->second.src, it->second.dst));
	// 		}
	// 	}
	// }
	// for (auto edge : edges_delete) { this->RemoveEdge(edge.first, edge.second); }

	return true;
}

bool ClusterGraph::UpdateNode(size_t index, string sfm_data_path)
{
	for(auto& node : nodes_) {
		if(node.idx == index) {
			node.sfm_data_path = sfm_data_path;
			return true;
		}
	}
	return false;
}

size_t ClusterGraph::FindConnectedNode(size_t src, size_t dst)
{
	// Finding node that connect src
	for (auto adj_map : adj_maps_) {
		for (auto ite = adj_map.begin(); ite != adj_map.end(); ite++) {
			auto edge = ite->second;
			if (edge.src == src && edge.dst != dst) {
				return edge.dst;
			}  
			if (edge.dst == src && edge.src != dst) {
				return edge.src;
			}
		}
	}
	return src;
}

bool ClusterGraph::RemoveEdge(size_t src, size_t dst)
{
	for(auto& em : adj_maps_) {
		for(auto it = em.begin(); it != em.end(); it++) {
			if(it->second.src == src && it->second.dst == dst) {
				em.erase(it);
				return true;
			}
		}
	}
	return false;
}

vector<LinkEdge> ClusterGraph::Prim() const 
{
	vector<LinkEdge> ans;
	const int max_v = nodes_.size();
    float cost[max_v][max_v];
	const float max_val = 1000.0; // numeric_limits<float>::max();

	// Convert adjacent list to adjacent matrix
    vector<EdgeMap> adjs = adj_maps_;
    for(int i = 0; i < max_v; i++) {
        for(int j = 0; j < max_v; j++) {
			if(i == j) cost[i][j] = 0;
            else if(adjs[i].find(j) != adjs[i].end()) {
				cost[i][j] = adjs[i][j].score;
			}
            else cost[i][j] = max_val;
        }
    }
	cout << "max_v: " << max_v << endl;
	// Prim algorithm
	float lowcost[max_v];
	int closest[max_v];
	for(int i = 0; i < max_v; i++) {
		lowcost[i] = cost[0][i];
		closest[i] = 0;
	}

	for(int i = 1; i < max_v; i++) {
		float minw = max_val;
		int k = -1;
		for(int j = 0; j < max_v; j++) {
			if(lowcost[j] != 0 && lowcost[j] < minw) {
				minw = lowcost[j];
				k = j;
			}
		}
		// store edges of MST
		ans.push_back(LinkEdge(closest[k], k));
		lowcost[k] = 0;
		cout << "MST edge: " << closest[k] << ", " << k << endl;
		// Update edges
		for(int j = 0; j < max_v; j++) {
			if(cost[k][j] != 0 && cost[k][j] < lowcost[j]) {
				lowcost[j] = cost[k][j];
				closest[j] = k;
			}
		}
	}

    return ans;
}

vector<LinkEdge> ClusterGraph::Kruskal() const
{
	vector<LinkEdge> ans;
	priority_queue<LinkEdge> edges;

	for (auto adj_map : adj_maps_) {
		for (auto ite = adj_map.begin(); ite != adj_map.end(); ite++) {
			edges.push(ite->second);
		}
	}

	int n = nodes_.size();
	UnionFind union_find(n);
	
	while (!edges.empty()) {
		LinkEdge edge = edges.top(); edges.pop();
		if (union_find.FindRoot(edge.src) != union_find.FindRoot(edge.dst)) {
			cout << "MST edge: " << edge.src << ", " << edge.dst << endl;
			union_find.UnionSet(edge.src, edge.dst);
			ans.push_back(edge);
		}
	}
	return ans;
}

unordered_map<int, int> ClusterGraph::ComputeDegree() const 
{
	unordered_map<int, int> degree;
	for(int i = 0; i < adj_maps_.size(); i++) {
        auto adj = adj_maps_[i];
        for(auto it = adj.begin(); it != adj.end(); it++) {
            degree[it->second.src]++;
            degree[it->second.dst]++;
        }
    }
	return degree;
}

int ClusterGraph::FindLeafNode(const unordered_map<int, int>& degree) const
{
	// Finding nodes with degree equals 1
    int idx = -1;
    for (auto ite = degree.begin(); ite != degree.end(); ite++) {
        if (ite->second == 1) { idx = ite->first; break; }
    }
	return idx;
}

LinkEdge ClusterGraph::FindConnectedEdge(const int idx) const
{
	LinkEdge edge;
    for(auto adj : adj_maps_) {
        bool find = false;
        for(auto it  = adj.begin(); it != adj.end(); it++) {
            if(it->second.src == idx || it->first == idx) {
                edge = it->second;
                find = true; break;
            }
        }
        if(find) { break; }
    }
	return edge;
}

}   // end of namespace i23dSFM
