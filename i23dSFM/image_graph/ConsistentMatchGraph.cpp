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
#include <utility>

#include "ConsistentMatchGraph.hpp"
#include "UnionFind.hpp"

using namespace std;

namespace i23dSFM
{
    // ConsistentMatchGraph::ConsistentMatchGraph()
    // {

    // }

    bool ConsistentMatchGraph::MakeNode(const ImageGraph g)
    {
        std::vector<ImageNode> imageNode = g.GetImageNode();
        for(ImageNode node : imageNode)
        {
            this->_tripletGraph.AddNode(node);
            this->_finalGraph.AddNode(node);
        }
        return true;
    }

    ImageGraph ConsistentMatchGraph::GetTripletGraph() const
    {
        return this->_tripletGraph;
    }

    ImageGraph ConsistentMatchGraph::GetFinalGraph() const
    {
        return this->_finalGraph;
    }

    priority_queue<LinkEdge> ConsistentMatchGraph::OrderEdge(const ImageGraph g) const
    {
        priority_queue<LinkEdge> weightEdge;
        std::vector<EdgeMap> edgeMaps = g.GetEdgeMap();

        // TODO:
        // replace the edge weight by quadratic mean of e_ij

        for(i23dSFM::EdgeMap edgeMap : edgeMaps)
        {
            std::unordered_map<int, LinkEdge>::iterator ite;
            for(ite = edgeMap.begin(); ite != edgeMap.end(); ite++)
            {
                weightEdge.push(ite->second);
            }
        }
        return weightEdge;
    }

    void ConsistentMatchGraph::OnlineMST(const ImageGraph g, size_t rejectThresh, size_t inlierThresh)
    {
        this->MakeNode(g);

        // Make set
        i23dSFM::UnionFind vertex_union(_tripletGraph.GetNodeSize());
        // Ordering edge weight increasingly
        priority_queue<LinkEdge> orderedEdge  = this->OrderEdge(g);
        std::vector<int> failTime(_tripletGraph.GetNodeSize(), 0);

        while(!(orderedEdge.empty() && _tripletGraph.GetEdgeSize() != _tripletGraph.GetNodeSize() - 1))
        {
            LinkEdge e = orderedEdge.top();
            orderedEdge.pop();
            if(vertex_union.Find(e.src) != vertex_union.Find(e.dst) 
                && failTime[e.src] < rejectThresh
                && failTime[e.dst] < rejectThresh)
            {
                // Verify whether edge(i, j) is a true match using
                // a strict inlier threshold
                if(g.GetEdgeMap()[e.src][e.dst].score > inlierThresh)
                {
                    vertex_union.UnionSet(e.src, e.dst);
                    _tripletGraph.AddEdge(e);
                }
                else
                {
                    failTime[e.src]++;
                    failTime[e.dst]++;
                }
            }
        }
    }

    void ConsistentMatchGraph::StrongTripletExpansion(const ImageGraph g, double discreThresh)
    {
        std::vector<MotionMap> motionMaps = _tripletGraph.GetMotionMap();
        std::vector<ImageNode> nodes = _tripletGraph.GetImageNode();
        std::vector<EdgeMap> adjMaps = _tripletGraph.GetEdgeMap();
        // store edges that needed be connected in i-th order triplets expansion
        std::vector<pair<size_t, size_t> > addedEdges; 

        // Triplet expansion
        while(true)
        {
            for(int i = 0; i < nodes.size(); i++)
            {
                if(adjMaps[i].size() < 2) continue; // node i is not a common connected node
                for(int j = 0; j < adjMaps[i].size(); j++)
                {
                    for(int k = j + 1; k < adjMaps.size(); k++)
                    {
                        if(adjMaps[j].find(k) != adjMaps[j].end()) // Already exists an edge between j & k
                            continue;
                        Eigen::Matrix3d R_hat = motionMaps[i][j] * motionMaps[k][i]* motionMaps[j][k];
                        if(this->ComputeDiscrepancy(R_hat) <= discreThresh)
                        {
                            addedEdges.push_back(make_pair(j, k));
                        }
                    }
                }
            }   // end of the first for-loop
            // x-th order strong triplets
            for(auto edge : addedEdges)
            {
                auto linkEdge = g.GetEdgeMap()[edge.first][edge.second];
                _tripletGraph.AddEdge(linkEdge);
            }
            addedEdges.clear();
        }
    }

    void ConsistentMatchGraph::ComponentMerging(size_t communityScale, double loopDiscreThresh)
    {
        // Compute community structures on triplet graph and split v into m communities
        this->ComputeCommunityStructure();
        // Create the candidate matching set
        priority_queue<LinkEdge> matchSet = this->CreateCandidateMatchSet();

        size_t m;   // TODO, community number
        size_t searchTime = communityScale * m * (m - 1) / 2;
        while(!matchSet.empty() && searchTime--)
        {
            LinkEdge edge = matchSet.top(); 
            matchSet.pop();
            vector<size_t> shortestPath = this->_tripletGraph.ShortestPath(edge.src, edge.dst);
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            for(int i = 1; i < shortestPath.size(); i++)
            {
                R *= _tripletGraph.GetMotionMap()[shortestPath[i - 1]][shortestPath[i]];
            }
            R *= shortestPath[0];
            if(this->ComputeDiscrepancy(R) < loopDiscreThresh / sqrt(shortestPath.size()))
            {
                _finalGraph.AddEdge(edge);
            }
        }
    }

    void ConsistentMatchGraph::ComputeCommunityStructure()
    {
        // TODO
    }

    priority_queue<LinkEdge> ConsistentMatchGraph::CreateCandidateMatchSet()
    {
        // TODO
    }

    // vector<size_T> ConsistentMatchGraph::FindShortestPath(size_t src, size_t dst)
    // {
    //     // TODO
    //     vector<size_t> path;
    //     // int ox[] = {1, -1, 0, 0}, oy[] = {0, 0, -1, 1};

    //     for(int i = 0; i < 4; i++)
    //     {
    //         int ux = src + ox[i], uy = src + oy[i];
    //         if()
    //     }
    // }

    double ConsistentMatchGraph::ComputeDiscrepancy(Eigen::Matrix3d R)
    {
        return acos((R.trace() - 1.0) / 2);
    }


} // namespace i23dSFM

