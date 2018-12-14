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

#include <vector>

#include "GraphCluster.hpp"

#include "i23dSFM/matching/indMatch_utils.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace std;
using namespace i23dSFM;
using namespace i23dSFM::matching;

#define __DEBUG__


vector<PairWiseMatches> BuildClusterMatches(const PairWiseMatches& mapMatches,
                                            const vector<shared_ptr<ImageGraph>>& igList)
{
    vector<PairWiseMatches> matches_list;
    map<Pair, IndMatches>::iterator ite;
    
    for (auto ig : igList) {
        PairWiseMatches pm_matches;
        std::vector<EdgeMap> adj_maps = ig->GetEdgeMap();
        for (auto edge_map : adj_maps) {
            for (auto ite = edge_map.begin(); ite != edge_map.end(); ite++) {
                // Pair pair(ite->second.src, ite->second.dst);
                int src = ig->GetImageNode()[ite->second.src].idx;
                int dst = ig->GetImageNode()[ite->second.dst].idx;
                Pair pair1(src, dst), pair2(dst, src);
                auto match_ite1 = mapMatches.find(pair1);
                auto match_ite2 = mapMatches.find(pair2);
                if (match_ite1 != mapMatches.end()) {
                    IndMatches ind_matches = match_ite1->second;
                    pm_matches.insert(make_pair(pair1, ind_matches));
                }
                else if (match_ite2 != mapMatches.end()) {
                    IndMatches ind_matches = match_ite2->second;
                    pm_matches.insert(make_pair(pair2, ind_matches));
                }
            }
        }
        matches_list.push_back(pm_matches);
    }
    return matches_list;
}

void OutputClusterMatches(const vector<PairWiseMatches>& matchesList, string dir)
{
    cout << "Begin output cluster matches\n";
    // if (!stlplus::folder_exists(dir)) stlplus::folder_create(dir);
    ofstream stream(dir + "/matches_list.txt");
    for(int i = 0; i < matchesList.size(); i++)
    {
        string matches_name = dir + "/cluster_matches" + to_string(i) + ".txt";
        ofstream fis(matches_name);
        stream << matches_name << endl;
        if(fis.is_open()) PairedIndMatchToStream(matchesList[i], fis);
        fis.close();
    }
    stream << endl;
    cout << "End output cluster matches\n";
}

PairWiseMatches RecoverMatches(string fileName)
{
    PairWiseMatches map_GeometricMatches;
    ifstream fis(fileName.c_str());
    IndexT i, j;
    int matchesSize;
    int indLeft, indRight;
    if(fis.is_open()) {
        while(fis >> i >> j) {
            fis >> matchesSize;
            IndMatches indMatches;
            while(matchesSize--) {
                fis >> indLeft >> indRight;
                indMatches.push_back(IndMatch(indLeft, indRight));
            }
            map_GeometricMatches.insert(make_pair(Pair(i, j), indMatches));
        }
    }

    else {
        cout << "Failed to open file " << fileName << " in recoverMatches" << endl;
        return map_GeometricMatches;
    }
    return map_GeometricMatches;
}


int main(int argc, char ** argv)
{
    if(argc < 7) {
        cout << "Please input file path of the vocabulary tree\n" << endl;
        cout << "Usage: \n" << 
            "i23dSFM_GraphCluster absolut_img_path absolut_voc_path " << 
            "cluster_option=expansion max_img_size completeness_ratio output_dir\n";
        cout << "Notice: cluster_option must be 'naive' or 'expansion'\n";
        return 0;
    }

    // It's best to put them on image root path
    string img_list(argv[1]);
    string voc_file(argv[2]); 
    string cluster_option(argv[3]);
    int max_image_num = atoi(argv[4]);
    float completeness_ratio = atof(argv[5]);
    string matches_dir(argv[6]);
    string output_dir(argv[7]);

    GraphCluster graph_cluster(max_image_num, completeness_ratio);
    
    string dir = stlplus::folder_part(voc_file);
    ImageGraph img_graph = graph_cluster.BuildGraph(img_list, voc_file);

#ifdef __DEBUG__
    img_graph.ShowInfo();
#endif

    size_t clustNum = 1;
    cout << "nodes: " << img_graph.GetNodeSize() << endl;
    cout << "graphUpper: " << graph_cluster.graphUpper << endl;
    if(img_graph.GetNodeSize() < graph_cluster.graphUpper) {
        cout << "size of graphs less than cluster size, camera cluster is the origin one\n";
        return 0;
    }
    else {
        clustNum = img_graph.GetNodeSize() / graph_cluster.graphUpper;
    }

    string nc_graph = graph_cluster.GenerateNCGraph(img_graph, dir);
    vector<size_t> clusters = graph_cluster.NormalizedCut(nc_graph, clustNum);
    queue<shared_ptr<ImageGraph>> sub_image_graphs = 
        graph_cluster.ConstructSubGraphs(img_graph, clusters, clustNum);

    if(cluster_option == "naive") {
        graph_cluster.NaiveGraphCluster(sub_image_graphs, dir, clustNum);
    }
    else if(cluster_option == "expansion") {
        vector<shared_ptr<ImageGraph>> insize_graphs = 
            graph_cluster.ExpanGraphCluster(img_graph, sub_image_graphs, dir, clustNum);
        // graph_cluster.MoveImages(insize_graphs, dir);
        graph_cluster.MoveImages(insize_graphs, dir, output_dir);
        cout << "Recover matches\n";
        PairWiseMatches pairwise_matches = RecoverMatches(matches_dir + "/matches.f.txt");
        vector<PairWiseMatches> matches = BuildClusterMatches(pairwise_matches, insize_graphs);
        OutputClusterMatches(matches, stlplus::folder_part(img_list));
        cout << "End recover matches\n";
    }
    else {
        cout << "cluster_option must be 'naive' or 'expansion'\n";
        return 0;
    }

}