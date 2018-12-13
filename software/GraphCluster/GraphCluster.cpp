/*
Copyright (c) 2018, Yu Chen
All rights reserved.

*/

#include "GraphCluster.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "omp.h"

extern "C"
{
  #include "third_party/graclus/graclus/Graclus.h"
}

// #define COPY_IMAGES

namespace i23dSFM {
GraphCluster::GraphCluster(size_t upper, float cr)
{
    graphUpper = upper;
    completeRatio = cr;
}

ImageGraph GraphCluster::BuildGraph(const string imageList, const string vocFile) const
{
    ImageGraph image_graph;
    ifstream img_in(imageList);
    ifstream voc_in(vocFile);
    string img;
    int idx = 0;
    
    if(!img_in.is_open()) {
        cerr << "File of image list cannot be opened!" << endl;
        return image_graph;
    }
    while(img_in >> img) {
        ImageNode inode(idx, img);
        image_graph.AddNode(inode);
        idx++;
    }
    img_in.close();

    if(!voc_in.is_open()) {
        cerr << "File of vocabulary tree cannot be opened!" << endl;
        return image_graph;
    }
    size_t src, dst;
    float score;
    int k = 0;
    while(voc_in >> src >> dst >> score) {
        // an undirect weight graph is required
         if(src != dst) {
             image_graph.AddEdgeu(src, dst, score);
         }
    }
    voc_in.close();

    return image_graph;
}

string GraphCluster::GenerateNCGraph(const ImageGraph& imageGraph, const string dir) const
{
    int k = 0;
    vector<string> ncFiles = stlplus::folder_files(dir);
    for(auto file : ncFiles) {
        if(file.find("normalized_cut") != string::npos) {
            k++;
        }
    }
    string filename = dir + "/normalized_cut_" + to_string(k) + ".txt";
    ofstream nc_out(filename);
    if(!nc_out.is_open()) {
        cerr << "Normalized-Cut Graph cannot be opened!" << endl;
        return filename;
    }
    nc_out << imageGraph.GetNodeSize() << " " << imageGraph.GetEdgeSize() / 2 << " 1\n";

    std::vector<ImageNode> img_nodes = imageGraph.GetImageNode();
    std::vector<EdgeMap> edge_maps = imageGraph.GetEdgeMap();

    for (int i = 0; i < img_nodes.size(); i++) {
		for (EdgeMap::iterator it = edge_maps[i].begin(); it != edge_maps[i].end(); it++) {
			// nc_out << it->second.dst + 1 << " " << it->second.score * 1e4 << " ";
            nc_out << it->second.dst + 1 << " " << it->second.score << " ";
		}
        nc_out << endl;
	}
    nc_out.close();
    return filename;
}

vector<size_t> GraphCluster::NormalizedCut(const string filename, const size_t clusterNum) const
{
    vector<size_t> clusters;
    char path[256];
    strcpy(path, filename.c_str());

    Graclus graclus = normalizedCut(path, clusterNum);
    
    for(int i = 0; i < graclus.clusterNum; i++) {
        clusters.push_back((size_t)graclus.part[i]);
    }
    
    return clusters;
}

void GraphCluster::MoveImages(queue<shared_ptr<ImageGraph>>& imageGraphs, string dir)
{
    int i = 0;
    while(!imageGraphs.empty()) {
        shared_ptr<ImageGraph> ig = imageGraphs.front();
        imageGraphs.pop();
        std::vector<ImageNode> img_nodes = ig->GetImageNode();

        if(!stlplus::folder_create(dir + "/image_part_" + std::to_string(i))) {
            cerr << "image part " << i << " cannot be created!" << endl;
        }

        for(auto inode : img_nodes) {
            string filename = stlplus::filename_part(inode.image_name);
            string new_file = dir + "/image_part_" + std::to_string(i) + "/" + filename;
            if(!stlplus::file_copy(inode.image_name, new_file)) {
                cout << "cannot copy " << inode.image_name << " to " << new_file << endl;
            }
        }
        i++;
    }
}

void GraphCluster::MoveImages(const vector<shared_ptr<ImageGraph>>& imageGraphs, string dir)
{
    ofstream cluster_list(dir + "/clusters.txt");
    if (!cluster_list.is_open()) {
        std::cerr << "clusters.txt cannot be created!\n";
        return;
    }
    ofstream sfmdata_list(dir + "/sfm_datas.txt");
    if (!sfmdata_list.is_open()) {
        std::cerr << "sfm data list file cannot be created!\n";
        return;
    }

    // #pragma omp parallel
    // {
        // #pragma omp for
        for(int i = 0; i < imageGraphs.size(); i++) {
            std::vector<ImageNode> img_nodes = imageGraphs[i]->GetImageNode();
            string partial_dir = dir + "/image_part_" + std::to_string(i);
            cluster_list << partial_dir << "\n";
            sfmdata_list << partial_dir + "/reconstruction_sequential/robust.json\n";
            if(!stlplus::folder_create(partial_dir)) {
                cerr << "image part " << i << " cannot be created!" << endl;
            }
            imageGraphs[i]->ShowInfo(partial_dir + "/graph_cluster.txt");
            #ifdef COPY_IMAGES
            for(auto inode : img_nodes) {
                string filename = stlplus::filename_part(inode.image_name);
                string new_file = partial_dir + "/" + filename;
                if(!stlplus::file_copy(inode.image_name, new_file)) {
                    cout << "cannot copy " << inode.image_name << " to " << new_file << endl;
                }
            }
            #endif
        }
    // }
    cluster_list.close();
    sfmdata_list.close();
}

void GraphCluster::MoveImages(const vector<shared_ptr<ImageGraph>>& imageGraphs, string inputDir, string outputDir)
{
    ofstream cluster_list(inputDir + "/clusters.txt");
    if (!cluster_list.is_open()) {
        std::cerr << "clusters.txt cannot be created!\n";
        return;
    }
    ofstream sfmdata_list(inputDir + "/sfm_datas.txt");
    if (!sfmdata_list.is_open()) {
        std::cerr << "sfm data list file cannot be created!\n";
        return;
    }

    SfM_Data sfm_data;
    if (!Load(sfm_data, outputDir + "/matches/sfm_data.json", ESfM_Data(VIEWS | INTRINSICS))) {
        std::cerr << "\nThe input SfM_Data file \"" 
                  << outputDir + "/matches/sfm_data.json" 
                  << "\" cannot be read.\n";
    }

    // #pragma omp parallel
    // {
    //     #pragma omp for
        for(int i = 0; i < imageGraphs.size(); i++) {
            std::vector<ImageNode> img_nodes = imageGraphs[i]->GetImageNode();
            string partial_dir = inputDir + "/image_part_" + std::to_string(i);
            cluster_list << partial_dir << "\n";
            sfmdata_list << partial_dir + "/reconstruction_sequential/robust.json\n";

            if(!stlplus::folder_create(partial_dir)) {
                cerr << "image part " << i << " cannot be created!" << endl;
            }
            imageGraphs[i]->ShowInfo(partial_dir + "/graph_cluster.txt");
            GenerateSfMData(sfm_data, img_nodes, partial_dir);
            #ifdef COPY_IMAGES
            for(auto inode : img_nodes) {
                string filename = stlplus::filename_part(inode.image_name);
                string new_file = partial_dir + "/" + filename;
                if(!stlplus::file_copy(inode.image_name, new_file)) {
                    cout << "cannot copy " << inode.image_name << " to " << new_file << endl;
                }
            }
            #endif
        }
    // }

    cluster_list.close();
    sfmdata_list.close();
}

void GraphCluster::GenerateSfMData(const SfM_Data& sfm_data, const std::vector<ImageNode>& imgNodes, string dir)
{
    SfM_Data partial_sfm_data;
    partial_sfm_data.s_root_path = sfm_data.s_root_path;
    Views& views = partial_sfm_data.views;
    Intrinsics& intrinsics = partial_sfm_data.intrinsics;

    // Copy intrinsics
    for (auto ite = sfm_data.intrinsics.begin(); ite != sfm_data.intrinsics.end(); ite++) {
        intrinsics.insert(make_pair(ite->first, ite->second));
    }

    // Copy view
    IndexT views_id = views.size();
    for (auto img_node : imgNodes) {
        string img1 = stlplus::filename_part(img_node.image_name);
        for (auto ite = sfm_data.views.begin(); ite != sfm_data.views.end(); ite++) {
            string img2 = stlplus::filename_part(ite->second->s_Img_path);
            if (img1 == img2) {
                views.insert(make_pair(ite->first, ite->second));
                break;
            }
        }
    }

    string path = dir + "/partial_sfm_data.json";
    if (Save(partial_sfm_data, path, ESfM_Data(ALL))) {
        cout << "sfm_data successfully saved in " << path << endl; 
    }
    else { cout << "Save sfm_data failed!\n"; }
}

pair<shared_ptr<ImageGraph>, shared_ptr<ImageGraph>> GraphCluster::BiPartition(const ImageGraph& imageGraph, string dir)
{
    pair<shared_ptr<ImageGraph>, shared_ptr<ImageGraph>> graph_pair;

    string nc_file = GenerateNCGraph(imageGraph, dir);
    vector<size_t> clusters = NormalizedCut(nc_file, 2);
    queue<shared_ptr<ImageGraph>> graphs = ConstructSubGraphs(imageGraph, clusters, 2);
    if(graphs.size() != 2) {
        cout << "Error occured when bi-partition image graph\n";
        return graph_pair;
    }
    graph_pair.first = graphs.front();
    graphs.pop();
    graph_pair.second = graphs.front();
    graphs.pop();    
    return graph_pair;
}

bool GraphCluster::HasEdge(const vector<shared_ptr<ImageGraph>>& graphs, const LinkEdge& edge) const
{
    bool has_edge = false;
    for(int i = 0; i < graphs.size(); i++) {
        auto graph = graphs[i];
        std::vector<EdgeMap> edge_maps = graph->GetEdgeMap();
        int real_src = graph->Map2CurrentIdx(edge.src);
        int real_dst = graph->Map2CurrentIdx(edge.dst);
        if(real_src == -1 || real_dst == -1) continue;
        EdgeMap::iterator it = edge_maps[real_src].find(real_dst);
        if(it != edge_maps[real_src].end()) { has_edge = true; break; }
    }
    return has_edge;
}

bool GraphCluster::HasEdge(queue<shared_ptr<ImageGraph>> graphs, const LinkEdge& edge)
{
    std::vector<shared_ptr<ImageGraph>> igs;
    while(!graphs.empty()) {
        igs.push_back(graphs.front());
        graphs.pop();
    }
    bool hasEdge = HasEdge(igs, edge);
    for(auto graph : igs) { graphs.push(graph); }
    return hasEdge;
}

priority_queue<LinkEdge> GraphCluster::DiscardedEdges(const ImageGraph& imageGraph, 
                                                      const vector<shared_ptr<ImageGraph>>& insizeGraphs, 
                                                      const queue<shared_ptr<ImageGraph>>& candidateGraphs)
{
    priority_queue<LinkEdge> discarded_edges;
    std::vector<EdgeMap> edge_maps = imageGraph.GetEdgeMap();
    std::vector<ImageNode> nodes = imageGraph.GetImageNode();
    // #pragma omp parallel
    // {
    //     #pragma omp for
        for(int i = 0; i < nodes.size(); i++) {
            for(int j = i + 1; j < nodes.size(); j++) {
                EdgeMap::iterator it = edge_maps[i].find(j);
                if(it != edge_maps[i].end()) {
                    LinkEdge edge = it->second;
                    // if(!HasEdge(insizeGraphs, edge) && !HasEdge(candidateGraphs, edge)) {
                    if(!HasEdge(insizeGraphs, edge)) {
                        discarded_edges.push(edge);
                    }
                }
            }
        }
    // }
    return discarded_edges;
}

int GraphCluster::RepeatedNodeNum(const ImageGraph& imageGraphL, const ImageGraph& imageGraphR)
{
    int num = 0;
    std::vector<ImageNode> l_nodes = imageGraphL.GetImageNode();
    std::vector<ImageNode> r_nodes = imageGraphR.GetImageNode();
    #pragma omp parallel
    {
        #pragma omp for
        for(int i = 0; i < l_nodes.size(); i++) {
            for(int j = 0; j < r_nodes.size(); j++) {
                if(l_nodes[i].idx == r_nodes[j].idx) num++;
            }
        }
    }
    return num;
}

pair<ImageNode, shared_ptr<ImageGraph>> GraphCluster::SelectCRGraph(const ImageGraph& imageGraph, 
                                                                    const vector<shared_ptr<ImageGraph>>& graphs, 
                                                                    const LinkEdge& edge)
{
    ImageNode unselected_node, selected_node;
    bool find = false;
    srand((unsigned)time(NULL));
    int ran = rand() % 2;

    for(int i = 0; i < graphs.size(); i++) {
        unselected_node = (ran == 0) ? imageGraph.GetNode(edge.dst) : imageGraph.GetNode(edge.src);
        selected_node = (ran == 0) ? graphs[i]->GetNode(edge.src) : graphs[i]->GetNode(edge.dst);
        int repeatedNodeNum = 0;
        if(selected_node.idx != -1) {
            #pragma omp parallel
            {
                #pragma omp for
                for(int j = 0; j < graphs.size(); j++) {
                    if(i != j) {
                        repeatedNodeNum += RepeatedNodeNum(*graphs[i], *graphs[j]);
                        // if((float)repeatedNodeNum / (float)graphs[i].GetNodeSize() > completeRatio) {
                        //     break;
                        // }
                    }
                }
            }
            if((float)repeatedNodeNum / (float)graphs[i]->GetNodeSize() < completeRatio) {
                return make_pair(unselected_node, graphs[i]);
            }
        }
    }
    shared_ptr<ImageGraph> rep(new ImageGraph());
    return make_pair(ImageNode(), rep);
}

bool GraphCluster::IsSatisfyCompleteConstraint(const ImageGraph& imageGraph, 
                                               const vector<shared_ptr<ImageGraph>>& graphs)
{
    int repeatedNodeNum = 0;
    for(int i = 0; i < graphs.size(); i++) {
        #pragma omp parallel
        {
            #pragma omp for
            for(int j = 0; j < graphs.size(); j++) {
                if(i != j) {
                    repeatedNodeNum += RepeatedNodeNum(*graphs[i], *graphs[j]);
                    // if((float)repeatedNodeNum / (float)graphs[i].GetNodeSize() > completeRatio) {
                    //     break;
                    // }
                }
            }
        }
        if((float)repeatedNodeNum / (float)graphs[i]->GetNodeSize() < completeRatio) {
            return true;
        }
    }
    return false;
}

vector<shared_ptr<ImageGraph>> GraphCluster::ExpanGraphCluster(const ImageGraph& imageGraph, 
                                                               queue<shared_ptr<ImageGraph>> imageGraphs, 
                                                               const string dir, 
                                                               const size_t clusterNum)
{
    vector<shared_ptr<ImageGraph>> insize_graphs;
    queue<shared_ptr<ImageGraph>>& candidate_graphs = imageGraphs;

    while(!candidate_graphs.empty()) {
        while(!candidate_graphs.empty()) {
            shared_ptr<ImageGraph> ig = candidate_graphs.front();
            candidate_graphs.pop();
            if(ig->GetNodeSize() <= graphUpper) {
                insize_graphs.push_back(ig);
            }
            else {
                pair<shared_ptr<ImageGraph>, shared_ptr<ImageGraph>> ig_pair = BiPartition(*ig, dir);
                candidate_graphs.push(ig_pair.first);
                candidate_graphs.push(ig_pair.second);
            }
        }

        // Graph expansion
        priority_queue<LinkEdge> discarded_edges = 
            DiscardedEdges(imageGraph, insize_graphs, candidate_graphs);
        while(!discarded_edges.empty()) {
            LinkEdge edge = discarded_edges.top();
            discarded_edges.pop();
            cout << "Discarded Edges: " << edge.src << ", " << edge.dst << endl; 

            pair<ImageNode, shared_ptr<ImageGraph>> crp_graph = SelectCRGraph(imageGraph, insize_graphs, edge);
            ImageNode unselected_node = crp_graph.first;
            shared_ptr<ImageGraph> selected_graph = crp_graph.second;
            if(unselected_node.idx != -1) {
                if(selected_graph->GetNode(unselected_node.idx).idx == -1) {
                    selected_graph->AddNode(unselected_node);
                }
                selected_graph->AddEdgeu(selected_graph->Map2CurrentIdx(edge.src), 
                                         selected_graph->Map2CurrentIdx(edge.dst), 
                                         edge.score);
            }
        }
        // After graph expansion, there may be some image graphs that don't
        // satisfy the size constraint, check this condition
        std::vector<shared_ptr<ImageGraph>>::iterator igIte;
        for(igIte = insize_graphs.begin(); igIte != insize_graphs.end();) {
            // Make a little relax of original constraint condition
            if((*igIte)->GetNodeSize() > graphUpper + (int)(graphUpper * relaxRatio)) {
            // if((*igIte)->GetNodeSize() > graphUpper) {
                candidate_graphs.push(*igIte);
                igIte = insize_graphs.erase(igIte);
            }
            else igIte++;
        }
    }
    cout << "End ExpanGraphCluster\n";
    return insize_graphs;
}

vector<shared_ptr<ImageGraph>> GraphCluster::NaiveGraphCluster(queue<shared_ptr<ImageGraph>> imageGraphs, 
                                                               const string dir, 
                                                               const size_t clusterNum)
{
    MoveImages(imageGraphs, dir);
}

queue<shared_ptr<ImageGraph>> GraphCluster::ConstructSubGraphs(ImageGraph imageGraph, 
                                                               const vector<size_t>& clusters, 
                                                               const size_t clusterNum)
{
    queue<shared_ptr<ImageGraph>> imageGraphs;
    std::vector<ImageNode> nodes = imageGraph.GetImageNode();
    std::vector<EdgeMap> edge_maps = imageGraph.GetEdgeMap();
    EdgeMap::iterator it;
    
    #pragma omp parallel
    {
        #pragma omp for
        for(int i = 0; i < clusterNum; i++) {
            shared_ptr<ImageGraph> ig(new ImageGraph());
            // Add nodes
            for(int j = 0; j < clusters.size(); j++) {
                if(clusters[j] == i) {
                    ig->AddNode(nodes[j]);
                }
            }

            // Add edges
            std::vector<ImageNode> ig_nodes = ig->GetImageNode();
            for(int l = 0; l < ig_nodes.size(); l++) {
                for(int r = l + 1; r < ig_nodes.size(); r++) {
                    it = edge_maps[imageGraph.Map2CurrentIdx(ig_nodes[l].idx)].find(
                                imageGraph.Map2CurrentIdx(ig_nodes[r].idx));
                    if(it != edge_maps[imageGraph.Map2CurrentIdx(ig_nodes[l].idx)].end()) {
                        ig->AddEdgeu(l, r, it->second.score);
                    }
                }
            }
            imageGraphs.push(ig);
        }
    }
    return imageGraphs;
}

}   // namespace i23dSfM