#ifndef GRAPH_CLUSTER_HPP
#define GRAPH_CLUSTER_HPP

#include <string>
#include <fstream>
#include <cstring>
// #include <vector>
#include <memory>
#include <queue>
#include <utility>

#include "i23dSFM/image_graph/ImageGraph.hpp"
#include "i23dSFM/sfm/sfm_data.hpp"
#include "i23dSFM/sfm/sfm_data_io.hpp"
#include "i23dSFM/sfm/pipelines/sfm_engine.hpp"
#include "i23dSFM/sfm/pipelines/sfm_features_provider.hpp"
#include "i23dSFM/sfm/pipelines/sfm_regions_provider.hpp"

using namespace Eigen;
using namespace i23dSFM;
using namespace i23dSFM::sfm;
using namespace i23dSFM::cameras;
using namespace i23dSFM::matching;
using namespace std;


namespace i23dSFM {
class GraphCluster 
{
private:
  const double relaxRatio = 0.35;

public:
  size_t graphUpper;
  float completeRatio;

public:

GraphCluster(size_t upper = 100, float cr = 0.7);
/**
 * @brief
 * @param imageList:
 * @param vocFile:
 * @return
 */
ImageGraph BuildGraph(const string imageList, const string vocFile) const;

/**
 * @brief
 * @param imageGraph:
 * @param dir: directory that store the normalized-cut files
 * @return
 */
string GenerateNCGraph(const ImageGraph& imageGraph, const string dir) const;

/**
 * @brief
 * @param filename:
 * @param clusterNum:
 * @return
 */
vector<size_t> NormalizedCut(const string filename, const size_t clusterNum) const;  

/**
 * @brief Naive graph cluster
 * @param imageGraph:
 * @param clusters:
 * @param clusterNum:
 */
void MoveImages(queue<shared_ptr<ImageGraph>>& imageGraphs, string dir);  

/**
 * @brief Naive graph cluster
 * @param imageGraph:
 * @param clusters:
 * @param clusterNum:
 */
void MoveImages(const vector<shared_ptr<ImageGraph>>& imageGraphs, string dir);  

/**
 * @brief Naive graph cluster
 * @param imageGraph:
 * @param clusters:
 * @param clusterNum:
 */
void MoveImages(const vector<shared_ptr<ImageGraph>>& imageGraphs, string inputDir, string outputDir); 

void GenerateSfMData(const SfM_Data& sfm_data, const std::vector<ImageNode>& imgNodes, string dir);

/**
 * @brief
 * @param imageGraph
 * @return
 */
pair<shared_ptr<ImageGraph>, shared_ptr<ImageGraph>> BiPartition(const ImageGraph& imageGraph, string dir); 

/**
 * @brief
 * @param igraphs
 * @param edge:
 * @return 
 */
bool HasEdge(const vector<shared_ptr<ImageGraph>>& graphs, const LinkEdge& edge) const;  

/**
 * @brief
 * @param igraphs
 * @param edge:
 * @return 
 */
bool HasEdge(queue<shared_ptr<ImageGraph>> graphs, const LinkEdge& edge);

/**
 * @brief Collect discarded edges in graph division
 * @param imageGraph:
 * @param insizeGraphs:
 * @param candidateGraphs:
 * @return
 */
priority_queue<LinkEdge> DiscardedEdges(const ImageGraph& imageGraph, 
                                        const vector<shared_ptr<ImageGraph>>& insizeGraphs, 
                                        const queue<shared_ptr<ImageGraph>>& candidateGraphs);

bool IsSatisfyCompleteConstraint(const ImageGraph& imageGraph, 
                                 const vector<shared_ptr<ImageGraph>>& graphs);
/**
 * @brief Select a image graph randomly which satisfy the completeness ratio
 * @param graphs:
 * @param edge:
 * @return
 */
pair<ImageNode, shared_ptr<ImageGraph>> SelectCRGraph(const ImageGraph& imageGraph, 
                                                      const vector<shared_ptr<ImageGraph>>& graphs, 
                                                      const LinkEdge& edge); 

/**
 * 
 */
int RepeatedNodeNum(const ImageGraph& imageGraphL, const ImageGraph& imageGraphR); 

/**
 * @brief 
 * @param imageGraph:
 * @param ncFile:
 * @param clusterNum:
 */
vector<shared_ptr<ImageGraph>> ExpanGraphCluster(const ImageGraph& imageGraph, 
                                                 queue<shared_ptr<ImageGraph>> imageGraphs, 
                                                 const string dir, 
                                                 const size_t clusterNum); 

/**
 * 
 * 
 * 
 */
vector<shared_ptr<ImageGraph>> NaiveGraphCluster(queue<shared_ptr<ImageGraph>> imageGraphs, 
                                                 const string dir, 
                                                 const size_t clusterNum); 

/**
 * 
 * 
 * 
 */
// void GraphCluster(const ImageGraph imageGraph, string dir, size_t clusterNum); 

/**
 * @brief
 * @param imageGraph
 * @param clusters
 * @param clusterNum
 * @return
 */
queue<shared_ptr<ImageGraph>> ConstructSubGraphs(ImageGraph imageGraph, 
                                                 const vector<size_t>& clusters, 
                                                 const size_t clusterNum); 
};

}   // namespace i23dSfM

#endif