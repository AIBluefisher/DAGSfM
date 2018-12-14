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

#ifndef i23dSFM_SFM_ALIGNER_HPP
#define i23dSFM_SFM_ALIGNER_HPP

#include <utility>

#include "ClusterGraph.hpp"

#include "i23dSFM/sfm/sfm.hpp"
// #include "i23dSFM/tracks/tracks.hpp"
#include "i23dSFM/geometry/rigid_transformation3D_srt.hpp"

using namespace i23dSFM::sfm;

namespace i23dSFM {
class SfMAligner 
{
    struct Sim3
    {
        // Eigen::Matrix4d similarity;
        // std::string sfm_data_path;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        double s;

        Sim3()
        {
            R = Eigen::Matrix3d::Identity();
            t = Eigen::Vector3d::Zero();
            s = 1.0;
        }
        Sim3(Eigen::Matrix3d rotation, Eigen::Vector3d translation, double scale)
        {
            // sfm_data_path = path;
            R = rotation;
            t = translation;
            s = scale; 
        }
    };
private:
    /** sfm data that stores landmarks and poses */
    SfM_Data _sfm_data; 

    /** merged sfm_data path*/
    string _merged_sfm_data_path;

    /**
     * Graph that stores the connection information between clusters.
     * Each node represents a cluster, node.idx is the id of cluster, node.image_name
     * is the absolute path of sfm_data.json in cluster.
     * Weight of edge between clusters represents the number of common 3D observations
     */ 
    ClusterGraph _cluster_graph;
    
    std::vector<std::unordered_map<int, Sim3>> _paths;
    std::vector<std::string> _ori_sfmdatas;
    std::vector<Sim3> _sim_paths;   // used to do final transformation
    int _last_idx;
public:
    /**
     * Constructor
     */
    SfMAligner();
    /**
     * Destructor
     */
    ~SfMAligner();  
    /** 
     * @brief  Initialize cluster graph after local incremental SfM
     * @note   
     * @param  filename: absolute path of file that stores all the paths of clusters
     * @retval True if initilalize succeed, else false
     */
    bool InitializeGraph(std::string filename);
    /** 
     * @brief  Update cluster graph by the path of MST
     * @note   
     * @retval None
     */
    void UpdateGraph(); 

    /** 
     * @brief Merge two clusters, sfm_data should be merged, and the original two nodes
     * should be replaced by the new merged node. By default, the cluster with less 3D points
     * is merged into the other cluster with more 3D point 
     * @note   
     * @param  dir: directory that stores the merged sfm_data.json
     * @retval None
     */
    void MergeClusters(string dir = "", string img_path = "");
    /** 
     * @brief  Find common cameras between two clusters
     * @note   
     * @param  sfm_data1: sfm_data belongs to cluster1
     * @param  sfm_data2: sfm_data belongs to cluster2
     * @param  poses1: camera poses belongs to cluster1
     * @param  poses2: camera poses belongs to cluster2
     * @retval None
     */
    void FindCommonCameras(const SfM_Data& sfm_data1, 
                            const SfM_Data& sfm_data2,
                            std::vector<pair<int, geometry::Pose3>>& poses1,
                            std::vector<pair<int, geometry::Pose3>>& poses2);
    /** 
     * @brief  Find common 3D observations between two clusters
     * @note   
     * @param  sfm_data1: sfm_data belongs to cluster1
     * @param  sfm_data2: sfm_data belongs to cluster2
     * @param  observations1: observations belongs to cluster1
     * @param  observations2: observations belongs to cluster2
     * @retval None
     */
    void FindCommonObservations(const SfM_Data& sfm_data1, 
                                const SfM_Data& sfm_data2,
                                std::vector<Vector3d>& observations1,
                                std::vector<Vector3d>& observations2);
    /** 
     * @brief  Find similarity transform between two clusters by using common observations
     * @note   
     * @param  observations1: observations belongs to cluster1
     * @param  observations2: observations belongs to cluster2
     * @param  R: rotation matrix of similarity transform
     * @param  t: translation vector of similarity transform
     * @param  scale: scale of similarity transform
     * @retval None
     */
    void FindSimilarityTransform(const std::vector<Vector3d>& observations1,
                                 const std::vector<Vector3d>& observations2,
                                 Mat3& R,
                                 Vector3d& t,
                                 double& scale,
                                 double& msd);

    /** 
     * @brief  Refine the structure by bundle adjustment after all clusters are merged into one
     * @note
     * @retval True if refine complete, else false
     */
    bool Refine(); 
    /** 
     * @brief  Get cluster graph
     * @note   
     * @retval ClusterGraph
     */
    ClusterGraph GetClusterGraph() const; 

    /** 
     * @brief  Generate .ply file from json file in path1 and stored in path2
     * @note   
     * @param  path1: path that store the sfm_data.json file
     * @param  path2: path that store the .ply file
     * @param  R: rotation matrix of similarity transform
     * @param  t: translation vector of similarity transform
     * @param  scale: scale of similarity transform
     * @retval None
     */
    // void TransformPly(string path1, string path2, const Eigen::Matrix4d& sim3);
    void TransformPly(const string& path1, 
                      const string& path2, 
                      const Mat3& R, 
                      const Vector3d& t, 
                      const double& scale) const;

    void TransformPly(const string& path1, 
                      const string& path2, 
                      const Mat3& R, 
                      const Vector3d& t, 
                      const double& scale,
                      const Vec3& color) const;
        void ComputePath(int src, int dst);

    std::vector<Vec3> GenerateColors(int size) const;
    void RenderCluster(const int idx, const std::vector<Vec3>& colors) const;
    void RenderClusters(const std::vector<Vec3>& colors) const;
    
private:
    /** 
     * @brief  
     * @note   
     * @param  path1: 
     * @param  path2: 
     * @param  edge: 
     * @retval None
     */
    void ConstructEdge(string path1, string path2, LinkEdge& edge);
    /** 
     * @brief  Check the reprojection error between 3D points
     * @note   
     * @param  src_observations: 
     * @param  dst_observations: 
     * @param  scale: scale
     * @param  R:  rotation matrix
     * @param  t: translation
     * @retval None
     */
    double CheckReprojError(const vector<Vector3d>& src_observations, 
                            const vector<Vector3d>& dst_observations, 
                            double scale,
                            Mat3 R,
                            Vector3d t );
    /** 
     * @brief  
     * @note   
     * @param  dPrecision: 
     * @param  count: 
     * @retval 
     */
    size_t BadTrackRejector(double dPrecision, size_t count);
    
    bool BundleAdjustment();

    void StoreOriginalSfMDatas();

    void MergeSfMDatas(string img_path);
};

}   // namespace i23dSFM

#endif