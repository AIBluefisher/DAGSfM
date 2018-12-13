/*
Copyright (c) 2018, Yu Chen
All rights reserved.
*/

#include <fstream>
#include <utility>
#include <limits>

#include "i23dSFM/sfm/sfm.hpp"
#include "i23dSFM/geometry/rigid_transformation3D_srt.hpp"
#include "i23dSFM/sfm/sfm_data_BA_ceres.hpp"
#include "i23dSFM/sfm/sfm_data_filters.hpp"
#include "software/SfM/SfMPlyHelper.hpp"
#include "omp.h"

#include "SfMAligner.hpp"
#include "RansacSimilarity.hpp"

using namespace std;

// #define USE_OBSERVATION
#define USE_CAMERA
#define USE_RANSAC

namespace i23dSFM {

SfMAligner::SfMAligner()
{

}

SfMAligner::~SfMAligner()
{

}

bool SfMAligner::InitializeGraph(std::string filename)
{
    ifstream file_in(filename);
    int idx = 0;
    string path;
    
    if(!file_in.is_open()) {
        cout << "file " << filename << "cannot be opened! Please check the path!\n";
        return false;
    }

    // Add nodes
    while(file_in >> path) {
        if (stlplus::file_exists(path)) {
            ImageNode img_node(idx++, path);
            _cluster_graph.AddNode(img_node);
        }
        else {
            std::cout << "file " << path << "doesn't exists\n";
        }
    }
    file_in.close();

    // Add edges
    vector<ImageNode> nodes = _cluster_graph.GetImageNode();
    #pragma omp parallel 
    {
        #pragma omp for
        for(int i = 0; i < nodes.size(); i++) {
            string sd_path1 = nodes[i].sfm_data_path;
            for(int j = i + 1; j < nodes.size(); j++) {
                string sd_path2 = nodes[j].sfm_data_path;
                LinkEdge edge(i, j, 0);
                this->ConstructEdge(sd_path1, sd_path2, edge);
                if(edge.score != numeric_limits<float>::max()) {
                    _cluster_graph.AddEdge(edge);
                 }
            }
        }
    }
    return true;
}

void SfMAligner::ConstructEdge(string path1, string path2, LinkEdge& edge)
{
    SfM_Data sfm_data1, sfm_data2;
    if(!Load(sfm_data1, path1, ESfM_Data(ALL))) {
        cout << "The input SfM_Data file \"" << path1 << "\" cannot be read." << endl;
        return;
    }
    if(!Load(sfm_data2, path2, ESfM_Data(ALL))) {
        cout << "The input SfM_Data file \"" << path1 << "\" cannot be read." << endl;
        return;
    }

    Mat3 R = Mat3::Identity(3, 3);
    Vector3d t = Vector3d::Zero();
    double s = 0.0;
    double msd = 0.0;

#ifdef USE_CAMERA
    FindCommonCameras(sfm_data1, sfm_data2, edge.src_poses, edge.dst_poses);
    if(!edge.src_poses.size()) edge.score = numeric_limits<float>::max();
    else {
        // edge.score = 1.0 / (float)edge.src_poses.size();
        vector<Vector3d> src_cameras = edge.GetSrcCameraCenter();
        vector<Vector3d> dst_cameras = edge.GetDstCameraCenter();
        FindSimilarityTransform(src_cameras, dst_cameras, R, t, s, msd);
        edge.R = R; edge.t = t; edge.scale = s;
        // TODO: Test msd with msd = (msd1 + msd2) / 2
        // msd = sqrt(msd1 * msd2); 
        edge.score = msd;
    }
#endif

#ifdef USE_OBSERVATION
    FindCommonObservations(sfm_data1, sfm_data2, edge.src_observations, edge.dst_observations);
    if (!edge.src_observations.size()) edge.score = numeric_limits<float>::max();
    else {
        // edge.score = 1.0 / (float)edge.src_observations.size();
        vector<Vector3d> src_observations = edge.src_observations;
        vector<Vector3d> dst_observations = edge.dst_observations;
        FindSimilarityTransform(src_observations, dst_observations, R, t, s, msd);
        edge.R = R; edge.t = t; edge.scale = s;
        edge.score = msd;
    }
#endif
    
}


void SfMAligner::UpdateGraph()
{
    cout << "\nConstruct Minimum Spanning Tree\n";
    vector<LinkEdge> edges = _cluster_graph.Kruskal();
    cout << "Construct Minimum Spanning Tree complete\n\n";

    // Remove edges that are not exists in MST
    vector<EdgeMap> adj_maps = _cluster_graph.GetEdgeMap();
    #pragma omp parallel
    {
        #pragma omp for
        for(int i = 0; i < adj_maps.size(); i++) {
            for(auto it = adj_maps[i].begin(); it != adj_maps[i].end(); it++) {
                LinkEdge le1(it->second.src, it->second.dst);
                LinkEdge le2(it->second.dst, it->second.src);
                if(find(edges.begin(), edges.end(), le1) == edges.end() &&
                   find(edges.begin(), edges.end(), le2) == edges.end()) {
                    cout << "Remove edge: " << it->second.src << ", " << it->second.dst << endl;
                    _cluster_graph.RemoveEdge(it->second.src, it->second.dst);
                    _cluster_graph.RemoveEdge(it->second.dst, it->second.src);
                }
            }
        }
    }
    _cluster_graph.ShowInfo();
}

void SfMAligner::MergeClusters(string dir, string img_path)
{
    _paths.resize(_cluster_graph.GetNodeSize());
    _sim_paths.resize(_cluster_graph.GetNodeSize());
    this->StoreOriginalSfMDatas();

    // Merge nodes until only one node left
    while(_cluster_graph.GetNodeSize() > 1) {
        unordered_map<int, int> degree = _cluster_graph.ComputeDegree();
        int idx = _cluster_graph.FindLeafNode(degree);
        LinkEdge edge = _cluster_graph.FindConnectedEdge(idx);

        cout << "Finding nodes with [degree = 1] complete\n";
        cout << "Node found: " << idx << endl;
        cout << edge.src << "->" << edge.dst << ": " << edge.score << endl;

    #ifdef USE_OBSERVATION
        vector<Vector3d> src_observations = 
                (idx == edge.src) ? edge.src_observations : edge.dst_observations;
        vector<Vector3d> dst_observations = 
                (idx == edge.src) ? edge.dst_observations : edge.src_observations;
        cout << "observation size: " << src_observations.size() << endl;
    #endif
    #ifdef USE_CAMERA
        vector<Vector3d> src_cameras = 
                (idx == edge.src) ? edge.GetSrcCameraCenter() : edge.GetDstCameraCenter();
        vector<Vector3d> dst_cameras = 
                (idx == edge.src) ? edge.GetDstCameraCenter() : edge.GetSrcCameraCenter();
    #endif

        // src is the node with degree equals 1
        int src = (idx == edge.src) ? edge.src : edge.dst;
        int dst = (idx == edge.src) ? edge.dst : edge.src;

        Mat3 R = Mat3::Identity(3, 3);
        Vector3d t = Vector3d::Zero();
        double s = 1.0;
        double msd = 0.0;
        
        cout << "Merge Clusters: " << src << "->" << dst
             << ": " << edge.score << endl;

    #ifdef USE_OBSERVATION
        FindSimilarityTransform(src_observations, dst_observations, R, t, s, msd);
    #endif
    #ifdef USE_CAMERA
        FindSimilarityTransform(src_cameras, dst_cameras, R, t, s, msd);
    #endif

        _last_idx = dst;
        Sim3 sim(R, t, s);
        _paths[src].insert(make_pair(dst, sim));

        _cluster_graph.DeleteNode(src);
        _cluster_graph.RemoveEdge(src, dst);
        _cluster_graph.RemoveEdge(dst, src);
        _cluster_graph.ShowInfo();
        cout << "\n\n";
    }

    for (int i = 0; i < _ori_sfmdatas.size(); i++) {
        if (i != _last_idx) ComputePath(i, _last_idx);
    }

    this->MergeSfMDatas(img_path);

    std::vector<Vec3> colors = this->GenerateColors(_ori_sfmdatas.size());
    this->RenderClusters(colors);
    this->RenderCluster(_last_idx, colors);

    cout << "Generating SfM Reconstruction Report...\n";
    Generate_SfM_Report(_sfm_data, 
                        stlplus::create_filespec(dir, "SfMReconstruction_Report.html"));
}

double SfMAligner::CheckReprojError(const vector<Vector3d>& src_observations, 
                                    const vector<Vector3d>& dst_observations,
                                    double scale, 
                                    Mat3 R,
                                    Vector3d t)
{
    double reproj_err = 0.0;
    int size = src_observations.size();
    for(int i = 0; i < size; i++) {
        Vector3d reproj_obv = scale * R * src_observations[i] + t;
        reproj_err += (reproj_obv - dst_observations[i]).norm();
    }
    cout << "Sum Reprojection Error: " << reproj_err
         << "\nPoints Size: " << src_observations.size()
         << "\nMean Reprojection Error: " 
         << (size ? (reproj_err / size) : numeric_limits<float>::max()) 
         << endl;
    return size ? (reproj_err / size) : numeric_limits<float>::max();
}

void SfMAligner::FindCommonCameras(const SfM_Data& sfm_data1, 
                                    const SfM_Data& sfm_data2,
                                    std::vector<pair<int, geometry::Pose3>>& poses1,
                                    std::vector<pair<int, geometry::Pose3>>& poses2)
{
    Views views1 = sfm_data1.GetViews(), views2 = sfm_data2.GetViews();
    Poses pose_list1 = sfm_data1.GetPoses(), pose_list2 = sfm_data2.GetPoses();

    for (Poses::iterator it1 = pose_list1.begin(); it1 != pose_list1.end(); it1++) {
        Poses::iterator it2 = pose_list2.find(it1->first);
        if (it2 != pose_list2.end()) {
            poses1.push_back(make_pair(it1->first, it1->second));
            poses2.push_back(make_pair(it2->first, it2->second));
            cout << "[" << poses1.size() << "]: " << it1->first << " - " << it2->first << endl;
        }
    }
    cout << "end of find common cameras\n";
}

void SfMAligner::FindCommonObservations(const SfM_Data& sfm_data1, 
                                        const SfM_Data& sfm_data2,
                                        std::vector<Vector3d>& observations1,
                                        std::vector<Vector3d>& observations2)
{
    // Find putative common 3D points by global track

    // std::vector<int> common_poseidx;
    // Poses poses1 = sfm_data1.GetPoses(), poses2 = sfm_data2.GetPoses();
    // for (auto it1 = poses1.begin(); it1 != poses1.end(); it1++) {
    //     auto it2 = poses2.find(it1->first);
    //     if (it2 != poses2.end()) { common_poseidx.push_back(it1->first); }
    // }

    // Landmarks inview_landmarks1, inview_landmarks2;
    Landmarks landmarks1 = sfm_data1.GetLandmarks(), landmarks2 = sfm_data2.GetLandmarks();
    // Find tracks that observed by the same view
    // for (auto poseidx : common_poseidx) {
    //     for (auto it1 = landmarks1.begin(); it1 != landmarks1.end(); it1++) {
    //         Observations obs1 = it1->second.obs;
    //         if (obs1.find(poseidx) != obs1.end()) { inview_landmarks1[it1->first] = it1->second; }
    //     }
    //     for (auto it2 = landmarks2.begin(); it2 != landmarks2.end(); it2++) {
    //         Observations obs2 = it2->second.obs;
    //         if (obs2.find(poseidx) != obs2.end()) { inview_landmarks2[it2->first] = it2->second; }
    //     }
    // }

    // // Find common 3D points
    // for (auto it1 = inview_landmarks1.begin(); it1 != inview_landmarks1.end(); it1++) {
    //     int trackid = it1->first;
    //     auto it2 = inview_landmarks2.find(trackid);
    //     if (it2 != inview_landmarks2.end()) {
    //         observations1.push_back(it1->second.X);
    //         observations2.push_back(it2->second.X);
    //         // cout << it1->first << " - " << it2->first << endl;
    //     }
    // }

    for (auto it1 = landmarks1.begin(); it1 != landmarks1.end(); it1++) {
        int trackid = it1->first;
        auto it2 = landmarks2.find(trackid);
        if (it2 != landmarks2.end()) {
            observations1.push_back(it1->second.X);
            observations2.push_back(it2->second.X);
        }
    }
    cout << "common observations size: " << observations1.size() << endl;
}

void SfMAligner::FindSimilarityTransform(const std::vector<Vector3d>& observations1,
                                         const std::vector<Vector3d>& observations2,
                                         Mat3& R,
                                         Vector3d& t,
                                         double& scale,
                                         double& msd) 
{
    std::vector<Vector3d> inliers1, inliers2;
#ifdef USE_RANSAC
    double threshold = 0.001;
    double p = 0.99;
    if (observations1.size() > 5) {
        cout << "Finding Similarity by RANSAC\n";
        RansacSimilarity(observations1, observations2, inliers1, inliers2, R, t, scale, threshold, p);
        cout << "inliers size: " << inliers1.size() << endl;
        // Re-compute similarity by inliers
        MatrixXd x1 = MatrixXd::Zero(3, inliers1.size()), x2 = MatrixXd::Zero(3, inliers2.size());
        for(int i = 0; i < inliers1.size(); i++) {
            x1.col(i) = inliers1[i];
            x2.col(i) = inliers2[i];
        }
        i23dSFM::geometry::FindRTS(x1, x2, &scale, &t, &R);
        // Optional non-linear refinement of the found parameters
        i23dSFM::geometry::Refine_RTS(x1, x2, &scale, &t, &R);
    }
#endif

    MatrixXd x1 = MatrixXd::Zero(3, observations1.size()), x2 = MatrixXd::Zero(3, observations2.size());
    for (int i = 0; i < observations1.size(); i++) {
        x1.col(i) = observations1[i];
        x2.col(i) = observations2[i];
    }

#ifdef USE_RANSAC
    if (observations1.size() <= 5 || inliers1.size() <= 5) {
        i23dSFM::geometry::FindRTS(x1, x2, &scale, &t, &R);
        i23dSFM::geometry::Refine_RTS(x1, x2, &scale, &t, &R);
    }
#else
    if (!i23dSFM::geometry::FindRTS(x1, x2, &scale, &t, &R)) {
        cout << "[WARNING]: Degenerate case when finding similarity transformation!\n";
    }
    // Optional non-linear refinement of the found parameters
    i23dSFM::geometry::Refine_RTS(x1, x2, &scale, &t, &R);
#endif
    
    cout << "scale: " << scale << endl;
    cout << "rotation: \n" << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << "\n"
                            << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << "\n"
                            << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << endl;
    cout << "translation: " << t[0] << ", " << t[1] << ", " << t[2] << endl; 

#ifdef USE_RANSAC
    msd = CheckReprojError(inliers1, inliers2, scale, R, t);
#else
    msd = CheckReprojError(observations1, observations2, scale, R, t);
#endif
}

bool SfMAligner::Refine()
{
    do {
        BundleAdjustment();
    } while(BadTrackRejector(4.0, 50) != 0);

    // Ensure there is no remaining outliers
    BadTrackRejector(4.0, 0);
    return true;
}

ClusterGraph SfMAligner::GetClusterGraph() const
{
    return _cluster_graph;
}

size_t SfMAligner::BadTrackRejector(double dPrecision, size_t count) {
    const size_t nbOutLiers_residualErr = RemoveOutliers_PixelResidualError(_sfm_data, dPrecision, 2);
    const size_t nbOutliers_angleErr = RemoveOutliers_AngleError(_sfm_data, 2.0);

    return (nbOutLiers_residualErr + nbOutliers_angleErr) > count;
}

bool SfMAligner::BundleAdjustment()
{
    Bundle_Adjustment_Ceres::BA_options options;
    if(_sfm_data.GetPoses().size() > 100) {
        options._preconditioner_type = ceres::JACOBI;
        options._linear_solver_type = ceres::SPARSE_SCHUR;
    }

    else {
        options._linear_solver_type = ceres::DENSE_SCHUR;
    }
    Bundle_Adjustment_Ceres bundle_adjustment_obj(options);
    return bundle_adjustment_obj.Adjust(_sfm_data, true, true, true, false);
}

void SfMAligner::StoreOriginalSfMDatas()
{
    for (int i = 0; i < _cluster_graph.GetNodeSize(); i++) {
        _ori_sfmdatas.push_back(_cluster_graph.GetImageNode()[i].sfm_data_path);
    }
}

void SfMAligner::ComputePath(int src, int dst)
{
    std::cout << "Computing Path: " << src << "->" << dst << std::endl;
    std::queue<int> qu;
    qu.push(src);
    Eigen::Matrix4d sim3 = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    double s = 1.0;

    Sim3 sim(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Identity(), 1.0);
    std::cout << "v: " << src << std::endl;
    while (!qu.empty()) {
        int u = qu.front(); qu.pop();
        auto it = _paths[u].begin();
        int v = it->first;
        std::cout << "v: " << v << std::endl;
        s = it->second.s * s;
        r = it->second.R * r.eval();
        t = it->second.s * it->second.R * t.eval() + it->second.t; 
        if (v == dst) {
            sim.s = s; sim.R = r; sim.t = t;
            _sim_paths[src] = sim;
            return;
        }
        else qu.push(v);
    }
    std::cout << "\n";
}

void SfMAligner::TransformPly(const string& path1, 
                              const string& path2, 
                              const Mat3& R, 
                              const Vector3d& t, 
                              const double& scale) const
{
    vector<Vec3> vec_points, points_colors;
    plyHelper::readPly(vec_points, path1, points_colors);

    for(int i = 0; i < vec_points.size(); i++) {
        vec_points[i] = scale * R * vec_points[i].eval() + t;
    }
    plyHelper::exportToPly(vec_points, &points_colors, path2.c_str());
}

void SfMAligner::TransformPly(const string& path1, 
                              const string& path2, 
                              const Mat3& R, 
                              const Vector3d& t, 
                              const double& scale,
                              const Vec3& color) const
{
    vector<Vec3> vec_points, points_colors;
    plyHelper::readPly(vec_points, path1, points_colors);
    vector<Vec3>().swap(points_colors);

    for(int i = 0; i < vec_points.size(); i++) {
        vec_points[i] = scale * R * vec_points[i].eval() + t;
        points_colors.push_back(color);
    }
    plyHelper::exportToPly(vec_points, &points_colors, path2.c_str());
}

std::vector<Vec3> SfMAligner::GenerateColors(int size) const
{
    std::vector<Vec3> colors;
    srand((unsigned int)time(NULL));
    for (int i = 0; i < size; i++) {
        double r = (double)(rand() % 256);
        double g = (double)(rand() % 256);
        double b = (double)(rand() % 256);
        colors.push_back(Vec3(r, g, b));
    }
    return colors;
}

void SfMAligner::RenderClusters(const std::vector<Vec3>& colors) const
{
    for (int i = 0; i < _ori_sfmdatas.size(); i++) {
        if (i != _last_idx) { this->RenderCluster(i, colors); }
    }
}

void SfMAligner::RenderCluster(const int idx,
                               const std::vector<Vec3>& colors) const
{
    std::string dir = stlplus::folder_part(_ori_sfmdatas[idx]);
    std::cout << dir << std::endl;
    Mat3 R = _sim_paths[idx].R;
    Vec3 t = _sim_paths[idx].t;
    double s = _sim_paths[idx].s;
    std::string ori_ply_path = dir + "/robust.ply";
    std::string trans_ply_path = dir + "/robust_transformed.ply";
    this->TransformPly(ori_ply_path, trans_ply_path, R, t, s, colors[idx]);
}

void SfMAligner::MergeSfMDatas(string img_path)
{
    if(!Load(_sfm_data, _ori_sfmdatas[_last_idx], ESfM_Data(ALL))) {
        cout << "The input SfM_Data file \"" 
             << _ori_sfmdatas[_last_idx] 
             << "\" cannot be read." << endl;
        return;
    }

    _sfm_data.s_root_path = img_path;
    Views& views = _sfm_data.views;
    Poses& poses = _sfm_data.poses;
    Intrinsics& intrinsics = _sfm_data.intrinsics;
    Landmarks& structure = _sfm_data.structure;
    Landmarks& control_points = _sfm_data.control_points;

    for (int i = 0; i < _ori_sfmdatas.size(); i++) {
        if (i == _last_idx) continue;
        SfM_Data sfm_data;
        Mat3 R = _sim_paths[i].R;
        Vec3 t = _sim_paths[i].t;
        double scale = _sim_paths[i].s;
        if (Load(sfm_data, _ori_sfmdatas[i], ESfM_Data(ALL))) {
            // Copy views
            for(auto vi = sfm_data.views.begin(); vi != sfm_data.views.end(); vi++) {
                views.insert(make_pair(vi->first, vi->second));
            }
            // TODO: copy intrinsic data from sfm_data

            // Copy control_points
            IndexT cp_s_id = control_points.size();
            for(auto ci = sfm_data.control_points.begin(); ci != sfm_data.control_points.end(); ci++) {
                control_points.insert(make_pair(cp_s_id++, ci->second));
            }
            // Transform camera poses
            for(auto pi = sfm_data.poses.begin(); pi != sfm_data.poses.end(); pi++) {
                // TODO: Do not transform the pose that has same pose_id with sfm_data2
                Vec3 center = scale * R * pi->second.center() + t;
                Mat3 rotation = pi->second.rotation();
                poses.insert(make_pair(pi->first, geometry::Pose3(rotation, center)));
            }
            // Transform structure
            IndexT ss_id = structure.size();
            for(auto si= sfm_data.structure.begin(); si != sfm_data.structure.end(); si++) {
                size_t trackId = si->first;
                // if (structure.find(trackId) != structure.end()) continue;
                Landmark landmark = si->second;
                landmark.X = scale * R * landmark.X.eval() + t;
                structure.insert(make_pair(ss_id++, landmark));
                // structure.insert(make_pair(trackId, landmark));
            }
        }
    }

    // cout << "Performing Global Bundle Adjustment\n";
    // Refine();
    
    string path = img_path + "/final_sfm_data.json";
    std::cout << "Saving final sfm data " << path << std::endl;
    if (Save(_sfm_data, path, ESfM_Data(ALL))) {
        std::cout << "sfm data successfully saved in " << path << std::endl;
    }
    else {
        std::cout << "Save sfm data " << path << std::endl;
    }
}

}   // namespace i23dSFM