// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "i23dSFM/sfm/sfm.hpp"
#include "software/SfM/SfMPlyHelper.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <string>
#include <vector>
#include<fstream>
#include<Eigen/Geometry>
using namespace i23dSFM;
using namespace i23dSFM::image;
using namespace i23dSFM::sfm;

/// Find the color of the SfM_Data Landmarks/structure
bool ColorizeTracks(
        const SfM_Data & sfm_data,
        std::vector<Vec3> & vec_3dPoints,
        std::vector<Vec3> & vec_tracksColor)
{
    // Colorize each track
    //  Start with the most representative image
    //    and iterate to provide a color to each 3D point

    {
        C_Progress_display my_progress_bar(sfm_data.GetLandmarks().size(),
                                           std::cout,
                                           "\nCompute scene structure color\n");

        vec_tracksColor.resize(sfm_data.GetLandmarks().size());
        vec_3dPoints.resize(sfm_data.GetLandmarks().size());

        //Build a list of contiguous index for the trackIds
        std::map<IndexT, IndexT> trackIds_to_contiguousIndexes;
        IndexT cpt = 0;
        for (Landmarks::const_iterator it = sfm_data.GetLandmarks().begin();
             it != sfm_data.GetLandmarks().end(); ++it, ++cpt)
        {
            trackIds_to_contiguousIndexes[it->first] = cpt;
            vec_3dPoints[cpt] = it->second.X;
        }

        // The track list that will be colored (point removed during the process)
        std::set<IndexT> remainingTrackToColor;
        std::transform(sfm_data.GetLandmarks().begin(), sfm_data.GetLandmarks().end(),
                       std::inserter(remainingTrackToColor, remainingTrackToColor.begin()),
                       stl::RetrieveKey());

        while( !remainingTrackToColor.empty() )
        {
            // Find the most representative image (for the remaining 3D points)
            //  a. Count the number of observation per view for each 3Dpoint Index
            //  b. Sort to find the most representative view index

            std::map<IndexT, IndexT> map_IndexCardinal; // ViewId, Cardinal
            for (std::set<IndexT>::const_iterator
                         iterT = remainingTrackToColor.begin();
                 iterT != remainingTrackToColor.end();
                 ++iterT)
            {
                const size_t trackId = *iterT;
                const Observations & obs = sfm_data.GetLandmarks().at(trackId).obs;
                for( Observations::const_iterator iterObs = obs.begin();
                     iterObs != obs.end(); ++iterObs)
                {
                    const size_t viewId = iterObs->first;
                    if (map_IndexCardinal.find(viewId) == map_IndexCardinal.end())
                        map_IndexCardinal[viewId] = 1;
                    else
                        ++map_IndexCardinal[viewId];
                }
            }

            // Find the View index that is the most represented
            std::vector<IndexT> vec_cardinal;
            std::transform(map_IndexCardinal.begin(),
                           map_IndexCardinal.end(),
                           std::back_inserter(vec_cardinal),
                           stl::RetrieveValue());
            using namespace stl::indexed_sort;
            std::vector< sort_index_packet_descend< IndexT, IndexT> > packet_vec(vec_cardinal.size());
            sort_index_helper(packet_vec, &vec_cardinal[0], 1);

            // First image index with the most of occurence
            std::map<IndexT, IndexT>::const_iterator iterTT = map_IndexCardinal.begin();
            std::advance(iterTT, packet_vec[0].index);
            const size_t view_index = iterTT->first;
            const View * view = sfm_data.GetViews().at(view_index).get();
            const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path,
                                                                        view->s_Img_path);
            Image<RGBColor> image_rgb;
            Image<unsigned char> image_gray;
            const bool b_rgb_image = ReadImage(sView_filename.c_str(), &image_rgb);
            if (!b_rgb_image) //try Gray level
            {
                const bool b_gray_image = ReadImage(sView_filename.c_str(), &image_gray);
                if (!b_gray_image)
                {
                    std::cerr << "Cannot open provided the image." << std::endl;
                    return false;
                }
            }

            // Iterate through the remaining track to color
            // - look if the current view is present to color the track
            std::set<IndexT> set_toRemove;
            for (std::set<IndexT>::const_iterator
                         iterT = remainingTrackToColor.begin();
                 iterT != remainingTrackToColor.end();
                 ++iterT)
            {
                const IndexT trackId = *iterT;
                const Observations & obs = sfm_data.GetLandmarks().at(trackId).obs;
                Observations::const_iterator it = obs.find(view_index);

                if (it != obs.end())
                {
                    // Color the track
                    const Vec2 & pt = it->second.x;
                    const RGBColor color = b_rgb_image ? image_rgb(pt.y(), pt.x()) : RGBColor(image_gray(pt.y(), pt.x()));

                    vec_tracksColor[ trackIds_to_contiguousIndexes[trackId] ] = Vec3(color.r(), color.g(), color.b());
                    set_toRemove.insert(trackId);
                    ++my_progress_bar;
                }
            }
            // Remove colored track
            for (std::set<IndexT>::const_iterator iter = set_toRemove.begin();
                 iter != set_toRemove.end(); ++iter)
            {
                remainingTrackToColor.erase(*iter);
            }
        }
    }
    return true;
}
/// Export camera poses positions as a Vec3 vector
void GetCameraPositions(const SfM_Data & sfm_data, std::vector<Vec3> & vec_camPosition)
{
    for (const auto & view : sfm_data.GetViews())
    {
        if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()))
        {
            const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.second.get());
            vec_camPosition.push_back(pose.center());
        }
    }
}
class PhotoPose{
public :

    Vec3 center;
    Vec4 quad;
    Mat3 rotation;
    friend istream& operator>>(istream & in ,PhotoPose &pose){
        in>>pose.center[0]>>pose.center[1]>>pose.center[2];
        in>>pose.quad[0]>>pose.quad[1]>>pose.quad[2]>>pose.quad[3];
        Eigen::Quaterniond q4=Eigen::Quaterniond(pose.quad);
        pose.rotation=q4.toRotationMatrix();


        return in;
    }

};
// Convert from a SfM_Data format to another
int main(int argc, char **argv)
{
    CmdLine cmd;

    std::string
            sSfM_Data_Filename_In,
            sPhotoPly_Out,
            sMeshlab_Out;
    string pose_file;
    cmd.add(make_option('i', sSfM_Data_Filename_In, "input_file"));
    cmd.add(make_option('t', pose_file, "meshlab_view"));
    cmd.add(make_option('p', sPhotoPly_Out, "photoply_file"));
    cmd.add(make_option('m', sMeshlab_Out, "meshlab_view"));
    try {
        if (argc == 1) throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    } catch(const std::string& s) {
        std::cerr << "Usage: " << argv[0] << '\n'
                  << "[-i|--input_file] path to the input SfM_Data scene\n"
                  << "[-o|--output_file] path to the output PLY file\n"
                  << "[-p|--photoply_file] path to the photo PLY file\n"
                  << "[-m|--meshlab_view] if output the view file in meshlab\n"
                  << std::endl;

        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    // Load input SfM_Data scene
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename_In, ESfM_Data(ALL)))
    {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << sSfM_Data_Filename_In << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }

    ifstream fpose(pose_file);
    typedef Hash_Map<string,PhotoPose> PoseMap;
    PoseMap poseMap;
    string img_name;
    PhotoPose p;
    while(fpose>>img_name>>p){
        poseMap["/"+img_name]=p;
    }
    // pandagmz add : Display photos in .ply file
    if(!sPhotoPly_Out.empty())
    {
        std::cout << "Adding photo in ply file begins" << std::endl;
        std::vector<Vec3> cam_3dPoints, cam_Color, cam_Pos;
        const Intrinsics &intrinsics = sfm_data.GetIntrinsics();
        const Views views = sfm_data.GetViews();
        Views::const_iterator iterViews;
        for(iterViews = views.begin(); iterViews != views.end(); ++iterViews)
        {
            View *view = iterViews->second.get();
            if(!sfm_data.IsPoseAndIntrinsicDefined(view))
                continue;
            std::shared_ptr<i23dSFM::cameras::IntrinsicBase> intrinsic = intrinsics.at(view->id_intrinsic);
            const std::vector<double> intrinsicsParams = intrinsic->getParams();
            //intrinsicsParams 0:focal length 1:ppx 2:ppy 3:k1 4:k2 5:k3
            double ppx, ppy, focal_length, small_rate = 0.1, down_sample = 32;
            //small_rate : decide the size of photo in ply file
            //down_sample : decide the downsampling of drawing photos in ply file
            focal_length = intrinsicsParams[0];
            ppx = intrinsicsParams[1];
            ppy = intrinsicsParams[2];
            const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path,view->s_Img_path);
			if(poseMap.find(view->s_Img_path)==poseMap.end()){
				continue;
			}
			PhotoPose quadp=poseMap[view->s_Img_path];

            cout<<view->s_Img_path<<endl;
            cout<<quadp.center<<endl<<quadp.rotation<<endl;
            const geometry::Pose3 pose(quadp.rotation,quadp.center);
            Image<RGBColor> image_rgb;
            cout << "Load image : " << sView_filename << endl;
            ReadImage(sView_filename.c_str(), &image_rgb);
            int x, y;
            for(x = 0; x < image_rgb.Height(); x+=down_sample)
                for(y = 0; y < image_rgb.Width(); y+=down_sample)
                {
                    Vec3 pixel_pos((y - ppx) * small_rate / focal_length, (x - ppy) * small_rate / focal_length, small_rate), pixel_color(image_rgb(x, y).r(), image_rgb(x, y).g(), image_rgb(x, y).b());
                    cam_3dPoints.emplace_back(pose.rotation().transpose() * pixel_pos + pose.center());
                    cam_Color.push_back(pixel_color);
                }

        }
        plyHelper::exportToPly(cam_3dPoints, cam_Pos, sPhotoPly_Out, &cam_Color);
        std::cout << "Write ply file successfully" << std::endl;
    }


}
