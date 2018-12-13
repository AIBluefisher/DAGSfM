
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "i23dSFM/sfm/sfm.hpp"
#include "i23dSFM/system/timer.hpp"
#include "i23dSFM/exif/exif_IO_EasyExif.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace i23dSFM;
using namespace i23dSFM::sfm;
using namespace i23dSFM::exif;

/// Export camera poses positions as a Vec3 vector
void GetCameraPositions(const SfM_Data & sfm_data, std::vector<Vec3> & vec_camPosition, std::vector<Vec3> & vec_camGPS)
{
  for (const auto & view : sfm_data.GetViews())
  {
    if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()))
    {
      const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.second.get());
      std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view.second.get()->s_Img_path);
      //to be commented, and add const to last sentence
      sView_filename = "/c" + sView_filename.substr(1);
      //end
      std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif());
      exifReader->open(sView_filename);
      Vec3 camGPS;
      camGPS[0] = exifReader->getLatitude();
      camGPS[1] = exifReader->getLongitude();
      camGPS[2] = exifReader->getAltitude();
      vec_camPosition.push_back(pose.center());
      vec_camGPS.push_back(camGPS);
    }
  }
}

/// Export camera frustrums as a triangle PLY file
int main(int argc, char **argv)
{
  using namespace std;
  std::cout << "Export camera frustums" << std::endl;

  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string sOutFile = "";
  std::string sGPSFile = "";

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('o', sOutFile, "output_file") );
  cmd.add( make_option('g', sGPSFile, "gps_file") );

  try {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] path to a SfM_Data scene\n"
    << "[-o|--output_file] PLY file to store the camera frustums as triangle meshes.\n"
    << "[-g|--gps_file] camera position with its GPS position from EXIF information.\n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // Assert that we can create the output directory/file
  if (!stlplus::folder_exists( stlplus::folder_part(sOutFile) ))
    if(!stlplus::folder_create( stlplus::folder_part(sOutFile) ))
      return EXIT_FAILURE;

  //pandagmz add : export camera coordinate with its position
  if (!sGPSFile.empty())
  {
    std::ofstream outfile(sGPSFile);
    std::vector<Vec3> vec_camPosition, vec_camGPS;
    GetCameraPositions(sfm_data, vec_camPosition, vec_camGPS);
    for(size_t i = 0; i < vec_camPosition.size(); i++)
    {
        outfile << std::setprecision(6) << vec_camPosition[i].transpose() << " " << std::setprecision(10) << vec_camGPS[i].transpose() << "\n";
    }
    outfile.close();
  }

  // If sfm_data have not structure, cameras are displayed as tiny normalized cones
  const Frustum_Filter frustum_filter(sfm_data);
  if (!sOutFile.empty())
  {
    if (frustum_filter.export_Ply(sOutFile))
      return EXIT_SUCCESS;
  }

  return EXIT_FAILURE;
}
