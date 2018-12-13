
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <cstdlib>

#include "i23dSFM/sfm/sfm.hpp"
#include "i23dSFM/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#define FOCAL_DIFF_THRESHOLD 0.04

using namespace i23dSFM;
using namespace i23dSFM::cameras;
using namespace i23dSFM::sfm;

/// From 2 given image file-names, find the two corresponding index in the View list
bool computeIndexFromImageNames(
  const SfM_Data & sfm_data,
  const std::pair<std::string,std::string>& initialPairName,
  Pair& initialPairIndex)
{
  if (initialPairName.first == initialPairName.second)
  {
    std::cerr << "\nInvalid image names. You cannot use the same image to initialize a pair." << std::endl;
    return false;
  }

  initialPairIndex = Pair(UndefinedIndexT, UndefinedIndexT);

  /// List views filenames and find the one that correspond to the user ones:
  std::vector<std::string> vec_camImageName;
  for (Views::const_iterator it = sfm_data.GetViews().begin();
    it != sfm_data.GetViews().end(); ++it)
  {
    const View * v = it->second.get();
    const std::string filename = stlplus::filename_part(v->s_Img_path);
    if (filename == initialPairName.first)
    {
      initialPairIndex.first = v->id_view;
    }
    else{
      if (filename == initialPairName.second)
      {
        initialPairIndex.second = v->id_view;
      }
    }
  }
  return (initialPairIndex.first != UndefinedIndexT &&
      initialPairIndex.second != UndefinedIndexT);
}


int main(int argc, char **argv)
{
  using namespace std;
  std::cout << "Hybrid(incremental + global) reconstruction" << std::endl
            << " Perform incremental SfM (Initial Pair Essential + Resection)." << std::endl
            << std::endl;

  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string sMatchesDir;
  std::string sOutDir = "";
  int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
  int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);
  std::pair<std::string,std::string> initialPairString("","");
  bool bRefineIntrinsics = true;
  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;
  bool bRepeatFocal = false;

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('m', sMatchesDir, "matchdir") );
  cmd.add( make_option('o', sOutDir, "outdir") );
  cmd.add( make_option('a', initialPairString.first, "initialPairA") );
  cmd.add( make_option('b', initialPairString.second, "initialPairB") );
  cmd.add( make_option('c', i_User_camera_model, "camera_model") );
  cmd.add( make_option('r', iRotationAveragingMethod, "rotationAveraging") );
  cmd.add( make_option('t', iTranslationAveragingMethod, "translationAveraging") );
  cmd.add( make_option('f', bRefineIntrinsics, "refineIntrinsics") );
  cmd.add( make_option('r', bRepeatFocal, "repeatFocal") );

  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] path to a SfM_Data scene\n"
    << "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
    << "[-o|--outdir] path where the output data will be stored\n"
    << "[-a|--initialPairA] filename of the first image (without path)\n"
    << "[-b|--initialPairB] filename of the second image (without path)\n"
    << "[-c|--camera_model] Camera model type for view with unknown intrinsic:\n"
      << "\t 1: Pinhole \n"
      << "\t 2: Pinhole radial 1\n"
      << "\t 3: Pinhole radial 3 (default)\n"
    << "[-f|--refineIntrinsics] \n"
    << "\t 0-> intrinsic parameters are kept as constant\n"
    << "\t 1-> refine intrinsic parameters (default). \n"
    << "[-r|--repeatFocal] \n"
    << "\t 0-> calculate focal length only once (default). \n"
    << "\t 1-> repeat refine focal length. \n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
      iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
    std::cerr << "\n Rotation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
      iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
    std::cerr << "\n Translation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace i23dSFM::features;
  const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type)
  {
    std::cerr << "Invalid: "
      << sImage_describer << " regions type file." << std::endl;
    return EXIT_FAILURE;
  }

  // Features reading
  std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }
  // Matches reading
  std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
  if (!matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.txt"))) {
    std::cerr << std::endl
      << "Invalid matches file." << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutDir.empty())  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create(sOutDir);

  //---------------------------------------
  // Hybrid reconstruction process
  //---------------------------------------

  double intrinsicError = 1.0;
  //Intrinsics intrinsics = sfm_data.GetIntrinsics();
  //Intrinsics::iterator iterIntrinsics;
  //std::cout << "gmz debug intrinsics size = " << intrinsics.size() << std::endl;
  //Views views = sfm_data.GetViews();
  //Views::iterator iterViews;
  //for(iterViews = views.begin(); iterViews != views.end(); iterViews++)
  //    std::cout << "intrinsic id : " << iterViews->second.get()->id_intrinsic << std::endl;
  //for(iterIntrinsics = intrinsics.begin(); iterIntrinsics != intrinsics.end(); iterIntrinsics++)
  //    std::cout << "gmz debug intrinsics : " << iterIntrinsics->second.get()->getParams()[0] << std::endl;
  SfM_Data sfm_data_backup = sfm_data;
  //Intrinsics intrinsics = sfm_data.intrinsics;
  Intrinsics::iterator iterIntrinsics;
  vector<double> vec_focal;
  int iVec;
  for(iterIntrinsics = sfm_data.intrinsics.begin(); iterIntrinsics != sfm_data.intrinsics.end(); iterIntrinsics++)
      vec_focal.push_back(iterIntrinsics->second.get()->getParams()[0]);

  i23dSFM::system::Timer timer;
  while(intrinsicError > FOCAL_DIFF_THRESHOLD) {
  sfm_data = sfm_data_backup;
  //sfm_data.intrinsics = intrinsics;
  iVec = 0;
  for(iterIntrinsics = sfm_data.intrinsics.begin(); iterIntrinsics != sfm_data.intrinsics.end(); iterIntrinsics++)
  {
      iterIntrinsics->second.get()->getParams()[0] = vec_focal[iVec];
      std::cout << "current focal length : " << vec_focal[iVec] << endl;
      iVec++;
  }

  HybridSfMReconstructionEngine sfmEngine(
    sfm_data,
    sOutDir,
    stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

  // Configure the features_provider & the matches_provider
  sfmEngine.SetFeaturesProvider(feats_provider.get());
  sfmEngine.SetMatchesProvider(matches_provider.get());

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(!bRefineIntrinsics);
  sfmEngine.SetUnknownCameraType(EINTRINSIC(i_User_camera_model));

    // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(
    ERotationAveragingMethod(iRotationAveragingMethod));
  sfmEngine.SetTranslationAveragingMethod(
    ETranslationAveragingMethod(iTranslationAveragingMethod));

  // Handle Initial pair parameter
  if (!initialPairString.first.empty() && !initialPairString.second.empty())
  {
    Pair initialPairIndex;
    if(!computeIndexFromImageNames(sfm_data, initialPairString, initialPairIndex))
    {
        std::cerr << "Could not find the initial pairs <" << initialPairString.first
          <<  ", " << initialPairString.second << ">!\n";
      return EXIT_FAILURE;
    }
    sfmEngine.setInitialPair(initialPairIndex);
  }
  bool sfmEngineProcess = sfmEngine.Process();
  intrinsicError = 0.0;
  iVec = 0;
  for(iterIntrinsics = sfm_data.intrinsics.begin(); iterIntrinsics != sfm_data.intrinsics.end(); iterIntrinsics++)
  {
      std::cout << "focal length from " << vec_focal[iVec] << " to " << iterIntrinsics->second.get()->getParams()[0] << endl;
      intrinsicError = std::max(intrinsicError, std::abs((iterIntrinsics->second.get()->getParams()[0] - vec_focal[iVec]) / vec_focal[iVec]));
      vec_focal[iVec] = iterIntrinsics->second.get()->getParams()[0];
      iVec++;
  }
  std::cout << "current focal error = " << intrinsicError << endl;
  if(bRepeatFocal && (intrinsicError > FOCAL_DIFF_THRESHOLD))
      continue;

  if (sfmEngineProcess)
  {
    std::cout << std::endl << " Total Ac-Sfm took (s): " << timer.elapsed() << std::endl;

    std::cout << "...Generating SfM_Report.html" << std::endl;
    Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

    //-- Export to disk computed scene (data & visualizable results)
    std::cout << "...Export SfM_Data to disk." << std::endl;
    Save(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(sOutDir, "sfm_data", ".json"),
      ESfM_Data(ALL));

    Save(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
      ESfM_Data(ALL));

    return EXIT_SUCCESS;
  }
  }
  return EXIT_FAILURE;
}
