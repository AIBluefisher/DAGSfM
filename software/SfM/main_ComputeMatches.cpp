

// Copyright (c) 2012, 2013 Pierre MOULON.
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
// Modified: Chenyu

#include "i23dSFM/sfm/sfm_data.hpp"
#include "i23dSFM/sfm/sfm_data_io.hpp"
#include "i23dSFM/sfm/pipelines/sfm_engine.hpp"
#include "i23dSFM/sfm/pipelines/sfm_features_provider.hpp"
#include "i23dSFM/sfm/pipelines/sfm_regions_provider.hpp"

/// Generic Image Collection image matching
#include "i23dSFM/matching_image_collection/Matcher_Regions_AllInMemory.hpp"
#include "i23dSFM/matching_image_collection/Cascade_Hashing_Matcher_Regions_AllInMemory.hpp"
#include "i23dSFM/matching_image_collection/GeometricFilter.hpp"
#include "i23dSFM/matching_image_collection/F_ACRobust.hpp"
#include "i23dSFM/matching_image_collection/E_ACRobust.hpp"
#include "i23dSFM/matching_image_collection/H_ACRobust.hpp"
#include "i23dSFM/matching/pairwiseAdjacencyDisplay.hpp"
#include "i23dSFM/matching/indMatch_utils.hpp"
#include "i23dSFM/system/timer.hpp"
#include "third_party/gms/gms_matcher.h"
#include "i23dSFM/graph/graph.hpp"
#include "i23dSFM/stl/stl.hpp"
#include "i23dSFM/tracks/tracks.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include <omp.h>
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <fstream>

using namespace i23dSFM;
using namespace i23dSFM::cameras;
using namespace i23dSFM::matching;
using namespace i23dSFM::tracks;
using namespace i23dSFM::robust;
using namespace i23dSFM::sfm;
using namespace i23dSFM::matching_image_collection;
using namespace std;


enum EGeometricModel {
    FUNDAMENTAL_MATRIX = 0,
    ESSENTIAL_MATRIX = 1,
    HOMOGRAPHY_MATRIX = 2
};


enum EPairMode {
    PAIR_EXHAUSTIVE = 0,
    PAIR_CONTIGUOUS = 1,
    PAIR_FROM_FILE = 2
};


void ExportTracks(const PairWiseMatches& map_matches, std::string dir)
{
    tracks::TracksBuilder tracksBuilder;
    tracks::STLMAPTracks map_tracks;
    std::cout << "\n" << "Track building" << std::endl;

    tracksBuilder.Build(map_matches);
    std::cout << "\n" << "Track filtering" << std::endl;
    tracksBuilder.Filter();
    std::cout << "\n" << "Track filtering : min occurence" << std::endl;
    tracksBuilder.FilterPairWiseMinimumMatches(20);
    std::cout << "\n" << "Track export to internal struct" << std::endl;
    //-- Build tracks with STL compliant type :
    tracksBuilder.ExportToSTL(map_tracks);

    std::cout << "\n" << "Track stats" << std::endl;
    {
      std::ostringstream osTrack;
      //-- Display stats :
      //    - number of images
      //    - number of tracks
      std::set<size_t> set_imagesId;
      tracks::TracksUtilsMap::ImageIdInTracks(map_tracks, set_imagesId);
      osTrack << "------------------" << "\n"
        << "-- Tracks Stats --" << "\n"
        << " Tracks number: " << tracksBuilder.NbTracks() << "\n"
        << " Images Id: " << "\n";
      std::copy(set_imagesId.begin(),
        set_imagesId.end(),
        std::ostream_iterator<size_t>(osTrack, ", "));
      osTrack << "\n------------------" << "\n";

      std::map<size_t, size_t> map_Occurence_TrackLength;
      tracks::TracksUtilsMap::TracksLength(map_tracks, map_Occurence_TrackLength);
      osTrack << "TrackLength, Occurrence" << "\n";
      for (std::map<size_t, size_t>::const_iterator iter = map_Occurence_TrackLength.begin();
        iter != map_Occurence_TrackLength.end(); ++iter)  {
        osTrack << "\t" << iter->first << "\t" << iter->second << "\n";
      }
      osTrack << "\n";
      std::cout << osTrack.str();
  }

  string filename = dir + "/tracks.txt";
  tracksBuilder.ExportToFile(filename, map_tracks);
}

/// Compute corresponding features between a series of views:
/// - Load view images description (regions: features & descriptors)
/// - Compute putative local feature matches (descriptors matching)
/// - Compute geometric coherent feature matches (robust model estimation from putative matches)
/// - Export computed data
int main(int argc, char **argv) {
    CmdLine cmd;

    std::string sSfM_Data_Filename;
    std::string sMatchesDirectory = "";
    std::string sGeometricModel = "f";
    float fDistRatio = 0.8f;
    int iMatchingVideoMode = -1;

    std::string sPredefinedPairList = "";
    bool bUpRight = false;

    std::string sNearestMatchingMethod = "AUTO";
    bool bForce = false;
    bool bGuided_matching = false;
    int imax_iteration = 2048;
    bool gms = true;

    //required
    cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
    cmd.add(make_option('o', sMatchesDirectory, "out_dir"));

    // Options
    cmd.add(make_option('r', fDistRatio, "ratio"));
    cmd.add(make_option('g', sGeometricModel, "geometric_model"));
    cmd.add(make_option('v', iMatchingVideoMode, "video_mode_matching"));
    cmd.add(make_option('l', sPredefinedPairList, "pair_list"));
    cmd.add(make_option('n', sNearestMatchingMethod, "nearest_matching_method"));
    cmd.add(make_option('f', bForce, "force"));
    cmd.add(make_option('m', bGuided_matching, "guided_matching"));
    cmd.add(make_option('I', imax_iteration, "max_iteration"));
    cmd.add(make_option('G', gms, "use gms method"));

    try {
        if (argc == 1)
            throw std::string("Invalid command line parameter.");

        cmd.process(argc, argv);
    }

    catch (const std::string &s) {
        std::cerr << "Usage: " << argv[0] << '\n' << "[-i|--input_file] a SfM_Data file\n"
                  << "[-o|--out_dir path] output path where computed are stored\n" << "\n[Optional]\n"
                  << "[-f|--force] Force to recompute data]\n"
                  << "[-r|--ratio] Distance ratio to discard non meaningful matches\n" << "	 0.8: (default).\n"
                  << "[-g|--geometric_model]\n"
                  << "	(pairwise correspondences filtering thanks to robust model estimation):\n"
                  << "   f: (default) fundamental matrix,\n" << "	 e: essential matrix,\n"
                  << "	h: homography matrix.\n" << "[-v|--video_mode_matching]\n"
                  << "  (sequence matching with an overlap of X images)\n" << "   X: with match 0 with (1->X), ...]\n"
                  << "   2: will match 0 with (1,2), 1 with (2,3), ...\n"
                  << "   3: will match 0 with (1,2,3), 1 with (2,3,4), ...\n" << "[-l]--pair_list] file\n"
                  << "[-n|--nearest_matching_method]\n" << "  AUTO: auto choice from regions type,\n"
                  << "  For Scalar based regions descriptor:\n" << "    BRUTEFORCEL2: L2 BruteForce matching,\n"
                  << "	ANNL2: L2 Approximate Nearest Neighbor matching,\n"
                  << "	CASCADEHASHINGL2: L2 Cascade Hashing matching.\n" << "	  FASTCASCADEHASHINGL2: (default)\n"
                  << "	   L2 Cascade Hashing with precomputed hashed regions\n"
                  << "	  (faster than CASCADEHASHINGL2 but use more memory).\n" << "  For Binary based descriptor:\n"
                  << "    BRUTEFORCEHAMMING: BruteForce Hamming matching.\n" << "[-m|--guided_matching]\n"
                  << "  use the found model to improve the pairwise correspondences." << std::endl;

        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << " You called : " << "\n" << argv[0] << "\n" << "--input_file " << sSfM_Data_Filename << "\n"
              << "--out_dir " << sMatchesDirectory << "\n" << "Optional parameters:" << "\n" << "--force " << bForce
              << "\n" << "--ratio " << fDistRatio << "\n" << "--geometric_model " << sGeometricModel << "\n"
              << "--video_mode_matching " << iMatchingVideoMode << "\n" << "--pair_list " << sPredefinedPairList << "\n"
              << "--nearest_matching_method " << sNearestMatchingMethod << "\n" << "--guided_matching "
              << bGuided_matching << std::endl;

    EPairMode ePairmode = (iMatchingVideoMode == -1) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;

    if (sPredefinedPairList.length()) {
        ePairmode = PAIR_FROM_FILE;

        if (iMatchingVideoMode > 0) {
            std::cerr << "\nIncompatible options: --videoModeMatching and --pairList" << std::endl;
            return EXIT_FAILURE;
        }
    }

    if (sMatchesDirectory.empty() || !stlplus::is_folder(sMatchesDirectory)) {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return EXIT_FAILURE;
    }

    EGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;

    std::string sGeometricMatchesFilename = "";

    switch (sGeometricModel[0]) {
        case 'f':
        case 'F':
            eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
            sGeometricMatchesFilename = "matches.f.txt";
            break;

        case 'e':
        case 'E':
            eGeometricModelToCompute = ESSENTIAL_MATRIX;
            sGeometricMatchesFilename = "matches.e.txt";
            break;

        case 'h':
        case 'H':
            eGeometricModelToCompute = HOMOGRAPHY_MATRIX;
            sGeometricMatchesFilename = "matches.h.txt";
            break;

        default:
            std::cerr << "Unknown geometric model" << std::endl;
            return EXIT_FAILURE;
    }

    // -----------------------------
    // - Load SfM_Data Views & intrinsics data
    // a. Compute putative descriptor matches
    // b. Geometric filtering of putative matches
    // + Export some statistics
    // -----------------------------
    //---------------------------------------
    // Read SfM Scene (image view & intrinsics data)
    //---------------------------------------
    SfM_Data sfm_data;

    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
        std::cerr << std::endl << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read."
                  << std::endl;
        return EXIT_FAILURE;
    }

    //---------------------------------------
    // Load SfM Scene regions
    //---------------------------------------
    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace i23dSFM::features;
    const std::string sImage_describer = stlplus::create_filespec(sMatchesDirectory, "image_describer", "json");

    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);

    if (!regions_type) {
        std::cerr << "Invalid: " << sImage_describer << " regions type file." << std::endl;
        return EXIT_FAILURE;
    }

    //---------------------------------------
    // a. Compute putative descriptor matches
    //	  - Descriptor matching (according user method choice)
    //	  - Keep correspondences only if NearestNeighbor ratio is ok
    //---------------------------------------
    // Load the corresponding view regions
    std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();

    if (!regions_provider->load(sfm_data, sMatchesDirectory, regions_type)) {
        std::cerr << std::endl << "Invalid regions." << std::endl;
        return EXIT_FAILURE;
    }
// Read the features
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, sMatchesDirectory, regions_type)) {
        std::cerr << std::endl
                  << "Invalid features." << std::endl;
        return EXIT_FAILURE;
    }
    PairWiseMatches map_PutativesMatches;

    // Build some alias from SfM_Data Views data:
    // - List views as a vector of filenames & image sizes
    std::vector<std::string> vec_fileNames;
    std::vector<std::pair<size_t, size_t> > vec_imagesSize;
    {
        vec_fileNames.reserve(sfm_data.GetViews().size());
        vec_imagesSize.reserve(sfm_data.GetViews().size());

        for (Views::const_iterator iter = sfm_data.GetViews().begin(); iter != sfm_data.GetViews().end(); ++iter) {
            const View *v = iter->second.get();

            vec_fileNames.push_back(stlplus::create_filespec(sfm_data.s_root_path,
                                                             v->s_Img_path));
            vec_imagesSize.push_back(std::make_pair(v->ui_width, v->ui_height));
        }
    }

    std::cout << std::endl << " - PUTATIVE MATCHES - " << std::endl;

    // If the matches already exists, reload them
    if (!bForce && stlplus::file_exists(sMatchesDirectory + "/matches.putative.txt")) {
        PairedIndMatchImport(sMatchesDirectory + "/matches.putative.txt", map_PutativesMatches);
        std::cout << "\t PREVIOUS RESULTS LOADED" << std::endl;
    }

    else { // Compute the putative matches
        std::cout << "Use: ";

        switch (ePairmode) {
            case PAIR_EXHAUSTIVE:
                std::cout << "exhaustive pairwise matching" << std::endl;
                break;

            case PAIR_CONTIGUOUS:
                std::cout << "sequence pairwise matching" << std::endl;
                break;

            case PAIR_FROM_FILE:
                std::cout << "user defined pairwise matching" << std::endl;
                break;
        }

        // Allocate the right Matcher according the Matching requested method
        std::unique_ptr<Matcher> collectionMatcher;

        if (sNearestMatchingMethod == "AUTO") {
            if (regions_type->IsScalar()) {
                std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
                collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions_AllInMemory(fDistRatio));
            } else if (regions_type->IsBinary()) {
                std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
                collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_HAMMING));
            }
        } else if (sNearestMatchingMethod == "BRUTEFORCEL2") {
            std::cout << "Using BRUTE_FORCE_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_L2));
        } else if (sNearestMatchingMethod == "BRUTEFORCEHAMMING") {
            std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_HAMMING));
        } else if (sNearestMatchingMethod == "ANNL2") {
            std::cout << "Using ANN_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, ANN_L2));
        } else if (sNearestMatchingMethod == "CASCADEHASHINGL2") {
            std::cout << "Using CASCADE_HASHING_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, CASCADE_HASHING_L2));
        } else if (sNearestMatchingMethod == "FASTCASCADEHASHINGL2") {
            std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
            collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions_AllInMemory(fDistRatio));
        }

        if (!collectionMatcher) {
            std::cerr << "Invalid Nearest Neighbor method: " << sNearestMatchingMethod << std::endl;
            return EXIT_FAILURE;
        }

        // Perform the matching
        system::Timer timer;
        {
            // From matching mode compute the pair list that have to be matched:
            Pair_Set pairs;

            switch (ePairmode) {
                case PAIR_EXHAUSTIVE:
                    pairs = exhaustivePairs(sfm_data.GetViews().size());
                    break;

                case PAIR_CONTIGUOUS:
                    pairs = contiguousWithOverlap(sfm_data.GetViews().size(), iMatchingVideoMode);
                    break;

                case PAIR_FROM_FILE:
                    if (!loadPairs(sfm_data.GetViews().size(), sPredefinedPairList, pairs)) {
                        return EXIT_FAILURE;
                    };
                    break;
            }

            

            // Photometric matching of putative pairs
            for (auto it: map_PutativesMatches) {
                if (pairs.find(it.first) != pairs.end()) {
                    //std::cout << "Pair " << it.first.first << " " << it.first.second << " has been loaded" << std::endl;
                    pairs.erase(it.first);
                }
            }

            cout << "pairs size: " << pairs.size() << endl;

            collectionMatcher->Match(sfm_data, regions_provider, pairs, map_PutativesMatches);

            //---------------------------------------
            //-- Export putative matches
            //---------------------------------------
            std::ofstream file(std::string(sMatchesDirectory + "/matches.putative.txt").c_str());

            if (file.is_open())
                PairedIndMatchToStream(map_PutativesMatches, file);

            file.close();
        }
        std::cout << "Task (Regions Matching) done in (s): " << timer.elapsed() << std::endl;
    }

    //-- export putative matches Adjacency matrix
    PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
                                         map_PutativesMatches,
                                         stlplus::create_filespec(sMatchesDirectory, "PutativeAdjacencyMatrix", "svg"));

    //-- export view pair graph once putative graph matches have been computed
    {
        std::set<IndexT> set_ViewIds;
        std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),
                       std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
        graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_PutativesMatches));
        graph::exportToGraphvizData(stlplus::create_filespec(sMatchesDirectory, "putative_matches"),
                                    putativeGraph.g);
    }

    //---------------------------------------
    // b. Geometric filtering of putative matches
    //	  - AContrario Estimation of the desired geometric model
    //	  - Use an upper bound for the a contrario estimated threshold
    //---------------------------------------
    std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
            new ImageCollectionGeometricFilter(&sfm_data, regions_provider));

    if (filter_ptr) {
        system::Timer timer;
        std::cout << std::endl << " - Geometric filtering - " << std::endl;
        PairWiseMatches map_GeometricMatches;
        // If the matches already exists, reload them
        if (stlplus::file_exists(sMatchesDirectory + "/matches.f.txt")) {
            PairedIndMatchImport(sMatchesDirectory + "/matches.f.txt", map_GeometricMatches);
            std::cout << "\t PREVIOUS GEOMETRIC FILTERING RESULTS LOADED" << std::endl;
            ExportTracks(map_GeometricMatches, sMatchesDirectory);
        }
        else {
            if (gms) {
                #ifdef I23DSFM_USE_OPENMP
                #pragma omp     parallel for schedule(dynamic)
                #endif

                for (int pi = 0; pi < map_PutativesMatches.size(); ++pi) {
                    PairWiseMatches::const_iterator iter = map_PutativesMatches.begin();
                    std::advance(iter, pi);

                    auto p = *iter;
                    auto pair_ids = p.first;
                    auto keyPoint1 =
                            regions_provider.get()->regions_per_view[pair_ids.first].get()->GetRegionsPositions();

                    vector<KeyPoint> scvKp1;
                    for (auto p: keyPoint1) { scvKp1.emplace_back(Point2f(p.x(), p.y()), 20); }

                    auto keyPoint2 =
                            regions_provider.get()->regions_per_view[pair_ids.second].get()->GetRegionsPositions();

                    vector<KeyPoint> scvKp2;
                    for (auto p: keyPoint2) { scvKp2.emplace_back(Point2f(p.x(), p.y()), 20); }

                    vector<DMatch> dmatchs;

                    for (auto m: p.second) { dmatchs.emplace_back(m._i, m._j, 0); }

                    gms_matcher gms(scvKp1,
                                    Size(sfm_data.views[pair_ids.first].get()->ui_height,
                                        sfm_data.views[pair_ids.first].get()->ui_width),
                                    scvKp2,
                                    Size(sfm_data.views[pair_ids.second].get()->ui_height,
                                        sfm_data.views[pair_ids.second].get()->ui_width),
                                    dmatchs);

                    cout << p.first.first << "-" << p.first.second << ":" << dmatchs.size() << endl;
                    std::vector<bool> vbInliers;
                    auto num_inliers = gms.GetInlierMask(vbInliers, true, true);

                    cout << "Get total " << num_inliers << " matches." << endl;
                    IndMatches matches;

                    // draw matches
                    for (size_t i = 0; i < vbInliers.size(); ++i) {
                        if (vbInliers[i] == true) {
                            matches.emplace_back(dmatchs[i].queryIdx, dmatchs[i].trainIdx);
                        }
                    }

    #ifdef I23DSFM_USE_OPENMP
    #pragma  omp critical
    #endif
                    map_GeometricMatches.emplace(pair_ids, matches);
                }
            }

            else {
                if(gms){
                    map_PutativesMatches = map_GeometricMatches;
                    map_GeometricMatches.clear();
                }
                /* ac_ransac */
    //			map_PutativesMatches = map_GeometricMatches;
    //			map_GeometricMatches.clear();

                switch (eGeometricModelToCompute) {
                    case HOMOGRAPHY_MATRIX: {
                        const bool bGeometric_only_guided_matching = true;

                        filter_ptr->Robust_model_estimation(GeometricFilter_HMatrix_AC(4.0, imax_iteration),
                                                            map_PutativesMatches, bGuided_matching,
                                                            bGeometric_only_guided_matching ? -1.0 : 0.6);
                        map_GeometricMatches = filter_ptr->Get_geometric_matches();

                    }
                        break;

                    case FUNDAMENTAL_MATRIX: {
                        filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(4.0, imax_iteration),
                                                            map_PutativesMatches, bGuided_matching);
                        map_GeometricMatches = filter_ptr->Get_geometric_matches();
                    }
                        break;

                    case ESSENTIAL_MATRIX: {
                        filter_ptr->Robust_model_estimation(GeometricFilter_EMatrix_AC(4.0, imax_iteration),
                                                            map_PutativesMatches, bGuided_matching);
                        map_GeometricMatches = filter_ptr->Get_geometric_matches();

                        //-- Perform an additional check to remove pairs with poor overlap
                        std::vector<PairWiseMatches::key_type> vec_toRemove;
                        for (PairWiseMatches::const_iterator iterMap = map_GeometricMatches.begin();
                            iterMap != map_GeometricMatches.end(); ++iterMap) {
                            const size_t putativePhotometricCount = map_PutativesMatches.find(
                                    iterMap->first)->second.size();
                            const size_t putativeGeometricCount = iterMap->second.size();
                            const float ratio = putativeGeometricCount / (float) putativePhotometricCount;
                            if (putativeGeometricCount < 50 || ratio < .3f) {
                                // the pair will be removed
                                vec_toRemove.push_back(iterMap->first);
                            }
                        }
                        //-- remove discarded pairs
                        for (std::vector<PairWiseMatches::key_type>::const_iterator
                                    iter = vec_toRemove.begin(); iter != vec_toRemove.end(); ++iter) {
                            map_GeometricMatches.erase(*iter);
                        }

                    }
                        break;
                }
            }
        }

       
//        //-- Perform an additional check to remove pairs with poor overlap
//        std::vector<PairWiseMatches::key_type> vec_toRemove;
//
//        for (PairWiseMatches::const_iterator iterMap = map_GeometricMatches.begin();
//             iterMap != map_GeometricMatches.end(); ++iterMap) {
//            cout << iterMap->first.first << "-" << iterMap->first.second << ":"
//                 << map_PutativesMatches[iterMap->first].size() << "->" << iterMap->second.size() << endl;
//
//            const size_t putativePhotometricCount = map_PutativesMatches.find(iterMap->first)->second.size();
//            const size_t putativeGeometricCount = iterMap->second.size();
//            const float ratio = putativeGeometricCount / (float)
//
//                    putativePhotometricCount;
//
//            if (putativeGeometricCount < 50 || ratio < .3f) {
//                // the pair will be removed
//                vec_toRemove.push_back(iterMap->first);
//            }
//        }
//
//        //-- remove discarded pairs
//        for (std::vector<PairWiseMatches::key_type>::const_iterator iter = vec_toRemove.begin();
//             iter != vec_toRemove.end(); ++iter) {
//            map_GeometricMatches.erase(*iter);
//        }


        //---------------------------------------
        //-- Export geometric filtered matches
        //---------------------------------------
        std::ofstream file(string(sMatchesDirectory + "/" + sGeometricMatchesFilename).c_str());

        if (file.is_open())
            PairedIndMatchToStream(map_GeometricMatches, file);

        file.close();


        ofstream ftime(sMatchesDirectory+"/matchfiler_time.txt");
        ftime<<timer.elapsed()<<endl;
        std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
        //-- export Adjacency matrix
        std::cout << "\n Export Adjacency Matrix of the pairwise's geometric matches" << std::endl;
        PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
                                             map_GeometricMatches,
                                             stlplus::create_filespec(sMatchesDirectory, "GeometricAdjacencyMatrix",
                                                                      "svg"));

        //-- export view pair graph once geometric filter have been done
        {
            std::set<IndexT> set_ViewIds;
            std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),
                           std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
            graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_GeometricMatches));
            graph::exportToGraphvizData(stlplus::create_filespec(sMatchesDirectory, "geometric_matches"),
                                        putativeGraph.g);
        }
        ofstream ofp(sMatchesDirectory+"/points.txt");
        for(auto pairedIndMatch:map_GeometricMatches) {
            auto p=pairedIndMatch.first;
            auto width_i=sfm_data.GetViews().at(p.first).get()->ui_width;
            auto height_i=sfm_data.GetViews().at(p.first).get()->ui_height;
            ofp<<width_i<<"-"<<height_i<<endl;
            auto width_j=sfm_data.GetViews().at(p.second).get()->ui_width;
            auto height_j=sfm_data.GetViews().at(p.second).get()->ui_height;
            ofp<<width_j<<"-"<<height_j<<endl;

            auto landmarks_i=feats_provider->getFeatures(p.first);

            auto landmarks_j=feats_provider->getFeatures(p.second);


            for(auto match : pairedIndMatch.second){
                auto point_i=landmarks_i[match._i].coords();
                ofp<<"["<<point_i(0,0)<<","<<point_i(1,0)<<"]";
                ofp<<"-";
                auto point_j=landmarks_j.at(match._j).coords();
                ofp<<"["<<point_j(0,0)<<","<<point_j(1,0)<<"]";
                ofp<<endl;
            }

        }
        ofp.close();

        // Added by Chenyu
        // Date: 2018-10-22
        ExportTracks(map_GeometricMatches, sMatchesDirectory);

        // Added by Chenyu
        // Date: 2018-09-26
        const PairWiseMatches& full_matches = map_GeometricMatches;
        // Generate match.out file
        ofstream match_out(sMatchesDirectory + "/match.out");
        if (!match_out.is_open()) {
            std::cerr << "match.out cannot be created!\n";
            return false;
        }
        for (PairWiseMatches::const_iterator iter = full_matches.begin();
            iter != full_matches.end(); iter++) {
            match_out << iter->first.first << " "
                    << iter->first.second << " "
                    << (double)iter->second.size() << "\n";
            match_out << iter->first.second << " "
                    << iter->first.first << " "
                    << (double)iter->second.size() << "\n";
        }
        match_out.close();

    }

    return EXIT_SUCCESS;
}


