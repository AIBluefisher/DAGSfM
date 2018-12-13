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
#include "third_party/cmdLine/cmdLine.h"
#include <omp.h>
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <fstream>

using namespace i23dSFM;
using namespace i23dSFM::cameras;
using namespace i23dSFM::matching;
using namespace i23dSFM::robust;
using namespace i23dSFM::sfm;
using namespace i23dSFM::matching_image_collection;
using namespace std;

int main(int argc, char* argv[])
{
    // string matches_dir = argv[1];
    string sfmdata_filename = argv[1];
    string matches_filename = argv[2];
    string matches_dir = stlplus::folder_part(matches_filename);
    ifstream stream(matches_filename.c_str());
    if (!stream.is_open()) {
        cerr << matches_filename << " cannot be opened!\n";
        return false;
    }

    vector<string> match_file_list;
    string match_file;
    while (stream >> match_file) {
        match_file_list.push_back(match_file);
    }
    stream.close();

    PairWiseMatches full_matches;
    for (auto filename : match_file_list) {
        PairWiseMatches block_matches;
        if (!PairedIndMatchImport(filename, block_matches)) {
            cerr << "cannot import matches file: " << filename << endl;
            return false;
        }
        CopyIndMatches(block_matches, full_matches);
    }

    ofstream file(matches_dir + "/matches.f.txt");
    if (file.is_open()) {
        PairedIndMatchToStream(full_matches, file);
    }
    file.close();

    // Generate match.out file
    ofstream match_out(matches_dir + "/match.out");
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

    // SfM_Data sfm_data;
    // if (!Load(sfm_data, sfmdata_filename, ESfM_Data(VIEWS | INTRINSICS))) {
    //     std::cerr << std::endl << "The input SfM_Data file \"" 
    //               << "\" cannot be read.\n";
    //     return false;
    // }
    // PairWiseMatches clMatches;
	// map<Pair, IndMatches>::iterator ite;
	// for(ite = full_matches.begin(); ite != full_matches.end(); ite++) {
	// 	Pair pair1, pair2;
	// 	pair2.second = pair1.first = ite->first.first;
	// 	pair2.first = pair1.second = ite->first.second;
	// 	clMatches[pair2] = clMatches[pair1] = ite->second;
	// }

	// // -- export initial camera cluster graph matrix, the format of the graph & matrix are below：
	// //       10	
	// //   1 ----- 2
	// //   |	  	 |
	// //  9|	  	 |6
	// //   |   7   |
	// //   3 ----- 5
	// //    \     /
	// //   11\   /28
	// //      \ /    
	// //       4
	// // -- then the matrix representation becomes
	// //  5 6 1			<--- # of nodes and edges and format
	// //  2 10 3 9		<--- nodes adjacent to 1 and corresponding edge weight
	// //  1 10 5 6	.
	// //  1 9 4 11 5 7	.
	// //  3 11 5 28	.
	// //  2 6 3 7 4 28	<--- nodes adjacent to 5 and corresponding edge weight
	// ofstream cgFile(std::string(matches_dir + "/camera_cluster.txt").c_str());
	// if(cgFile.is_open()) {
	// 	cgFile << sfm_data.views.size() << " " << full_matches.size() << " " << 1 << endl;
	// 	std::map<Pair, IndMatches>::iterator ite = clMatches.begin();
	// 	int pre = ite->first.first;
	// 	int cur;
	// 	cgFile << (ite->first.second + 1) << " " << ite->second.size() << " ";
	// 	++ite;
	// 	for(; ite != clMatches.end(); ite++)
	// 	{
	// 		cur = ite->first.first; 
	// 		if(cur != pre) cgFile << endl;
	// 		cgFile << (ite->first.second + 1) << " " << ite->second.size() << " ";
	// 		pre = cur;
	// 	}
		
	// }
	// cgFile.close();
	// ofstream cf(std::string(matches_dir + "/camera_cluster1.txt").c_str());
	// if(cf.is_open()) {
	// 	cf << sfm_data.views.size() << " " << full_matches.size() << endl;
	// 	std::map<Pair, IndMatches>::iterator ite1 = full_matches.begin();
	// 	int pre = ite1->first.first;
	// 	int cur;
	// 	cf << ite1->first.first << " " << ite1->first.second << " " << ite1->second.size() << " ";
	// 	++ite1;
	// 	for(; ite1 != full_matches.end(); ite1++) {
	// 		cur = ite1->first.first; 
	// 		if(cur != pre)  {
	// 			cf << endl << cur << " ";
	// 		}
	// 		cf << ite1->first.second << " " << ite1->second.size() << " ";
	// 		pre = cur;
	// 	}
	// }
	// cf.close();

}