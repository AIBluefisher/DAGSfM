/** \file feature_matching.h
 *	\brief functions related to matching utilities
 *
 */
// Author: Tianwei Shen <shentianweipku@gmail.com>

#ifndef VOT_FEATURE_MATCHING_H
#define VOT_FEATURE_MATCHING_H

#include <string>

namespace vot {
class SiftData;
class SiftMatchPair;
class MatchParam;

/**
 * @brief PairwiseSiftMatching: do a pairwise matching between two image(sift) files,
 * and save the match info in the .mat file.
 * @param sift1: the sift data of the first sift file
 * @param sift2: the sift data of the second sift file
 * @param match_pair: the result sift match pair
 * @param match_param: match parameters
 * @return
 */
bool PairwiseSiftMatching(SiftData &sift1, SiftData &sift2, SiftMatchPair &match_pair, MatchParam &match_param,
                          std::string imagefile1 = "", std::string imagefile2 = "");
}	// end of namespace vot

#endif	// VOT_FEATURE_MATCHING_H
