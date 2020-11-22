#ifndef SRC_SFM_FILTER_VIEW_PAIRS_FROM_ORIENTATION_H_
#define SRC_SFM_FILTER_VIEW_PAIRS_FROM_ORIENTATION_H_

#include <Eigen/Core>
#include <unordered_map>

#include "base/database.h"
#include "sfm/twoview_info.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

// Filters view pairs based on the orientation estimates. If the relative
// rotation obtained from the two view match (i.e. TwoViewInfo.rotation_2)
// differs from the relative rotation formed by the orientation estimates by
// more than the threshold then the view pair is deemed "bad" and is removed.
// The view id pairs assumes the view with the lower id be the first entry in
// the ImagePair.
//
// Concretely, we only keep R_{i,j} if: ||R_{i,j} - R_j * R_i^t|| < tau, where
// || A - B || is the angular distance between A and B.
//
// NOTE: This function will remove any view pairs that contain a view that does
// not have an entry in orientations and will output a comment to LOG(WARNING).
void FilterViewPairsFromOrientation(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations,
    const double max_relative_rotation_difference_degrees,
    std::unordered_map<ImagePair, TwoViewInfo>& view_pairs, Database& database);
}  // namespace DAGSfM

#endif