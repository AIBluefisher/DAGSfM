#include "sfm/filter_view_pairs_from_orientation.h"

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <unordered_set>

#include "math/rotation.h"
#include "math/util.h"
#include "rotation_estimation/rotation_estimator.h"
#include "sfm/twoview_info.h"
#include "util/hash.h"
#include "util/map_util.h"
#include "util/types.h"

namespace DAGSfM {

namespace {

bool AngularDifferenceIsAcceptable(
    const Eigen::Vector3d& orientation1, const Eigen::Vector3d& orientation2,
    const Eigen::Vector3d& relative_orientation,
    const double sq_max_relative_rotation_difference_radians) {
  const Eigen::Vector3d composed_relative_rotation =
      MultiplyRotations(orientation2, -orientation1);
  const Eigen::Vector3d loop_rotation =
      MultiplyRotations(-relative_orientation, composed_relative_rotation);
  const double sq_rotation_angular_difference_radians =
      loop_rotation.squaredNorm();
  return sq_rotation_angular_difference_radians <=
         sq_max_relative_rotation_difference_radians;
}

}  // namespace

void FilterViewPairsFromOrientation(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations,
    const double max_relative_rotation_difference_degrees,
    std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
    Database& database) {
  CHECK_GE(max_relative_rotation_difference_degrees, 0.0);

  // Precompute the squared threshold in radians.
  const double max_relative_rotation_difference_radians =
      DegToRad(max_relative_rotation_difference_degrees);
  const double sq_max_relative_rotation_difference_radians =
      max_relative_rotation_difference_radians *
      max_relative_rotation_difference_radians;

  std::unordered_set<ImagePair> view_pairs_to_remove;

  for (auto& view_pair : view_pairs) {
    const Eigen::Vector3d* orientation1 =
        FindOrNull(orientations, view_pair.first.first);
    const Eigen::Vector3d* orientation2 =
        FindOrNull(orientations, view_pair.first.second);

    // If the view pair contains a view that does not have an orientation then
    // remove it.
    if (orientation1 == nullptr || orientation2 == nullptr) {
      LOG(WARNING)
          << "View pair (" << view_pair.first.first << ", "
          << view_pair.first.second
          << ") contains a view that does not exist! Removing the view pair.";
      view_pairs_to_remove.insert(view_pair.first);
      continue;
    }

    // Remove the view pair if the relative rotation estimate is not within the
    // tolerance.
    if (!AngularDifferenceIsAcceptable(
            *orientation1, *orientation2, view_pair.second.rotation_2,
            sq_max_relative_rotation_difference_radians)) {
      view_pairs_to_remove.insert(view_pair.first);
    } else {
      // Update relative rotations.
      view_pair.second.rotation_2 = geometry::RelativeRotationFromTwoRotations(
          *orientation1, *orientation2);
    }
  }

  // Remove all the "bad" relative poses.
  for (const ImagePair view_id_pair : view_pairs_to_remove) {
    view_pairs.erase(view_id_pair);
    database.DeleteMatches(view_id_pair.first, view_id_pair.second);
    database.DeleteInlierMatches(view_id_pair.first, view_id_pair.second);
  }
  VLOG(1) << "Removed " << view_pairs_to_remove.size()
          << " view pairs by rotation filtering.";
}

}  // namespace DAGSfM