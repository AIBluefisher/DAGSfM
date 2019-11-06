#ifndef GRAPHSFM_SFM_TWOVIEW_INFO_H_
#define GRAPHSFM_SFM_TWOVIEW_INFO_H_

#include <Eigen/Core>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <stdint.h>

#include "math/rotation.h"
#include "sfm/types.h"

namespace GraphSfM {

// A struct to hold match and projection data between two views. It is assumed
// that the first view is at the origin with an identity rotation.
class TwoViewInfo 
{
public:
    TwoViewInfo() : position_2(Eigen::Vector3d::Zero()),
                    rotation_2(Eigen::Vector3d::Zero()),
                    visibility_score(1) {}

    Eigen::Vector3d position_2;
    Eigen::Vector3d rotation_2;

    // The visibility score is computed based on the inlier features from 2-view
    // geometry estimation. This score is similar to the number of verified
    // matches, but has a spatial weighting to encourage good coverage of the
    // image by the inliers. The visibility score here is the sum of the
    // visibility scores for each image.
    int visibility_score;
};

// Inverts the two view info such that the focal lengths are swapped and the
// rotation and position are inverted.
void SwapCameras(TwoViewInfo* twoview_info);

}  // namespace GraphSfM


#endif  // GRAPHSFM_SFM_TWOVIEW_INFO_H_
