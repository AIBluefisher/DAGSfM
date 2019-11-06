#ifndef GRAPHSFM_SFM_TYPES_H_
#define GRAPHSFM_SFM_TYPES_H_

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <stdint.h>
#include <utility>
#include <tuple>

namespace GraphSfM {

typedef uint32_t ViewId;
typedef uint32_t TrackId;
typedef uint32_t CameraIntrinsicsGroupId;
typedef std::pair<ViewId, ViewId> ViewIdPair;
typedef std::tuple<ViewId, ViewId, ViewId> ViewIdTriplet;

static const ViewId kInvalidViewId = std::numeric_limits<ViewId>::max();
static const TrackId kInvalidTrackId = std::numeric_limits<TrackId>::max();
static const CameraIntrinsicsGroupId kInvalidCameraIntrinsicsGroupId =
    std::numeric_limits<CameraIntrinsicsGroupId>::max();

// Used as the projection matrix type.
typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;

}  // namespace GraphSfM

#endif  // GRAPHSFM_SFM_TYPES_H_
