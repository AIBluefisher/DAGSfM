#ifndef FUNC_H_INCLUDED
#define FUNC_H_INCLUDED

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <iomanip>
#include <map>

#include "base/image.h"
#include "base/tracks.h"
#include "estimators/two_view_geometry.h"
#include "util/math.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {
const size_t kMax = static_cast<size_t>(std::numeric_limits<int32_t>::max());

typedef struct {
  std::map<int, int> view2D;
} track;

struct weightPair {
  weightPair(double w, size_t n) {
    weight = w;
    nInliers = n;
  }
  double weight;
  size_t nInliers;
};

struct width_height_focal_ {
  width_height_focal_(double w, double h, double f) {
    width = w;
    height = h;
    focal = f;
  }

  double width;
  double height;
  double focal;

  bool operator<(const width_height_focal_& other) const {
    if (width < other.width)
      return true;
    else if (width == other.width) {
      if (height < other.height)
        return true;
      else if (height == other.height) {
        if (focal < other.focal) return true;
      }
    }
    return false;
  }
};

bool comPair(std::pair<weightPair, colmap::image_pair_t> A,
             std::pair<weightPair, colmap::image_pair_t> B) {
  if (A.first.weight > B.first.weight) return true;

  if (A.first.weight == B.first.weight && A.first.nInliers > B.first.nInliers) {
    return true;
  }

  return false;
}

bool comCommunity(std::pair<int, int> A, std::pair<int, int> B) {
  if (A.second < B.second)
    return true;
  else
    return false;
}

bool compView(std::pair<int, Eigen::Vector2d> A,
              std::pair<int, Eigen::Vector2d> B) {
  Eigen::Vector2d a = A.second;
  Eigen::Vector2d b = B.second;

  if (a(0) > b(0))
    return true;
  else if (a(0) == b(0) && a(1) < b(1)) {
    return true;
  } else {
    return false;
  }
}

bool comAngle(std::pair<weightPair, colmap::image_pair_t> A,
              std::pair<weightPair, colmap::image_pair_t> B) {
  if (A.first.weight > B.first.weight) return true;

  if (A.first.weight == B.first.weight && A.first.nInliers > B.first.nInliers) {
    return true;
  }

  return false;
}

bool comInlier(std::pair<weightPair, colmap::image_pair_t> A,
               std::pair<weightPair, colmap::image_pair_t> B) {
  if (A.first.nInliers > B.first.nInliers) return true;

  if (A.first.nInliers == B.first.nInliers && A.first.weight > B.first.weight) {
    return true;
  }

  return false;
}

bool comImageWeight(std::pair<int, int> A, std::pair<int, int> B) {
  if (A.second > B.second) return true;

  if (A.second == B.second && A.first > B.first) {
    return true;
  }

  return false;
}

int mergeTwoClusters(std::map<colmap::image_t, size_t>& imgCluster,
                     colmap::image_t id1, colmap::image_t id2) {
  size_t c1 = imgCluster[id1];
  size_t c2 = imgCluster[id2];

  int nChanges = 0;
  if (c1 < c2) {
    for (auto& iC : imgCluster) {
      if (iC.second == c2) {
        iC.second = c1;
        nChanges++;
      }
    }
  } else {
    for (auto& iC : imgCluster) {
      if (iC.second == c1) {
        iC.second = c2;
        nChanges++;
      }
    }
  }
  return nChanges;
}

bool comEdges(std::pair<colmap::image_pair_t, double> A,
              std::pair<colmap::image_pair_t, double> B) {
  if (A.second > B.second)
    return true;
  else
    return false;
}

colmap::image_pair_t ComputePairID(colmap::image_t image_id1,
                                   colmap::image_t image_id2) {
  colmap::image_pair_t pair_id;
  if (image_id1 > image_id2) {
    pair_id = kMax * image_id2 + image_id1;
  } else {
    pair_id = kMax * image_id1 + image_id2;
  }

  return pair_id;
}

double ComputeTripleID(double a, double b, double c) {
  return a * 10000 * 100000 + b * 100000 + c;
}

double ComputeRotationError(const Eigen::Matrix3d& R12,
                            const Eigen::Matrix3d& R12_5points) {
  /* double row0 = R12.row(0) * R12_5points.transpose().col(0);
   double row1 = R12.row(1) * R12_5points.transpose().col(1);
   double row2 = R12.row(2) * R12_5points.transpose().col(2);
   double acosValue = (row0 + row1 + row2 - 1.0 ) / 2.0;
   acosValue = acosValue > 1.0 ? 1.0 : acosValue;
   acosValue = acosValue < -1.0 ? -1.0 : acosValue;
   return RadToDeg(acos(acosValue));*/

  const Eigen::Matrix3d& ErrorR = R12 * R12_5points.transpose();
  const Eigen::Matrix3d eRij(ErrorR);
  const Eigen::Vector3d erij;
  RotationMatrixToAngleAxis((const double*)eRij.data(), (double*)erij.data());
  return (double)(erij.norm() * 180.0 / 3.141592653);
}

double ComputeRaysAngle(const Eigen::Vector3d& ray1,
                        const Eigen::Vector3d& ray2) {
  Eigen::Vector3d r1 = ray1 / ray1.norm();
  Eigen::Vector3d r2 = ray2 / ray2.norm();
  return RadToDeg(acos(r1.dot(r2)));
}

void Transform2LocalTracks(
    tracks::STLMAPTracks& map_tracks, vector<track>& tracks_,
    std::map<colmap::image_pair_t, TwoViewGeometry>& EpipolarGeometry,
    std::map<colmap::image_t, class Image>& images_,
    std::map<colmap::image_pair_t, FeatureMatches>& ImgMatches) {
  tracks_.clear();
  std::map<colmap::image_pair_t, int> planesSame;
  std::map<int, int> trackViewData;
  trackViewData[10] = 0;

  LOG(INFO) << map_tracks.size() << " map_tracks " << std::endl;

  for (auto& trackMap : map_tracks) {
    const tracks::submapTrack& sTrack = trackMap.second;
    if (sTrack.size() < static_cast<size_t>(2) ||
        sTrack.size() >= static_cast<size_t>(100))
      continue;

    int indexNum = (int)sTrack.size() / 10;
    if (trackViewData.count(indexNum) > 0)
      trackViewData[indexNum]++;
    else {
      if (indexNum >= 10)
        trackViewData[10]++;
      else
        trackViewData[indexNum] = 1;
    }

    bool flag = false;
    track curTrack;
    for (auto& subTrack : sTrack) {
      std::map<int, int>& visibleTracks =
          images_[subTrack.first].VisibleTracks();
      curTrack.view2D[subTrack.first] = subTrack.second;
      if (visibleTracks.count(subTrack.second) >
          0)  /// one feature exist in many tracks
      {
        flag = true;
        break;
      }
      visibleTracks[subTrack.second] = tracks_.size();
    }

    if (!flag) tracks_.push_back(curTrack);
  }
  LOG(INFO) << tracks_.size() << " tracks " << std::endl;

  for (auto& trackV : trackViewData) {
    if (trackV.first < 10) {
      if (trackV.first == 0) {
        LOG(INFO) << 3 << " - " << (trackV.first + 1) * 10 << " : "
                  << trackV.second << std::endl;
      } else {
        LOG(INFO) << trackV.first * 10 << " - " << (trackV.first + 1) * 10
                  << " : " << trackV.second << std::endl;
      }
    } else
      LOG(INFO) << trackV.first * 10 << " - infinite"
                << " : " << trackV.second << std::endl;
  }
}
}  // namespace DAGSfM
#endif  // FUNC_H_INCLUDED
