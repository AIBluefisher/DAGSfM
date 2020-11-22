// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef SRC_ROTATION_ESTIMATION_ROTATION_ESTIMATOR_H_
#define SRC_ROTATION_ESTIMATION_ROTATION_ESTIMATOR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>

#include "math/util.h"
#include "rotation_estimation/align_rotations.h"
#include "sfm/twoview_info.h"
#include "util/map_util.h"
#include "util/random.h"
#include "util/types.h"
#include "util/util.h"

using namespace colmap;

namespace DAGSfM {

namespace geometry {
static RandomNumberGenerator rng(56);

// Computes R_ij = R_j * R_i^t.
inline Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2,
    const double noise) {
  const Eigen::Matrix3d noisy_rotation =
      Eigen::AngleAxisd(DegToRad(noise), rng.RandVector3d().normalized())
          .toRotationMatrix();

  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(noisy_rotation * rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

// Computes R_ij = R_j * R_i^t.
inline Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2) {
  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

// Aligns rotations to the ground truth rotations via a similarity
// transformation.
inline void AlignOrientations(
    const std::unordered_map<image_t, Eigen::Vector3d>& gt_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* rotations) {
  // Collect all rotations into a vector.
  std::vector<Eigen::Vector3d> gt_rot, rot;
  std::unordered_map<int, int> index_to_view_id;
  int current_index = 0;
  for (const auto& gt_rotation : gt_rotations) {
    gt_rot.emplace_back(gt_rotation.second);
    rot.emplace_back(FindOrDie(*rotations, gt_rotation.first));

    index_to_view_id[current_index] = gt_rotation.first;
    ++current_index;
  }

  AlignRotations(gt_rot, &rot);

  for (unsigned int i = 0; i < rot.size(); i++) {
    const image_t view_id = FindOrDie(index_to_view_id, i);
    (*rotations)[view_id] = rot[i];
  }
}

// return R_j = R_ij * R_i.
inline Eigen::Vector3d ApplyRelativeRotation(
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& relative_rotation) {
  Eigen::Vector3d rotation2;
  Eigen::Matrix3d rotation1_matrix, relative_rotation_matrix;
  ceres::AngleAxisToRotationMatrix(
      rotation1.data(), ceres::ColumnMajorAdapter3x3(rotation1_matrix.data()));
  ceres::AngleAxisToRotationMatrix(
      relative_rotation.data(),
      ceres::ColumnMajorAdapter3x3(relative_rotation_matrix.data()));

  const Eigen::Matrix3d rotation2_matrix =
      relative_rotation_matrix * rotation1_matrix;
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(rotation2_matrix.data()), rotation2.data());
  return rotation2;
}

}  // namespace geometry

// The recommended type of rotations solver is the Robust L1-L2 method. This
// method is scalable, extremely accurate, and very efficient. See the
// global_pose_estimation directory for more details.
enum class GlobalRotationEstimatorType {
  ROBUST_L1L2 = 0,
  NONLINEAR = 1,
  LAGRANGIAN_DUAL = 2
};

// A generic class defining the interface for global rotation estimation
// methods. These methods take in as input the relative pairwise orientations
// and output estimates for the global orientation of each view.
class RotationEstimator {
 public:
  RotationEstimator() {}
  virtual ~RotationEstimator() {}
  // Input the view pairs containing relative rotations between matched
  // geometrically verified views and outputs a rotation estimate for each view.
  //
  // Returns true if the rotation estimation was a success, false if there was a
  // failure. If false is returned, the contents of rotations are undefined.
  virtual bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* rotations) = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(RotationEstimator);
};

}  // namespace DAGSfM

#endif  // SRC_ROTATION_ESTIMATION_ROTATION_ESTIMATOR_H_
