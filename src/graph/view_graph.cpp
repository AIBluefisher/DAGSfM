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

#include "graph/view_graph.h"

#include <ceres/rotation.h>

#include "base/triplet_extractor.h"
#include "math/util.h"

namespace DAGSfM {

namespace {

double ComputeLoopRotationError(const TwoViewInfo& info_one_two,
                                const TwoViewInfo& info_one_three,
                                const TwoViewInfo& info_two_three) {
  // Get relative rotation matrices.
  Eigen::Matrix3d rotation1_2, rotation1_3, rotation2_3;
  ceres::AngleAxisToRotationMatrix(
      info_one_two.rotation_2.data(),
      ceres::ColumnMajorAdapter3x3(rotation1_2.data()));
  ceres::AngleAxisToRotationMatrix(
      info_one_three.rotation_2.data(),
      ceres::ColumnMajorAdapter3x3(rotation1_3.data()));
  ceres::AngleAxisToRotationMatrix(
      info_two_three.rotation_2.data(),
      ceres::ColumnMajorAdapter3x3(rotation2_3.data()));

  // Compute loop rotation.
  const Eigen::Matrix3d loop_rotation =
      rotation2_3 * rotation1_2 * rotation1_3.transpose();
  Eigen::Vector3d loop_rotation_angle_axis;
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(loop_rotation.data()),
      loop_rotation_angle_axis.data());

  // Return the angle of the loop rotation which is the error of the
  // concatenated triplet rotation.
  return RadToDeg(loop_rotation_angle_axis.norm());
}

void RemoveTripletEdgesFromInvalidViewPairs(
    const ImageIdTriplet& triplet,
    std::unordered_set<ImagePair>* invalid_image_pairs) {
  const ImagePair pair_01(std::get<0>(triplet), std::get<1>(triplet));
  const ImagePair pair_02(std::get<0>(triplet), std::get<2>(triplet));
  const ImagePair pair_12(std::get<1>(triplet), std::get<2>(triplet));
  invalid_image_pairs->erase(pair_01);
  invalid_image_pairs->erase(pair_02);
  invalid_image_pairs->erase(pair_12);
}

}  // namespace

bool ViewGraph::AddTwoViewGeometry(const ImagePair& image_pair,
                                   const TwoViewInfo& twoview_info) {
  if (twoview_geometries_.count(image_pair) > 0) {
    return false;
  }

  twoview_geometries_.emplace(image_pair, twoview_info);
  image_pairs_.emplace_back(image_pair);
  scores_.push_back(twoview_info.visibility_score);

  return true;
}

bool ViewGraph::RemoveTwoViewGeometry(const ImagePair& image_pair) {
  if (twoview_geometries_.count(image_pair) == 0) {
    return false;
  }

  twoview_geometries_.erase(image_pair);
  return true;
}

void ViewGraph::TwoviewGeometriesToImagePairs() {
  image_pairs_.clear();
  image_pairs_.reserve(twoview_geometries_.size());
  for (const auto& view_pair : twoview_geometries_) {
    image_pairs_.emplace_back(view_pair.first.first, view_pair.first.second);
  }
}

void ViewGraph::FilterViewGraphCyclesByRotation(
    const double max_loop_error_degrees) {
  const std::unordered_map<ImagePair, TwoViewInfo>& image_pairs =
      this->twoview_geometries_;

  // Initialize a list of invalid view pairs to all view pairs. View pairs
  // deemed valid will be removed from this list.
  std::unordered_set<ImagePair> invalid_image_pairs;
  invalid_image_pairs.reserve(image_pairs.size());
  for (const auto& image_pair : image_pairs) {
    invalid_image_pairs.insert(image_pair.first);
  }

  // Find all triplets.
  TripletExtractor<image_t> triplet_extractor;
  std::vector<std::vector<ImageIdTriplet> > connected_triplets;
  CHECK(triplet_extractor.ExtractTriplets(invalid_image_pairs,
                                          &connected_triplets))
      << "Could not extract triplets from view pairs.";

  // Examine the cycles of size 3 to determine invalid view pairs from the
  // rotations.
  for (const auto& triplets : connected_triplets) {
    for (const ImageIdTriplet& triplet : triplets) {
      const TwoViewInfo& info_one_two =
          image_pairs.at(ImagePair(std::get<0>(triplet), std::get<1>(triplet)));
      const TwoViewInfo& info_one_three =
          image_pairs.at(ImagePair(std::get<0>(triplet), std::get<2>(triplet)));
      const TwoViewInfo& info_two_three =
          image_pairs.at(ImagePair(std::get<1>(triplet), std::get<2>(triplet)));

      // Compute loop rotation error.
      const double loop_rotation_error_degrees = ComputeLoopRotationError(
          info_one_two, info_one_three, info_two_three);

      // Add the view pairs to the list of valid view pairs if the loop error is
      // within the designated tolerance.
      if (loop_rotation_error_degrees < max_loop_error_degrees) {
        RemoveTripletEdgesFromInvalidViewPairs(triplet, &invalid_image_pairs);
      }
    }
  }

  LOG(INFO) << "Removing " << invalid_image_pairs.size() << " of "
            << image_pairs.size()
            << " view pairs from loop rotation filtering.";

  // Remove any view pairs not in the list of valid edges.
  for (const ImagePair& image_pair_id : invalid_image_pairs) {
    RemoveTwoViewGeometry(image_pair_id);
  }
}

}  // namespace DAGSfM