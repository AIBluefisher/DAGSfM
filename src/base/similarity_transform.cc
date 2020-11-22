// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
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
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "base/similarity_transform.h"

#include "base/pose.h"
#include "base/projection.h"
#include "base/reconstruction.h"
#include "estimators/similarity_transform.h"
#include "optim/loransac.h"

namespace colmap {

SimilarityTransform3::SimilarityTransform3() {
  SimilarityTransform3(1, ComposeIdentityQuaternion(),
                       Eigen::Vector3d(0, 0, 0));
}

SimilarityTransform3::SimilarityTransform3(const Eigen::Matrix3x4d& matrix) {
  transform_.matrix().topLeftCorner<3, 4>() = matrix;
}

SimilarityTransform3::SimilarityTransform3(
    const Eigen::Transform<double, 3, Eigen::Affine>& transform)
    : transform_(transform) {}

SimilarityTransform3::SimilarityTransform3(const double scale,
                                           const Eigen::Vector4d& qvec,
                                           const Eigen::Vector3d& tvec) {
  Eigen::Matrix4d matrix = Eigen::MatrixXd::Identity(4, 4);
  matrix.topLeftCorner<3, 4>() = ComposeProjectionMatrix(qvec, tvec);
  matrix.block<3, 3>(0, 0) *= scale;
  transform_.matrix() = matrix;
}

void SimilarityTransform3::Estimate(const std::vector<Eigen::Vector3d>& src,
                                    const std::vector<Eigen::Vector3d>& dst) {
  transform_.matrix().topLeftCorner<3, 4>() =
      SimilarityTransformEstimator<3>().Estimate(src, dst)[0];
}

SimilarityTransform3 SimilarityTransform3::Inverse() const {
  return SimilarityTransform3(transform_.inverse());
}

void SimilarityTransform3::TransformPoint(Eigen::Vector3d* xyz) const {
  *xyz = transform_ * *xyz;
}

void SimilarityTransform3::TransformPose(Eigen::Vector4d* qvec,
                                         Eigen::Vector3d* tvec) const {
  // Projection matrix P1 projects 3D object points to image plane and thus to
  // 2D image points in the source coordinate system:
  //    x' = P1 * X1
  // 3D object points can be transformed to the destination system by applying
  // the similarity transformation S:
  //    X2 = S * X1
  // To obtain the projection matrix P2 that transforms the object point in the
  // destination system to the 2D image points, which do not change:
  //    x' = P2 * X2 = P2 * S * X1 = P1 * S^-1 * S * X1 = P1 * I * X1
  // and thus:
  //    P2' = P1 * S^-1
  // Finally, undo the inverse scaling of the rotation matrix:
  //    P2 = s * P2'

  Eigen::Matrix4d src_matrix = Eigen::MatrixXd::Identity(4, 4);
  src_matrix.topLeftCorner<3, 4>() = ComposeProjectionMatrix(*qvec, *tvec);
  Eigen::Matrix4d dst_matrix =
      src_matrix.matrix() * transform_.inverse().matrix();
  dst_matrix *= Scale();

  *qvec = RotationMatrixToQuaternion(dst_matrix.block<3, 3>(0, 0));
  *tvec = dst_matrix.block<3, 1>(0, 3);
}

Eigen::Matrix4d SimilarityTransform3::Matrix() const {
  return transform_.matrix();
}

double SimilarityTransform3::Scale() const {
  return Matrix().block<1, 3>(0, 0).norm();
}

Eigen::Vector4d SimilarityTransform3::Rotation() const {
  return RotationMatrixToQuaternion(Matrix().block<3, 3>(0, 0) / Scale());
}

Eigen::Vector3d SimilarityTransform3::Translation() const {
  return Matrix().block<3, 1>(0, 3);
}

bool ComputeAlignmentBetweenReconstructions(
    const Reconstruction& src_reconstruction,
    const Reconstruction& ref_reconstruction,
    const double min_inlier_observations, const double max_reproj_error,
    Eigen::Matrix3x4d* alignment) {
  CHECK_GE(min_inlier_observations, 0.0);
  CHECK_LE(min_inlier_observations, 1.0);

  RANSACOptions ransac_options;
  ransac_options.max_error = 1.0 - min_inlier_observations;
  ransac_options.min_inlier_ratio = 0.2;

  LORANSAC<ReconstructionAlignmentEstimator, ReconstructionAlignmentEstimator>
      ransac(ransac_options);
  ransac.estimator.SetMaxReprojError(max_reproj_error);
  ransac.estimator.SetReconstructions(&src_reconstruction, &ref_reconstruction);
  ransac.local_estimator.SetMaxReprojError(max_reproj_error);
  ransac.local_estimator.SetReconstructions(&src_reconstruction,
                                            &ref_reconstruction);

  const auto& common_image_ids =
      src_reconstruction.FindCommonRegImageIds(ref_reconstruction);

  if (common_image_ids.size() < 3) {
    return false;
  }

  std::vector<const Image*> src_images(common_image_ids.size());
  std::vector<const Image*> ref_images(common_image_ids.size());
  for (size_t i = 0; i < common_image_ids.size(); ++i) {
    src_images[i] = &src_reconstruction.Image(common_image_ids[i]);
    ref_images[i] = &ref_reconstruction.Image(common_image_ids[i]);
  }

  const auto report = ransac.Estimate(src_images, ref_images);

  if (report.success) {
    *alignment = report.model;
  }

  return report.success;
}

bool ComputeAlignmentBetweenReconstructions(
    const Reconstruction& src_reconstruction,
    const Reconstruction& ref_reconstruction,
    const double min_inlier_observations, const double max_reproj_error,
    std::vector<const Image*>& src_images,
    std::vector<const Image*>& ref_images, Eigen::Matrix3x4d* alignment) {
  CHECK_GE(min_inlier_observations, 0.0);
  CHECK_LE(min_inlier_observations, 1.0);

  RANSACOptions ransac_options;
  ransac_options.max_error = 1.0 - min_inlier_observations;
  ransac_options.min_inlier_ratio = 0.2;

  LORANSAC<ReconstructionAlignmentEstimator, ReconstructionAlignmentEstimator>
      ransac(ransac_options);
  ransac.estimator.SetMaxReprojError(max_reproj_error);
  ransac.estimator.SetReconstructions(&src_reconstruction, &ref_reconstruction);
  ransac.local_estimator.SetMaxReprojError(max_reproj_error);
  ransac.local_estimator.SetReconstructions(&src_reconstruction,
                                            &ref_reconstruction);

  const auto& common_image_ids =
      src_reconstruction.FindCommonRegImageIds(ref_reconstruction);

  if (common_image_ids.size() < 3) {
    return false;
  }

  src_images.resize(common_image_ids.size());
  ref_images.resize(common_image_ids.size());
  for (size_t i = 0; i < common_image_ids.size(); ++i) {
    src_images[i] = &src_reconstruction.Image(common_image_ids[i]);
    ref_images[i] = &ref_reconstruction.Image(common_image_ids[i]);
  }

  const auto report = ransac.Estimate(src_images, ref_images);

  if (report.success) {
    *alignment = report.model;
  }

  return report.success;
}

}  // namespace colmap
