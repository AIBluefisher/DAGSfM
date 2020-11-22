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

#ifndef COLMAP_SRC_BASE_SIMILARITY_TRANSFORM_H_
#define COLMAP_SRC_BASE_SIMILARITY_TRANSFORM_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include "base/pose.h"
#include "base/projection.h"
#include "base/reconstruction.h"
#include "estimators/rigid_transformation3D_srt.h"
#include "estimators/similarity_transform.h"
#include "optim/loransac.h"
#include "util/alignment.h"
#include "util/types.h"

namespace colmap {

struct RANSACOptions;
class Reconstruction;

// 3D similarity transformation with 7 degrees of freedom.
class SimilarityTransform3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimilarityTransform3();

  explicit SimilarityTransform3(const Eigen::Matrix3x4d& matrix);

  explicit SimilarityTransform3(
      const Eigen::Transform<double, 3, Eigen::Affine>& transform);

  SimilarityTransform3(const double scale, const Eigen::Vector4d& qvec,
                       const Eigen::Vector3d& tvec);

  void Estimate(const std::vector<Eigen::Vector3d>& src,
                const std::vector<Eigen::Vector3d>& dst);

  SimilarityTransform3 Inverse() const;

  void TransformPoint(Eigen::Vector3d* xyz) const;
  void TransformPose(Eigen::Vector4d* qvec, Eigen::Vector3d* tvec) const;

  Eigen::Matrix4d Matrix() const;
  double Scale() const;
  Eigen::Vector4d Rotation() const;
  Eigen::Vector3d Translation() const;

 private:
  Eigen::Transform<double, 3, Eigen::Affine> transform_;
};

struct RobustAlignmentEstimator {};

struct ReconstructionAlignmentEstimator {
  static const int kMinNumSamples = 3;

  typedef const Image* X_t;
  typedef const Image* Y_t;
  typedef Eigen::Matrix3x4d M_t;

  void SetMaxReprojError(const double max_reproj_error) {
    max_squared_reproj_error_ = max_reproj_error * max_reproj_error;
  }

  void SetReconstructions(const Reconstruction* reconstruction1,
                          const Reconstruction* reconstruction2) {
    CHECK_NOTNULL(reconstruction1);
    CHECK_NOTNULL(reconstruction2);
    reconstruction1_ = reconstruction1;
    reconstruction2_ = reconstruction2;
  }

  // Estimate 3D similarity transform from corresponding projection centers.
  std::vector<M_t> Estimate(const std::vector<X_t>& images1,
                            const std::vector<Y_t>& images2) const {
    const int sample_size = 3;

    CHECK_GE(images1.size(), sample_size);
    CHECK_GE(images2.size(), sample_size);

    std::vector<Eigen::Vector3d> proj_centers1(images1.size());
    std::vector<Eigen::Vector3d> proj_centers2(images2.size());
    for (size_t i = 0; i < sample_size; ++i) {
      CHECK_EQ(images1[i]->ImageId(), images2[i]->ImageId());
      proj_centers1[i] = images1[i]->ProjectionCenter();
      proj_centers2[i] = images2[i]->ProjectionCenter();
    }

    SimilarityTransform3 tform12;
    tform12.Estimate(proj_centers1, proj_centers2);

    return {tform12.Matrix().topRows<3>()};

    // Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, sample_size);
    // Eigen::MatrixXd x2 = Eigen::MatrixXd::Zero(3, sample_size);
    // for (int i = 0; i < sample_size; i++) {
    //     CHECK_EQ(images1[i]->ImageId(), images2[i]->ImageId());
    //     x1.col(i) = images1[i]->ProjectionCenter();
    //     x2.col(i) = images2[i]->ProjectionCenter();
    // }

    // double s = 1.0;
    // Eigen::Vector3d t = Eigen::Vector3d::Zero();
    // Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    // DAGSfM::FindRTS(x1, x2, &s, &t, &R);
    // M_t model;
    // model.block(0, 0, 3, 3) = s * R;
    // model.block(0, 3, 3, 1) = t;

    // return { model };
  }

  // For each image, determine the ratio of 3D points that correctly project
  // from one image to the other image and vice versa for the given alignment.
  // The residual is then defined as 1 minus this ratio, i.e., an error
  // threshold of 0.3 means that 70% of the points for that image must reproject
  // within the given maximum reprojection error threshold.
  void Residuals(const std::vector<X_t>& images1,
                 const std::vector<Y_t>& images2, const M_t& alignment12,
                 std::vector<double>* residuals) const {
    CHECK_EQ(images1.size(), images2.size());
    CHECK_NOTNULL(reconstruction1_);
    CHECK_NOTNULL(reconstruction2_);

    const Eigen::Matrix3x4d alignment21 =
        SimilarityTransform3(alignment12).Inverse().Matrix().topRows<3>();

    residuals->resize(images1.size());

    for (size_t i = 0; i < images1.size(); ++i) {
      const auto& image1 = *images1[i];
      const auto& image2 = *images2[i];

      CHECK_EQ(image1.ImageId(), image2.ImageId());
      CHECK_EQ(image1.CameraId(), image2.CameraId());

      const auto& camera1 = reconstruction1_->Camera(image1.CameraId());
      const auto& camera2 = reconstruction2_->Camera(image2.CameraId());

      const Eigen::Matrix3x4d proj_matrix1 = image1.ProjectionMatrix();
      const Eigen::Matrix3x4d proj_matrix2 = image2.ProjectionMatrix();

      CHECK_EQ(image1.NumPoints2D(), image2.NumPoints2D());

      size_t num_inliers = 0;
      size_t num_common_points = 0;

      for (point2D_t point2D_idx = 0; point2D_idx < image1.NumPoints2D();
           ++point2D_idx) {
        // Check if both images have a 3D point.

        const auto& point2D1 = image1.Point2D(point2D_idx);
        if (!point2D1.HasPoint3D()) {
          continue;
        }

        const auto& point2D2 = image2.Point2D(point2D_idx);
        if (!point2D2.HasPoint3D()) {
          continue;
        }

        num_common_points += 1;

        // Reproject 3D point in image 1 to image 2.
        const Eigen::Vector3d xyz12 =
            alignment12 *
            reconstruction1_->Point3D(point2D1.Point3DId()).XYZ().homogeneous();
        if (CalculateSquaredReprojectionError(point2D2.XY(), xyz12,
                                              proj_matrix2, camera2) >
            max_squared_reproj_error_) {
          continue;
        }

        // Reproject 3D point in image 2 to image 1.
        const Eigen::Vector3d xyz21 =
            alignment21 *
            reconstruction2_->Point3D(point2D2.Point3DId()).XYZ().homogeneous();
        if (CalculateSquaredReprojectionError(point2D1.XY(), xyz21,
                                              proj_matrix1, camera1) >
            max_squared_reproj_error_) {
          continue;
        }

        num_inliers += 1;
      }

      if (num_common_points == 0) {
        (*residuals)[i] = 1.0;
      } else {
        const double negative_inlier_ratio =
            1.0 - static_cast<double>(num_inliers) /
                      static_cast<double>(num_common_points);
        (*residuals)[i] = negative_inlier_ratio * negative_inlier_ratio;
      }
    }

    // double mean_residual = 0.0;
    // for (auto residual : *residuals) {
    //     mean_residual += residual;
    // }
    // mean_residual /= residuals->size();
    // LOG(INFO) << "mean residual: " << mean_residual;
  }

 private:
  double max_squared_reproj_error_ = 0.0;
  const Reconstruction* reconstruction1_ = nullptr;
  const Reconstruction* reconstruction2_ = nullptr;
};

// Robustly compute alignment between reconstructions by finding images that
// are registered in both reconstructions. The alignment is then estimated
// robustly inside RANSAC from corresponding projection centers. An alignment
// is verified by reprojecting common 3D point observations.
// The min_inlier_observations threshold determines how many observations
// in a common image must reproject within the given threshold.
bool ComputeAlignmentBetweenReconstructions(
    const Reconstruction& src_reconstruction,
    const Reconstruction& ref_reconstruction,
    const double min_inlier_observations, const double max_reproj_error,
    Eigen::Matrix3x4d* alignment);

// Robustly compute alignment between reconstructions by finding images that
// are registered in both reconstructions. The alignment is then estimated
// robustly inside RANSAC from corresponding projection centers. An alignment
// is verified by reprojecting common 3D point observations.
// The min_inlier_observations threshold determines how many observations
// in a common image must reproject within the given threshold.
bool ComputeAlignmentBetweenReconstructions(
    const Reconstruction& src_reconstruction,
    const Reconstruction& ref_reconstruction,
    const double min_inlier_observations, const double max_reproj_error,
    std::vector<const Image*>& src_images,
    std::vector<const Image*>& ref_images, Eigen::Matrix3x4d* alignment);

}  // namespace colmap

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(colmap::SimilarityTransform3)

#endif  // COLMAP_SRC_BASE_SIMILARITY_TRANSFORM_H_
