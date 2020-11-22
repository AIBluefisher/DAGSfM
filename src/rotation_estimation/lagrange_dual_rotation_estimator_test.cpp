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

#include "rotation_estimation/lagrange_dual_rotation_estimator.h"

#include <ceres/rotation.h>

#include <cstring>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <string>
#include <unordered_map>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "math/distribution.h"
#include "math/util.h"
#include "rotation_estimation/robust_rotation_estimator.h"
#include "solver/bcm_sdp_solver.h"
#include "util/map_util.h"
#include "util/random.h"
#include "util/timer.h"

using namespace Eigen;
using namespace DAGSfM;

class LagrangeDualRotationAveragingTest : public ::testing::Test {
 protected:
  std::unordered_map<image_t, Vector3d> orientations_;
  std::unordered_map<ImagePair, TwoViewInfo> view_pairs_;

 public:
  void TestLagrangeDualRotationEstimator(
      const int num_views, const int num_view_pairs,
      const double relative_rotation_noise, const double mean,
      const double variance, const double rotation_tolerance_degrees) {
    // At least a cycle graph
    ASSERT_LE(num_views + 1, num_view_pairs);

    // Set up the cameras
    CreateGTOrientations(num_views);
    CreateRelativeRotations(num_view_pairs, relative_rotation_noise, mean,
                            variance);

    // Initialize estimated orientations
    std::unordered_map<image_t, Eigen::Vector3d> estimated_orientations;
    // Or initialized randomly.
    this->InitializeRotationsFromSpanningTree(estimated_orientations);

    // Set up for lagrange dual summary
    solver::Summary summary;

    // Estimate global orientations
    LagrangeDualRotationEstimator rotation_estimator(num_views, 3);

    const size_t max_iterations = 500;
    const double tolerance = 1e-18;
    solver::SolverOption option(max_iterations, tolerance, true);
    option.log_to_terminal = false;
    option.solver_type = solver::RBR_BCM;
    // option.solver_type = solver::ADMM_SDP;
    rotation_estimator.SetRAOption(option);
    rotation_estimator.SetRASummary(summary);

    LOG(INFO) << "\nEstimating Global Orientations using Lagrange Dual...";
    colmap::Timer timer;
    timer.Start();
    EXPECT_TRUE(rotation_estimator.EstimateRotations(view_pairs_,
                                                     &estimated_orientations));
    EXPECT_EQ(estimated_orientations.size(), orientations_.size());
    timer.Pause();
    LOG(INFO) << "Elapsed time: " << timer.ElapsedSeconds();

    rotation_estimator.ComputeErrorBound(view_pairs_);
    const double alpha_max = rotation_estimator.GetErrorBound();
    const double error_bound = RadToDeg(alpha_max);

    // LOG(INFO) << "Align the rotations and measure the error";
    // // Align the rotations and measure the error
    // geometry::AlignOrientations(orientations_, &estimated_orientations);

    double sum_angular_error = 0.0;
    double min_angular_error = std::numeric_limits<double>::max();
    double max_angular_error = 0.0;

    for (unsigned i = 0; i < orientations_.size(); i++) {
      const auto& rotation = FindOrDie(orientations_, i);
      const Vector3d& estimated_rotation = FindOrDie(estimated_orientations, i);
      const Vector3d relative_rotation =
          geometry::RelativeRotationFromTwoRotations(estimated_rotation,
                                                     rotation, 0.0);
      const double angular_error = RadToDeg(relative_rotation.norm());

      sum_angular_error += angular_error;
      min_angular_error = std::min(min_angular_error, angular_error);
      max_angular_error = std::max(max_angular_error, angular_error);
      // EXPECT_LT(angular_error, rotation_tolerance_degrees)
      std::cout << "\n";
      LOG(INFO) << i << "-th GT        rotation angle = "
                << RadToDeg(rotation.norm());
      LOG(INFO) << i << "-th Estimated rotation angle = "
                << RadToDeg(estimated_rotation.norm());
      LOG(INFO) << i << "-th Angular Residual         = " << angular_error;
    }

    std::cout << "\n";
    LOG(INFO) << "Angular Residual Upperbound: " << error_bound;
    LOG(INFO) << "Average Angular Residual: "
              << sum_angular_error / orientations_.size();
    LOG(INFO) << "Maximum Angular Residual: " << max_angular_error;
    LOG(INFO) << "Minimum Angular Residual: " << min_angular_error;
  }

 protected:
  void SetUp() override {}

  void CreateGTOrientations(const int num_views) {
    double rotated_angle_rad = 0.0;
    Eigen::Vector3d unit_axis(0.0, 0.0, 1.0);

    for (int i = 0; i < num_views; i++) {
      // Rotation about the z-axis by 2π/n rad and
      // by construction, forming a cycle graph
      Eigen::Vector3d angle_axis = rotated_angle_rad * unit_axis;
      orientations_[i] = angle_axis;
      rotated_angle_rad += (2 * M_PI / num_views);
    }
  }

  void CreateRelativeRotations(const int num_view_pairs, const double noise,
                               const double mean, const double variance) {
    RandomNumberGenerator rng;
    NormalDistribution normal_distribution(mean, variance);
    const double relative_rotation_noise = normal_distribution.Eval(noise);
    const double noise_factor = 2.0;

    // Create a set of view id pairs that will contain a spanning tree
    for (size_t i = 1; i < orientations_.size(); i++) {
      const double x = rng.RandDouble(std::numeric_limits<double>::min(),
                                      std::numeric_limits<double>::max());
      const double real_noise = normal_distribution.Eval(x);

      const ImagePair view_id_pair(i - 1, i);
      view_pairs_[view_id_pair].visibility_score = geometry::rng.RandInt(1, 10);
      view_pairs_[view_id_pair].rotation_2 =
          geometry::RelativeRotationFromTwoRotations(
              FindOrDie(orientations_, view_id_pair.first),
              FindOrDie(orientations_, view_id_pair.second),
              noise_factor * real_noise /
                  view_pairs_[view_id_pair].visibility_score);
    }

    // Add random edges
    while (view_pairs_.size() < (unsigned)num_view_pairs) {
      ImagePair view_id_pair(
          geometry::rng.RandInt(0, orientations_.size() - 1),
          geometry::rng.RandInt(0, orientations_.size() - 1));

      // Ensure the first id is smaller than second id &&
      // do not add the views that already exists
      if (view_id_pair.first >= view_id_pair.second ||
          ContainsKey(view_pairs_, view_id_pair)) {
        continue;
      }

      view_pairs_[view_id_pair].visibility_score = geometry::rng.RandInt(1, 10);
      view_pairs_[view_id_pair].rotation_2 =
          geometry::RelativeRotationFromTwoRotations(
              FindOrDie(orientations_, view_id_pair.first),
              FindOrDie(orientations_, view_id_pair.second),
              noise_factor * relative_rotation_noise /
                  view_pairs_[view_id_pair].visibility_score);
    }
  }

  void InitializeRotationsFromSpanningTree(
      std::unordered_map<image_t, Vector3d>& initial_orientations) {
    initial_orientations[0] = Vector3d::Zero();
    for (size_t i = 1; i < orientations_.size(); i++) {
      initial_orientations[i] = geometry::ApplyRelativeRotation(
          FindOrDie(initial_orientations, i - 1),
          FindOrDieNoPrint(view_pairs_, ImagePair(i - 1, i)).rotation_2);
    }
    // for (size_t i = 0; i < orientations_.size(); i++) {
    //   initial_orientations[i] = Eigen::Vector3d::Zero();
    // }
  }
};

TEST_F(LagrangeDualRotationAveragingTest, smallTestNoNoise) {
  const int num_views = 4;
  const int num_view_pairs = 6;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.1;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, TwentyViewsTestWithSmallNoise) {
  const int num_views = 20;
  const int num_view_pairs = 30;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, TwentyViewsTestWithLargeNoise) {
  const int num_views = 20;
  const int num_view_pairs = 30;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, FiftyViewsTestWithSmallNoise) {
  const int num_views = 50;
  const int num_view_pairs = 75;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, FiftyViewsTestWithLargeNoise) {
  const int num_views = 50;
  const int num_view_pairs = 75;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, OneHundredViewsTestWithSmallNoise) {
  const int num_views = 100;
  const int num_view_pairs = 300;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 1e-8;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, OneHundredViewsTestWithLargeNoise) {
  const int num_views = 100;
  const int num_view_pairs = 300;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, TwoHundredViewsTestWithSmallNoise) {
  const int num_views = 200;
  const int num_view_pairs = 400;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, TwoHundredViewsTestWithLargeNoise) {
  const int num_views = 200;
  const int num_view_pairs = 400;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, FiveHundredViewsTestWithSmallNoise) {
  const int num_views = 500;
  const int num_view_pairs = 2000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, FiveHundredViewsTestWithLargeNoise) {
  const int num_views = 500;
  const int num_view_pairs = 2000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, OneThousandViewsTestWithSmallNoise) {
  const int num_views = 1000;
  const int num_view_pairs = 4000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, OneThousandViewsTestWithLargeNoise) {
  const int num_views = 1000;
  const int num_view_pairs = 4000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, FiveThousandViewsTestWithSmallNoise) {
  const int num_views = 5000;
  const int num_view_pairs = 20000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, FiveThousandViewsTestWithLargeNoise) {
  const int num_views = 5000;
  const int num_view_pairs = 20000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, TenThousandViewsTestWithSmallNoise) {
  const int num_views = 10000;
  const int num_view_pairs = 40000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, TenThousandViewsTestWithLargeNoise) {
  const int num_views = 10000;
  const int num_view_pairs = 40000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest,
       FiftyThousandViewsTestWithSmallNoise) {
  const int num_views = 50000;
  const int num_view_pairs = 200000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest,
       FiftyThousandViewsTestWithLargeNoise) {
  const int num_views = 50000;
  const int num_view_pairs = 200000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}