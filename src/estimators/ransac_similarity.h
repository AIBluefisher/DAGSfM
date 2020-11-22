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

#ifndef ESTIMATORS_RANSAC_SIMILARITY_H_
#define ESTIMATORS_RANSAC_SIMILARITY_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <ctime>
#include <random>
#include <vector>

#include "estimators/rigid_transformation3D_srt.h"
#include "estimators/sim3.h"
#include "ransac/estimator.h"
#include "ransac/prosac.h"
#include "solver/l1_solver.h"

using namespace std;
using namespace Eigen;

namespace DAGSfM {
inline double ReprojectionErr(const Vector3d& point1, const Vector3d& point2,
                              const Matrix3d& R, const Vector3d& t,
                              const double& scale) {
  Vector3d transformed_point = scale * R * point1 + t;
  return (transformed_point - point2).norm();
}

struct Euclidean3D {
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  Euclidean3D() {
    R = Eigen::Matrix3d::Identity();
    t = Eigen::Vector3d::Zero();
  }

  Euclidean3D(const Eigen::Matrix3d& rotation,
              const Eigen::Vector3d& translation) {
    R = rotation;
    t = translation;
  }

  Euclidean3D(const Euclidean3D& eu) {
    R = eu.R;
    t = eu.t;
  }
};

struct Correspondence3D {
  Eigen::Vector3d p1;
  Eigen::Vector3d p2;

  Correspondence3D() {
    p1 = Eigen::Vector3d::Zero();
    p2 = Eigen::Vector3d::Zero();
  }

  Correspondence3D(const Eigen::Vector3d& point1,
                   const Eigen::Vector3d& point2) {
    p1 = point1;
    p2 = point2;
  }
};

class SimilarityEstimator : public Estimator<Correspondence3D, Sim3> {
 public:
  SimilarityEstimator() {}
  ~SimilarityEstimator() {}

  double SampleSize() const { return 4; }

  bool EstimateModel(const std::vector<Correspondence3D>& data,
                     std::vector<Sim3>* models) const {
    Sim3 model;
    int sample_size = (int)SampleSize();

    Eigen::MatrixXd x1 = MatrixXd::Zero(3, sample_size);
    Eigen::MatrixXd x2 = MatrixXd::Zero(3, sample_size);
    for (int i = 0; i < sample_size; i++) {
      x1.col(i) = data[i].p1;
      x2.col(i) = data[i].p2;
    }

    DAGSfM::FindRTS(x1, x2, &model.s, &model.t, &model.R);
    // DAGSfM::Refine_RTS(x1, x2, &model.s, &model.t, &model.R);

    models->push_back(model);
    return true;
  }

  double Error(const Correspondence3D& corre, const Sim3& sim3) const {
    return ReprojectionErr(corre.p1, corre.p2, sim3.R, sim3.t, sim3.s);
  }
};

typedef std::pair<Euclidean3D, Euclidean3D> CorrespondenceEuc;

class EuclideanEstimator : public Estimator<CorrespondenceEuc, Euclidean3D> {
 private:
  double _s;  // fixed scale
  std::vector<Correspondence3D> _correspondences;

 public:
  EuclideanEstimator() {}

  EuclideanEstimator(const double& scale,
                     const std::vector<Correspondence3D>& correspondences)
      : Estimator<CorrespondenceEuc, Euclidean3D>() {
    _s = scale;
    _correspondences.assign(correspondences.begin(), correspondences.end());
  }

  ~EuclideanEstimator() {}

  double SampleSize() const { return 1; }

  bool EstimateModel(const std::vector<CorrespondenceEuc>& data,
                     std::vector<Euclidean3D>* models) const {
    Euclidean3D model;
    int sample_size = (int)SampleSize();
    // double angle_axis[3] = {0, 0, 0};

    for (int i = 0; i < sample_size; i++) {
      Eigen::Matrix3d R_a = data[i].first.R, R_b = data[i].second.R;
      Eigen::Matrix3d R_ab = R_b.transpose() * R_a;
      model.R = R_ab;
    }

    std::vector<Eigen::Vector3d> translations;

    // construct a l1-solver to solve: argmin |T - t_{ab}|_1
    Eigen::MatrixXd A(3 * sample_size, 3);
    Eigen::VectorXd b(3 * sample_size);
    Eigen::VectorXd solution = Eigen::Vector3d::Zero();
    for (int i = 0; i < sample_size; i++) {
      A(3 * i + 0, 0) = A(3 * i + 1, 1) = A(3 * i + 2, 2) = 1.0;
    }

    for (int i = 0; i < sample_size; i++) {
      Eigen::Matrix3d R_b = data[i].second.R;
      Eigen::Vector3d t_a = data[i].first.t, t_b = data[i].second.t;
      Eigen::Vector3d t_ab = _s * R_b.transpose() * t_a - R_b.transpose() * t_b;
      b[3 * i + 0] = t_ab[0];
      b[3 * i + 1] = t_ab[1];
      b[3 * i + 2] = t_ab[2];
      translations.push_back(t_ab);
    }
    // L1Solver<Eigen::MatrixXd>::Options options;
    // options.max_num_iterations = 500;
    // L1Solver<Eigen::MatrixXd> l1_solver(options, A);
    // l1_solver.Solve(b, &solution);
    // model.t = solution;
    model.t = translations[0];

    models->push_back(model);
    return true;
  }

  double Error(const CorrespondenceEuc& ceuc, const Euclidean3D& euc) const {
    // double reproj_err = 0.0;
    // for (int i = 0; i < _correspondences.size(); i++) {
    //     reproj_err += ReprojectionErr(_correspondences[i].p1,
    //                                   _correspondences[i].p2,
    //                                   euc.R, euc.t, _s);
    // }
    // reproj_err /= (double)_correspondences.size();
    // return reproj_err;
    const Eigen::Vector3d c1 = -ceuc.first.R.transpose() * ceuc.first.t;
    const Eigen::Vector3d c2 = -ceuc.second.R.transpose() * ceuc.second.t;
    return ReprojectionErr(c1, c2, euc.R, euc.t, _s);
  }
};

inline void RansacSimilarity(const vector<Vector3d>& points1,
                             const vector<Vector3d>& points2,
                             vector<Vector3d>& inliers1,
                             vector<Vector3d>& inliers2, Matrix3d& R,
                             Vector3d& t, double& scale,
                             const double threshold = 0.001,
                             const double p = 0.99) {
  int size = points1.size();

  // prepare input data
  std::vector<Correspondence3D> input_points;
  for (int i = 0; i < size; i++) {
    input_points.push_back(Correspondence3D(points1[i], points2[i]));
  }

  SimilarityEstimator sim3_estimator;
  Sim3 sim3;
  RansacParameters params;
  params.rng =
      std::make_shared<RandomNumberGenerator>((unsigned int)time(NULL));
  params.error_thresh = threshold;
  params.max_iterations = 5000;
  params.use_mle = true;

  Prosac<SimilarityEstimator> prosac_sim3(params, sim3_estimator);
  prosac_sim3.Initialize();
  RansacSummary summary;
  // LOG(INFO) << "Estimating...";
  prosac_sim3.Estimate(input_points, &sim3, &summary);
  // LOG(INFO) << "Estimate ends";
  std::vector<int> inlier_indeces = summary.inliers;
  for (auto index : inlier_indeces) {
    inliers1.push_back(input_points[index].p1);
    inliers2.push_back(input_points[index].p2);
  }
  R = sim3.R;
  t = sim3.t;
  scale = sim3.s;
}

}  // namespace DAGSfM

#endif