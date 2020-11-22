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

#include "ransac/lmed.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "ransac/estimator.h"
#include "ransac/lmed_quality_measurement.h"
#include "util/random.h"

namespace DAGSfM {
namespace {
// Number of synthetic points.
const int kNumInlierPoints = 5000;
const int kNumOutlierPoints = 2500;

RandomNumberGenerator rng(52);

// TODO(vfragoso): These classes below  (Point, Line, and LineEstimator) can be
// put in a single file. Several tests such as ransac_test.cc and prosac_test.cc
// use these classes.
struct Point {
  double x;
  double y;
  Point() {}
  Point(double _x, double _y) : x(_x), y(_y) {}
};

// y = mx + b
struct Line {
  double m;
  double b;
  Line() {}
  Line(double _m, double _b) : m(_m), b(_b) {}
};

class LineEstimator : public Estimator<Point, Line> {
 public:
  LineEstimator() {}
  ~LineEstimator() {}

  double SampleSize() const { return 2; }
  bool EstimateModel(const std::vector<Point>& data,
                     std::vector<Line>* models) const {
    Line model;
    model.m = (data[1].y - data[0].y) / (data[1].x - data[0].x);
    model.b = data[1].y - model.m * data[1].x;
    models->push_back(model);
    return true;
  }

  double Error(const Point& point, const Line& line) const {
    double a = -1.0 * line.m;
    double b = 1.0;
    double c = -1.0 * line.b;
    return fabs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
  }
};

class LmedTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    input_points = new std::vector<Point>;
    input_points->reserve(kNumInlierPoints + kNumOutlierPoints);
    for (int i = 0; i < kNumInlierPoints; ++i) {
      input_points->emplace_back(i + rng.RandGaussian(0.0, 0.1),
                                 i + rng.RandGaussian(0.0, 0.1));
    }
    for (int i = 0; i < kNumOutlierPoints; ++i) {
      input_points->emplace_back(rng.RandDouble(0.0, 10000),
                                 rng.RandDouble(0.0, 10000));
    }
    // Reshuffle.
    std::random_shuffle(input_points->begin(), input_points->end());
  }

  static void TearDownTestCase() { delete input_points; }

  // Synthetic points.
  static std::vector<Point>* input_points;
};

std::vector<Point>* LmedTest::input_points = nullptr;

}  // namespace

// Tests the computation of the squared residuals by using the correct line
// model.
TEST_F(LmedTest, ComputingQualityMeasureOfCorrectModel) {
  LineEstimator line_estimator;
  LmedQualityMeasurement lmed_quality_measurement(line_estimator.SampleSize());
  Line correct_line(1.0, 0.0);
  std::vector<double> residuals(input_points->size());
  for (int i = 0; i < residuals.size(); ++i) {
    residuals[i] = line_estimator.Error(input_points->at(i), correct_line);
  }
  std::vector<int> inliers;
  EXPECT_LT(lmed_quality_measurement.ComputeCost(residuals, &inliers), 0.5);
  const double inlier_ratio = static_cast<double>(inliers.size()) /
                              static_cast<double>(residuals.size());
  EXPECT_NEAR(inlier_ratio, 0.666, 0.1);
}

// Tests the Lmed estimator by fitting a line to the input_points.
TEST_F(LmedTest, LineFitting) {
  LineEstimator line_estimator;
  Line line;
  RansacParameters params;
  params.rng = std::make_shared<RandomNumberGenerator>(rng);
  // This threshold is arbitrary to comply with sample_consensus_estimator.h.
  params.error_thresh = 5.0;
  LMed<LineEstimator> lmed_line(params, line_estimator);
  lmed_line.Initialize();
  RansacSummary summary;
  CHECK(lmed_line.Estimate(*input_points, &line, &summary));
  EXPECT_LT(fabs(line.m - 1.0), 0.1);
  EXPECT_NEAR(
      static_cast<double>(summary.inliers.size()) / input_points->size(), 0.666,
      0.1);
}

}  // namespace DAGSfM
