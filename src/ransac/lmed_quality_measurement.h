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

#ifndef RANSAC_LMED_QUALITY_MEASUREMENT_H
#define RANSAC_LMED_QUALITY_MEASUREMENT_H

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "ransac/quality_measurement.h"

namespace DAGSfM {
// Quality metric according to P.Rousseeuw, "Least Median of Squares
// Regression," Journal of the American statistical association, 1984. The idea
// of Least Median of Squares Regression (LMed) is to find a hypothesis that
// minimizes the median of the squared residuals.
class LmedQualityMeasurement : public QualityMeasurement {
 public:
  explicit LmedQualityMeasurement(const int min_sample_size)
      : min_sample_size_(min_sample_size) {}
  virtual ~LmedQualityMeasurement() {}

  // The cost is the squared residual. LMed minimizes the median of the squared
  // residuals over the hypotheses.
  double ComputeCost(const std::vector<double>& residuals,
                     std::vector<int>* inliers) override {
    inliers->reserve(residuals.size());
    const double median = CalculateMedianOfSquaredResiduals(residuals);
    CalculateInliers(residuals, median, min_sample_size_, inliers);
    return median;
  }

 private:
  // Minimum number of samples to generate a hypothesis. This is used to
  // calculate a good threshold to count inliers.
  const int min_sample_size_;

  // --------------------------- Helper functions ------------------------------
  // Computes the squared of a residual.
  // Params:
  //   residual:  The residual to be squared.
  static double ComputeSquaredResidual(const double residual) {
    return residual * residual;
  }

  // Calculates the median of the squared residuals.
  // Params:
  //   residuals:  The residuals for each of the data points.
  double CalculateMedianOfSquaredResiduals(
      const std::vector<double>& residuals) {
    std::vector<double> squared_residuals(residuals.size());
    std::transform(residuals.begin(), residuals.end(),
                   squared_residuals.begin(), ComputeSquaredResidual);
    std::nth_element(squared_residuals.begin(),
                     squared_residuals.begin() + squared_residuals.size() / 2,
                     squared_residuals.end());
    double median = squared_residuals[squared_residuals.size() / 2];
    if ((squared_residuals.size() % 2) != 0) {
      std::nth_element(
          squared_residuals.begin(),
          squared_residuals.begin() + (squared_residuals.size() / 2) - 1,
          squared_residuals.end());
      median = 0.5 *
               (squared_residuals[(squared_residuals.size() / 2) - 1] + median);
    }
    return median;
  }

  // Calculates the inlier ratio from the residuals.
  void CalculateInliers(const std::vector<double>& residuals,
                        const double median, const int min_num_samples,
                        std::vector<int>* inliers) {
    // The median holds a squared residual. Thus, we take the squared root.
    // The threshold calculated here is computed based on a heuristic that
    // OpenCV uses. See modules/calib3d/src/ptsetreg.cpp
    const double inlier_threshold =
        2.5 * 1.4826 * (1 + 5.0 / (residuals.size() - min_num_samples)) *
        std::sqrt(median);
    const double squared_inlier_threshold = inlier_threshold * inlier_threshold;
    for (int i = 0; i < residuals.size(); i++) {
      if ((residuals[i] * residuals[i]) < squared_inlier_threshold) {
        inliers->emplace_back(i);
      }
    }
  }
};

}  // namespace DAGSfM

#endif  // RANSAC_LMED_QUALITY_MEASUREMENT_H
