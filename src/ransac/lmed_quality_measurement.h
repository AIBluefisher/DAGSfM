#ifndef RANSAC_LMED_QUALITY_MEASUREMENT_H
#define RANSAC_LMED_QUALITY_MEASUREMENT_H

#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <vector>

#include "ransac/quality_measurement.h"

namespace GraphSfM {
// Quality metric according to P.Rousseeuw, "Least Median of Squares
// Regression," Journal of the American statistical association, 1984. The idea
// of Least Median of Squares Regression (LMed) is to find a hypothesis that
// minimizes the median of the squared residuals.
class LmedQualityMeasurement : public QualityMeasurement 
{
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
        const std::vector<double>& residuals) 
    {
        std::vector<double> squared_residuals(residuals.size());
        std::transform(residuals.begin(), residuals.end(),
                       squared_residuals.begin(), ComputeSquaredResidual);
        std::nth_element(squared_residuals.begin(),
                         squared_residuals.begin() + squared_residuals.size() / 2,
                         squared_residuals.end());
        double median = squared_residuals[squared_residuals.size() / 2];
        if ((squared_residuals.size() % 2) != 0) {
            std::nth_element(squared_residuals.begin(),
                             squared_residuals.begin() +
                             (squared_residuals.size() / 2) - 1,
                             squared_residuals.end());
            median = 0.5 * (squared_residuals[(squared_residuals.size() / 2) - 1] +
                            median);
        }
        return median;
    }

    // Calculates the inlier ratio from the residuals.
    void CalculateInliers(const std::vector<double>& residuals,
                          const double median,
                          const int min_num_samples,
                          std::vector<int>* inliers) 
    {
        // The median holds a squared residual. Thus, we take the squared root.
        // The threshold calculated here is computed based on a heuristic that
        // OpenCV uses. See modules/calib3d/src/ptsetreg.cpp
        const double inlier_threshold = 2.5 * 1.4826 *
            (1 + 5.0 / (residuals.size() - min_num_samples)) * std::sqrt(median);
        const double squared_inlier_threshold = inlier_threshold * inlier_threshold;
        for (int i = 0; i < residuals.size(); i++) {
            if ((residuals[i] * residuals[i]) < squared_inlier_threshold) {
                inliers->emplace_back(i);
            }
        }
    }
};

}  // namespace GraphSfM

#endif  // RANSAC_LMED_QUALITY_MEASUREMENT_H
