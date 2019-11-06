#ifndef SRC_MATH_DISTRIBUTION_H_
#define SRC_MATH_DISTRIBUTION_H_

#include <glog/logging.h>
#include <stdio.h>
#include <cmath>

#include "math/util.h"

namespace GraphSfM {
// Abstract class for probability disributions.
class Distribution {
 public:
  Distribution() {}
  virtual ~Distribution() {}

  // Evaluate the disribution at x. NOTE: If eval results in a change of private
  // variables of a derived class, you should implement the changes in a
  // different method (i.e. an Update method)
  virtual double Eval(const double x) const = 0;
};

// Normal Gaussian Distribution.
class NormalDistribution : public Distribution {
 public:
  NormalDistribution(const double mean, const double sigma) : mean_(mean) {
    CHECK_GT(sigma, 0)
        << "Sigma must be greater than zero in a normal distribution";
    alpha_ = 1.0 / (sigma * sqrt(2.0 * M_PI));
    beta_ = -1.0 / (2.0 * sigma * sigma);
  }

  ~NormalDistribution() {}

  double Eval(const double x) const {
    const double normalized_x = x - mean_;
    return alpha_ * exp(beta_ * normalized_x * normalized_x);
  }

 private:
  // Normal factor.
  double alpha_;
  // Normal factor.
  double beta_;
  // The mean of the distribution.
  double mean_;
};

// Uniform distribution between left and right. Probability is uniform when x is
// within this span, and 0 when x is outside of the span.
class UniformDistribution : public Distribution {
 public:
  UniformDistribution(const double left, const double right)
      : left_(left), right_(right) {
    CHECK_LT(left, right) << "Left bound must be less than the right bound for "
                          << "uniform distributions.";
    CHECK_NE(right, left) << "Left bound is equal to right bound! Uniform "
                          << "distribution must have a nonzero range.";
    inverse_span_ = (left == right) ? 1.0 : 1.0 / (right - left);
  }

  // Destructor
  ~UniformDistribution() {}

  double Eval(const double x) const {
    return (left_ <= x && x <= right_) ? inverse_span_ : 0;
  }

 protected:
  const double left_;
  const double right_;
  double inverse_span_;
};

}  // namespace GraphSfM

#endif  // GRAPHSFM_MATH_DISTRIBUTION_H_
