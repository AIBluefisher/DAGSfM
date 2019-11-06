#include <glog/logging.h>
#include <math.h>
#include <vector>

#include "gtest/gtest.h"

#include "ransac/estimator.h"
#include "ransac/exhaustive_ransac.h"
#include "util/random.h"

namespace GraphSfM {
namespace {
RandomNumberGenerator rng(46);

struct Point 
{
    double x;
    double y;
    Point() {}
    Point(double _x, double _y) : x(_x), y(_y) {}
};

// y = mx + b
struct Line 
{
    double m;
    double b;
    Line() {}
    Line(double _m, double _b) : m(_m), b(_b) {}
};

class LineEstimator : public Estimator<Point, Line> 
{
public:
    LineEstimator() {}
    ~LineEstimator() {}

    double SampleSize() const { return 2; }
    bool EstimateModel(const std::vector<Point>& data,
                       std::vector<Line>* models) const 
    {
        Line model;
        model.m = (data[1].y - data[0].y) / (data[1].x - data[0].x);
        model.b = data[1].y - model.m * data[1].x;
        models->push_back(model);
        return true;
    }

    double Error(const Point& point, const Line& line) const 
    {
        double a = -1.0 * line.m;
        double b = 1.0;
        double c = -1.0 * line.b;
        return fabs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
    }
};
}  // namespace

TEST(RansacTest, LineFitting) 
{
    // Create a set of points along y=x with a small random pertubation.
    std::vector<Point> input_points;
    for (int i = 0; i < 10000; ++i) {
        if (i % 2 == 0) {
          double noise_x = rng.RandGaussian(0.0, 0.1);
          double noise_y = rng.RandGaussian(0.0, 0.1);
          input_points.push_back(Point(i + noise_x, i + noise_y));
        } else {
          double noise_x = rng.RandDouble(0.0, 10000);
          double noise_y = rng.RandDouble(0.0, 10000);
          input_points.push_back(Point(noise_x, noise_y));
        }
    }

    LineEstimator line_estimator;
    Line line;
    RansacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(rng);
    params.error_thresh = 0.5;
    ExhaustiveRansac<LineEstimator> ransac_line(params, line_estimator);
    ransac_line.Initialize();
    RansacSummary summary;
    CHECK(ransac_line.Estimate(input_points, &line, &summary));
    ASSERT_LT(fabs(line.m - 1.0), 0.1);
}

TEST(RansacTest, TerminationNumInliers) 
{
    // Create a set of points along y=x with a small random pertubation.
    // Create a set of points along y=x with a small random pertubation.
    std::vector<Point> input_points;
    for (int i = 0; i < 10000; ++i) {
        if (i % 2 == 0) {
            double noise_x = rng.RandGaussian(0.0, 0.1);
            double noise_y = rng.RandGaussian(0.0, 0.1);
            input_points.push_back(Point(i + noise_x, i + noise_y));
        } else {
            double noise_x = rng.RandDouble(0.0, 10000);
            double noise_y = rng.RandDouble(0.0, 10000);
            input_points.push_back(Point(noise_x, noise_y));
        }
    }

    LineEstimator line_estimator;
    Line line;
    RansacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(rng);
    params.error_thresh = 0.5;
    ExhaustiveRansac<LineEstimator> ransac_line(params, line_estimator);
    ransac_line.Initialize();
    RansacSummary summary;
    ransac_line.Estimate(input_points, &line, &summary);
    ASSERT_GE(summary.inliers.size(), 2500);
}
}  // namespace GraphSfM
