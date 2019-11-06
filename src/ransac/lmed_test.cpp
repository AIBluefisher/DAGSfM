#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "ransac/estimator.h"
#include "ransac/lmed.h"
#include "ransac/lmed_quality_measurement.h"
#include "util/random.h"

namespace GraphSfM {
namespace {
// Number of synthetic points.
const int kNumInlierPoints = 5000;
const int kNumOutlierPoints = 2500;

RandomNumberGenerator rng(52);

// TODO(vfragoso): These classes below  (Point, Line, and LineEstimator) can be
// put in a single file. Several tests such as ransac_test.cc and prosac_test.cc
// use these classes.
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

class LmedTest : public ::testing::Test 
{
public:
    static void SetUpTestCase() 
    {
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

    static void TearDownTestCase() 
    {
        delete input_points;
    }

    // Synthetic points.
    static std::vector<Point>* input_points;
};

std::vector<Point>* LmedTest::input_points = nullptr;

}  // namespace

// Tests the computation of the squared residuals by using the correct line
// model.
TEST_F(LmedTest, ComputingQualityMeasureOfCorrectModel) 
{
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
TEST_F(LmedTest, LineFitting) 
{
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
    EXPECT_NEAR(static_cast<double>(summary.inliers.size()) /
                input_points->size(), 0.666, 0.1);
}

}  // namespace GraphSfM
