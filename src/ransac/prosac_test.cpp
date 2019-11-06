#include <math.h>
#include <random>

#include "gtest/gtest.h"

#include "ransac/estimator.h"
#include "ransac/prosac.h"
// #include "test/test_utils.h"

namespace GraphSfM {
namespace {
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

TEST(ProsacTest, LineFitting) 
{
    // Create a set of points along y=x with a small random pertubation.
    std::default_random_engine generator(90);
    std::normal_distribution<double> gauss_distribution(0.0, 0.5);
    std::normal_distribution<double> small_distribution(0.0, 0.05);
    const int num_points = 10000;
    std::vector<Point> input_points(num_points);
    std::vector<double> confidence(num_points);

    for (int i = 0; i < num_points; ++i) {
        if (i < 300) {
            double noise_x = small_distribution(generator);
            double noise_y = small_distribution(generator);
            input_points[i] = Point(i + noise_x, i + noise_y);
            confidence[i] = 0.95;
        } else {
            double noise_x = gauss_distribution(generator);
            double noise_y = gauss_distribution(generator);
            input_points[i] = Point(i + noise_x, i + noise_y);
            confidence[i] = 0.1;
        }
    }

    LineEstimator line_estimator;
    Line line;
    RansacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(113);
    params.error_thresh = 0.5;
    Prosac<LineEstimator> prosac_line(params, line_estimator);
    prosac_line.Initialize();
    RansacSummary summary;
    prosac_line.Estimate(input_points, &line, &summary);
    ASSERT_LT(fabs(line.m - 1.0), 0.1);
}

}  // namespace GraphSfM
