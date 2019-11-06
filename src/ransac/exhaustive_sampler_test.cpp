#include <algorithm>
#include <glog/logging.h>
#include <memory>
#include <utility>
#include <vector>

#include "ransac/exhaustive_sampler.h"
#include "util/map_util.h"
#include "util/random.h"
#include "gtest/gtest.h"

namespace GraphSfM {

TEST(ExhaustiveSampler, EnsureExhaustiveSample) 
{
    std::shared_ptr<RandomNumberGenerator> rng =
        std::make_shared<RandomNumberGenerator>(55);
    static const int kMinNumSamples = 2;
    static const int kNumDataPoints = 100;
    std::vector<int> data_points(kNumDataPoints);
    std::iota(data_points.begin(), data_points.end(), 0);

    ExhaustiveSampler sampler(rng, kMinNumSamples);
    CHECK(sampler.Initialize(data_points.size()));

    for (int i = 0; i < data_points.size(); i++) {
        for (int j = i + 1; j < data_points.size(); j++) {
            std::vector<int> subset;
            EXPECT_TRUE(sampler.Sample(&subset));

            // Make sure that the sampling is unique.
            EXPECT_EQ(subset.size(), kMinNumSamples);
            EXPECT_EQ(subset[0], i);
            EXPECT_EQ(subset[1], j);
        }
    }

    // The next sample after all combinations are enumerated should be (0, 1).
    std::vector<int> subset;
    EXPECT_TRUE(sampler.Sample(&subset));
    EXPECT_EQ(subset[0], 0);
    EXPECT_EQ(subset[1], 1);
}

}  // namespace GraphSfM
