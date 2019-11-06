#include <glog/logging.h>
#include <algorithm>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "ransac/random_sampler.h"
#include "util/random.h"

namespace GraphSfM {

namespace {

bool IsUnique(const std::vector<int>& vec) 
{
    std::vector<int> sorted_vec = vec;
    std::sort(sorted_vec.begin(), sorted_vec.end());
    return std::unique(sorted_vec.begin(), sorted_vec.end()) == sorted_vec.end();
}

}  // namespace

TEST(RandomSampler, UniqueMinimalSample) 
{
    std::shared_ptr<RandomNumberGenerator> rng =
        std::make_shared<RandomNumberGenerator>(55);
    static const int kMinNumSamples = 3;
    const std::vector<int> data_points = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    RandomSampler sampler(rng, kMinNumSamples);
    CHECK(sampler.Initialize(data_points.size()));
    for (int i = 0; i < 100; i++) {
        std::vector<int> subset;
        EXPECT_TRUE(sampler.Sample(&subset)); 

        // Make sure that the sampling is unique.
        EXPECT_EQ(subset.size(), kMinNumSamples);
        EXPECT_TRUE(IsUnique(subset));
    }
}

}  // namespace GraphSfM
