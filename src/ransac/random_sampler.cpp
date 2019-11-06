#include "ransac/random_sampler.h"

#include <algorithm>
#include <glog/logging.h>
#include <memory>
#include <numeric>
#include <stdlib.h>
#include <vector>

#include "ransac/sampler.h"
#include "util/random.h"

namespace GraphSfM {

RandomSampler::RandomSampler(const std::shared_ptr<RandomNumberGenerator>& rng,
                             const int min_num_samples)
    : Sampler(rng, min_num_samples) {}

bool RandomSampler::Initialize(const int num_datapoints) 
{
    CHECK_GE(num_datapoints, this->min_num_samples_);
    sample_indices_.resize(num_datapoints);
    std::iota(sample_indices_.begin(), sample_indices_.end(), 0);
    return true;
}

// Samples the input variable data and fills the vector subset with the
// random samples.
bool RandomSampler::Sample(std::vector<int>* subset_indices) 
{
    subset_indices->reserve(this->min_num_samples_);
    for (int i = 0; i < this->min_num_samples_; i++) {
        std::swap(
            sample_indices_[i],
            sample_indices_[this->rng_->RandInt(i, sample_indices_.size() - 1)]);
        subset_indices->emplace_back(sample_indices_[i]);
    }

    return true;
}

}  // namespace GraphSfM
