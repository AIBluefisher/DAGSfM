#ifndef RANSAC_PROSAC_SAMPLER_H
#define RANSAC_PROSAC_SAMPLER_H

#include <algorithm>
#include <glog/logging.h>
#include <random>
#include <cstdlib>
#include <vector>

#include "ransac/sampler.h"
#include "util/random.h"

namespace GraphSfM {
// Prosac sampler used for PROSAC implemented according to "Matching with PROSAC
// - Progressive Sampling Consensus" by Chum and Matas.
class ProsacSampler : public Sampler 
{
public:
    ProsacSampler(const std::shared_ptr<RandomNumberGenerator>& rng,
                  const int min_num_samples);
    ~ProsacSampler() {}

    bool Initialize(const int num_datapoints);
    // Set the sample such that you are sampling the kth prosac sample (Eq. 6).
    void SetSampleNumber(int k);

    // Samples the input variable data and fills the vector subset with the prosac
    // samples.
    // NOTE: This assumes that data is in sorted order by quality where data[i] is
    // of higher quality than data[j] for all i < j.
    bool Sample(std::vector<int>* subset_indices);

private:
    int num_datapoints_;
    // Number of iterations of PROSAC before it just acts like ransac.
    int ransac_convergence_iterations_;

    // The kth sample of prosac sampling.
    int kth_sample_number_;
};

}  // namespace GraphSfM

#endif  // RANSAC_PROSAC_SAMPLER_H
