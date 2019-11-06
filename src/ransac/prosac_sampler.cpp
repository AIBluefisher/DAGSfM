#include <algorithm>
#include <glog/logging.h>
#include <random>
#include <cstdlib>
#include <vector>

#include "ransac/prosac_sampler.h"
#include "ransac/sampler.h"
#include "util/random.h"

namespace GraphSfM {
// Prosac sampler used for PROSAC implemented according to "Matching with PROSAC
// - Progressive Sampling Consensus" by Chum and Matas.
ProsacSampler::ProsacSampler(const std::shared_ptr<RandomNumberGenerator>& rng,
                             const int min_num_samples)
    : Sampler(rng, min_num_samples) {}

bool ProsacSampler::Initialize(const int num_datapoints) 
{
    num_datapoints_ = num_datapoints;
    ransac_convergence_iterations_ = 20000;
    kth_sample_number_ = 1;
    return true;
}

// Set the sample such that you are sampling the kth prosac sample (Eq. 6).
void ProsacSampler::SetSampleNumber(int k) { kth_sample_number_ = k; }

// Samples the input variable data and fills the vector subset with the prosac
// samples.
// NOTE: This assumes that data is in sorted order by quality where data[i] is
// of higher quality than data[j] for all i < j.
bool ProsacSampler::Sample(std::vector<int>* subset_indices) 
{
    // Set t_n according to the PROSAC paper's recommendation.
    double t_n = ransac_convergence_iterations_;
    int n = this->min_num_samples_;
    // From Equations leading up to Eq 3 in Chum et al.
    for (int i = 0; i < this->min_num_samples_; i++) {
        t_n *= static_cast<double>(n - i) / (num_datapoints_ - i);
    }

    double t_n_prime = 1.0;
    // Choose min n such that T_n_prime >= t (Eq. 5).
    for (int t = 1; t <= kth_sample_number_; t++) {
        if (t > t_n_prime && n < num_datapoints_) {
          double t_n_plus1 = (t_n * (n + 1.0)) / (n + 1.0 - this->min_num_samples_);
          t_n_prime += ceil(t_n_plus1 - t_n);
          t_n = t_n_plus1;
          n++;
        }
    }
    subset_indices->reserve(this->min_num_samples_);
    if (t_n_prime < kth_sample_number_) {
        // Randomly sample m data points from the top n data points.
        std::vector<int> random_numbers;
        for (int i = 0; i < this->min_num_samples_; i++) {
          // Generate a random number that has not already been used.
          int rand_number;
          while (std::find(random_numbers.begin(),
                           random_numbers.end(),
                           (rand_number = this->rng_->RandInt(0, n - 1))) !=
                 random_numbers.end()) {
          }

        random_numbers.push_back(rand_number);

        // Push the *unique* random index back.
        subset_indices->push_back(rand_number);
      }
    } else {
        std::vector<int> random_numbers;
        // Randomly sample m-1 data points from the top n-1 data points.
        for (int i = 0; i < this->min_num_samples_ - 1; i++) {
                // Generate a random number that has not already been used.
                int rand_number;
                while (std::find(random_numbers.begin(),
                                 random_numbers.end(),
                                 (rand_number = this->rng_->RandInt(0, n - 2))) !=
                       random_numbers.end()) {
                }
                random_numbers.push_back(rand_number);    

            // Push the *unique* random index back.
            subset_indices->push_back(rand_number);
        }
        // Make the last point from the nth position.
        subset_indices->push_back(n);
    }
    CHECK_EQ(subset_indices->size(), this->min_num_samples_)
        << "Prosac subset is incorrect "
        << "size!";
    kth_sample_number_++;
    return true;
}

}  // namespace GraphSfM
