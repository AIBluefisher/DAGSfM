#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

#include <memory>
#include <vector>

#include "util/random.h"

namespace GraphSfM {
// Purely virtual class used for the sampling consensus methods (e.g. Ransac,
// Prosac, MLESac, etc.)
class Sampler 
{
public:
    Sampler(const std::shared_ptr<RandomNumberGenerator>& rng,
              const int min_num_samples)
          : min_num_samples_(min_num_samples) {
        if (rng.get() == nullptr) {
            rng_ = std::make_shared<RandomNumberGenerator>();
        } else {
            rng_ = rng;
        }
    }

    virtual ~Sampler() {}

    // Initializes any non-trivial variables and sets up sampler if
    // necessary. Must be called before Sample is called.
    virtual bool Initialize(const int num_datapoints) = 0;

    // Samples the input variable data and fills the vector subset with the
    // samples.
    virtual bool Sample(std::vector<int>* subset_indices) = 0;

protected:
    std::shared_ptr<RandomNumberGenerator> rng_;
    int min_num_samples_;
};

}  // namespace GraphSfM

#endif  // RANSAC_SAMPLER_H
