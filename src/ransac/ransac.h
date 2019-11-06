#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include "ransac/random_sampler.h"
#include "ransac/sample_consensus_estimator.h"
#include "ransac/sampler.h"

namespace GraphSfM {

template <class ModelEstimator>
class Ransac : public SampleConsensusEstimator<ModelEstimator> 
{
public:
    typedef typename ModelEstimator::Datum Datum;
    typedef typename ModelEstimator::Model Model;

    Ransac(const RansacParameters& ransac_params, const ModelEstimator& estimator)
        : SampleConsensusEstimator<ModelEstimator>(ransac_params, estimator) {}
    virtual ~Ransac() {}

    // Initializes the random sampler and inlier support measurement.
    bool Initialize() {
        Sampler* random_sampler = new RandomSampler(this->ransac_params_.rng,
                                                    this->estimator_.SampleSize());
        return SampleConsensusEstimator<ModelEstimator>::Initialize(random_sampler);
    }
};

}  // namespace GraphSfM

#endif  // RANSAC_RANSAC_H
