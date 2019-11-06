#ifndef RANSAC_EXHAUSTIVE_RANSAC_H
#define RANSAC_EXHAUSTIVE_RANSAC_H

#include "ransac/exhaustive_sampler.h"
#include "ransac/sample_consensus_estimator.h"
#include "ransac/sampler.h"

namespace GraphSfM {

template <class ModelEstimator>
class ExhaustiveRansac : public SampleConsensusEstimator<ModelEstimator> 
{
public:
    typedef typename ModelEstimator::Datum Datum;
    typedef typename ModelEstimator::Model Model;

    ExhaustiveRansac(const RansacParameters& ransac_params,
                     const ModelEstimator& estimator)
        : SampleConsensusEstimator<ModelEstimator>(ransac_params, estimator) {}
    virtual ~ExhaustiveRansac() {}

    // Initializes the random sampler and inlier support measurement.
    bool Initialize() {
        Sampler* random_sampler = new ExhaustiveSampler(
            this->ransac_params_.rng, this->estimator_.SampleSize());
        return SampleConsensusEstimator<ModelEstimator>::Initialize(random_sampler);
    }
};

}  // namespace GraphSfM

#endif  // THEIA_RANSAC_EXHAUSTIVE_RANSAC_H_
