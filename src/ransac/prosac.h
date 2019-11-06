#ifndef RANSAC_PROSAC_H
#define RANSAC_PROSAC_H

#include <algorithm>
#include <cstdlib>
#include <math.h>
#include <vector>

#include "ransac/estimator.h"
#include "ransac/prosac_sampler.h"
#include "ransac/sample_consensus_estimator.h"

namespace GraphSfM {
// Estimate a model using PROSAC. The Estimate method is inherited, but for
// PROSAC requires the data to be in sorted order by quality (with highest
// quality at index 0).
template <class ModelEstimator>
class Prosac : public SampleConsensusEstimator<ModelEstimator> 
{
public:
    typedef typename ModelEstimator::Datum Datum;
    typedef typename ModelEstimator::Model Model;

    Prosac(const RansacParameters& ransac_params, const ModelEstimator& estimator)
        : SampleConsensusEstimator<ModelEstimator>(ransac_params, estimator) {}
    ~Prosac() {}

    bool Initialize() {
        Sampler* prosac_sampler = new ProsacSampler(this->ransac_params_.rng,
                                                    this->estimator_.SampleSize());
        return SampleConsensusEstimator<ModelEstimator>::Initialize(prosac_sampler);
    }
};
}  // namespace GraphSfM

#endif  // THEIA_RANSAC_PROSAC_H_
