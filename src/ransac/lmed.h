#ifndef RANSAC_LMED_H
#define RANSAC_LMED_H

#include "ransac/estimator.h"
#include "ransac/lmed_quality_measurement.h"
#include "ransac/random_sampler.h"
#include "ransac/sample_consensus_estimator.h"

namespace GraphSfM {
// Estimates a model using the least median of squares regression (LMed). This
// method was published in P. Rousseeuw, "Least Median of Squares
// Regression," Journal of the American statistical association, 1984. The idea
// is to find the model that minimizes the median of the squared residuals.
// The constraint for this method is that the dataset has to have at most 50% of
// the points as outliers.
// The implementation explores the model solution space randomly. In other
// words, the hypotheses are generated from subsets of data drawn uniformly.
template <class ModelEstimator>
class LMed : public SampleConsensusEstimator<ModelEstimator> 
{
public:
    typedef typename ModelEstimator::Datum Datum;
    typedef typename ModelEstimator::Model Model;

    LMed(const RansacParameters& ransac_params, const ModelEstimator& estimator)
        : SampleConsensusEstimator<ModelEstimator>(ransac_params, estimator) {}
    virtual ~LMed() {}

    bool Initialize() override 
    {
        const bool init_status =
            SampleConsensusEstimator<ModelEstimator>::Initialize(new RandomSampler(
                this->ransac_params_.rng, this->estimator_.SampleSize()));
        this->quality_measurement_.reset(
            new LmedQualityMeasurement(this->estimator_.SampleSize()));
        return init_status;
    }
};

}  // namespace GraphSfM

#endif  // RANSAC_LMED_H
