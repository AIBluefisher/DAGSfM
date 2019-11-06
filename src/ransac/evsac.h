#ifndef THEIA_RANSAC_EVSAC_H_
#define THEIA_RANSAC_EVSAC_H_

#include <statx/distributions/evd/common.h>

#include "ransac/estimator.h"
#include "ransac/evsac_sampler.h"
#include "ransac/sample_consensus_estimator.h"

namespace GraphSfM {
// Estimate a model using EVSAC sampler.
template <class ModelEstimator>
class Evsac : public SampleConsensusEstimator<ModelEstimator> 
{
public:
    typedef typename ModelEstimator::Datum Datum;
    typedef typename ModelEstimator::Model Model;

    // Params:
    // sorted_distances:  The matrix containing k L2 sorted distances. The
    //   matrix has num. of query features as rows and k columns.
    // predictor_threshold:  The threshold used to decide correct or
    //   incorrect matches/correspondences. The recommended value is 0.65.
    // fitting_method:  The fiting method MLE or QUANTILE_NLS (see statx doc).
    //   The recommended fitting method is the MLE estimation.
    Evsac(const RansacParameters& ransac_params,
          const ModelEstimator& estimator,
          const Eigen::MatrixXd& sorted_distances,
          const double predictor_threshold,
          const FittingMethod fitting_method)
        : SampleConsensusEstimator<ModelEstimator>(ransac_params, estimator),
          sorted_distances_(sorted_distances),
          predictor_threshold_(predictor_threshold),
          fitting_method_(fitting_method) {}

    ~Evsac() {}

    bool Initialize() 
    {
      Sampler* prosac_sampler =
          new EvsacSampler<Datum>(this->estimator_.SampleSize(),
                                  this->sorted_distances_,
                                  this->predictor_threshold_,
                                  this->fitting_method_);
      return SampleConsensusEstimator<ModelEstimator>::Initialize(prosac_sampler);
    }

protected:
    // L2 descriptor sorted distances.
    // rows: num of reference features.
    // cols: k-th smallest distances.
    const Eigen::MatrixXd& sorted_distances_;
    // Threshold for predictor (MR-Rayleigh).
    const double predictor_threshold_;
    // The fitting method.
    const FittingMethod fitting_method_;

private:
    DISALLOW_COPY_AND_ASSIGN(Evsac);
};
}  // namespace GraphSfM

#endif  // RANSAC_EVSAC_H
