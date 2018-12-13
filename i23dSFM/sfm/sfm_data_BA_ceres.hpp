// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_SFM_DATA_BA_CERES_HPP
#define I23DSFM_SFM_DATA_BA_CERES_HPP

#include "i23dSFM/sfm/sfm_data.hpp"
#include "i23dSFM/sfm/sfm_data_BA.hpp"
#include "i23dSFM/sfm/sfm_data_BA_ceres_camera_functor.hpp"
#include "ceres/ceres.h"

namespace i23dSFM {
namespace sfm {

/// Create the appropriate cost functor according the provided input camera intrinsic model
ceres::CostFunction * IntrinsicsToCostFunction(
  cameras::IntrinsicBase * intrinsic,
  const Vec2 & observation);

class Bundle_Adjustment_Ceres : public Bundle_Adjustment
{
  public:
  struct BA_options
  {
    bool _bVerbose;
    unsigned int _nbThreads;
    bool _bCeres_Summary;
    ceres::LinearSolverType _linear_solver_type;
    ceres::PreconditionerType _preconditioner_type;
    ceres::SparseLinearAlgebraLibraryType _sparse_linear_algebra_library_type;

    BA_options(const bool bVerbose = true, bool bmultithreaded = true);
  };
  private:
    BA_options _i23dSFM_options;

  public:
  Bundle_Adjustment_Ceres(Bundle_Adjustment_Ceres::BA_options options = BA_options());

  bool Adjust(
    SfM_Data & sfm_data,            // the SfM scene to refine
    bool bRefineRotations = true,   // tell if pose rotations will be refined
    bool bRefineTranslations = true,// tell if the pose translation will be refined
    bool bRefineIntrinsics = true,  // tell if the camera intrinsic will be refined
    bool bRefineStructure = true);  // tell if the structure will be refined
};

} // namespace sfm
} // namespace i23dSFM

#endif // I23DSFM_SFM_DATA_BA_CERES_HPP
