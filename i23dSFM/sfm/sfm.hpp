// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_SFM_HPP
#define I23DSFM_SFM_HPP

#include "i23dSFM/types.hpp"
#include "i23dSFM/numeric/numeric.h"

//-----------------
// SfM data
//-----------------
#include "i23dSFM/sfm/sfm_data.hpp"
#include "i23dSFM/sfm/sfm_data_utils.hpp"
#include "i23dSFM/sfm/sfm_data_io.hpp"
#include "i23dSFM/sfm/sfm_data_filters.hpp"
#include "i23dSFM/sfm/sfm_data_filters_frustum.hpp"
#include "i23dSFM/sfm/sfm_data_BA.hpp"
#include "i23dSFM/sfm/sfm_data_BA_ceres.hpp"

#include "i23dSFM/sfm/sfm_filters.hpp"
#include "i23dSFM/sfm/sfm_data_triangulation.hpp"

//-----------------
// SfM pipelines
//-----------------
#include "i23dSFM/sfm/sfm_report.hpp"
#include "i23dSFM/sfm/pipelines/sfm_engine.hpp"
#include "i23dSFM/sfm/pipelines/sfm_features_provider.hpp"
#include "i23dSFM/sfm/pipelines/sfm_regions_provider.hpp"
#include "i23dSFM/sfm/pipelines/sfm_matches_provider.hpp"

#include "i23dSFM/sfm/pipelines/sfm_robust_model_estimation.hpp"

#include "i23dSFM/sfm/pipelines/sequential/sequential_SfM.hpp"

#include "i23dSFM/sfm/pipelines/global/sfm_global_reindex.hpp"
#include "i23dSFM/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"

#include "i23dSFM/sfm/pipelines/hybrid/hybrid_SfM.hpp"

#include "i23dSFM/sfm/pipelines/structure_from_known_poses/structure_estimator.hpp"

#include "i23dSFM/sfm/pipelines/localization/SfM_Localizer.hpp"
#include "i23dSFM/sfm/pipelines/localization/SfM_Localizer_Single_3DTrackObservation_Database.hpp"

#endif // I23DSFM_SFM_HPP
