
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_SFM_FEATURES_PROVIDER_HPP
#define I23DSFM_SFM_FEATURES_PROVIDER_HPP

#include <i23dSFM/types.hpp>
#include <i23dSFM/sfm/sfm_data.hpp>
#include <i23dSFM/features/features.hpp>
#include "third_party/progress/progress.hpp"

#include <memory>

namespace i23dSFM {
namespace sfm {

/// Abstract PointFeature provider (read some feature and store them as PointFeature).
/// Allow to load and return the features related to a view
struct Features_Provider
{
  /// PointFeature array per ViewId of the considered SfM_Data container
  Hash_Map<IndexT, features::PointFeatures> feats_per_view;

  virtual bool load(
    const SfM_Data & sfm_data,
    const std::string & feat_directory,
    std::unique_ptr<features::Regions>& region_type)
  {
    C_Progress_display my_progress_bar( sfm_data.GetViews().size(),
      std::cout, "\n- Features Loading -\n" );
    // Read for each view the corresponding features and store them as PointFeatures
    bool bContinue = true;
#ifdef I23DSFM_USE_OPENMP
    #pragma omp parallel
#endif
    for (Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end() && bContinue; ++iter)
    {
#ifdef I23DSFM_USE_OPENMP
    #pragma omp single nowait
#endif
      {
        const std::string sImageName = stlplus::create_filespec(sfm_data.s_root_path, iter->second.get()->s_Img_path);
        const std::string basename = stlplus::basename_part(sImageName);
        const std::string featFile = stlplus::create_filespec(feat_directory, basename, ".feat");

        std::unique_ptr<features::Regions> regions(region_type->EmptyClone());
        if (!regions->LoadFeatures(featFile))
        {
          std::cerr << "Invalid feature files for the view: " << sImageName << std::endl;
#ifdef I23DSFM_USE_OPENMP
      #pragma omp critical
#endif
          bContinue = false;
        }
#ifdef I23DSFM_USE_OPENMP
      #pragma omp critical
#endif
        {
          // save loaded Features as PointFeature
          feats_per_view[iter->second.get()->id_view] = regions->GetRegionsPositions();
          ++my_progress_bar;
        }
      }
    }
    return bContinue;
  }

  /// Return the PointFeatures belonging to the View, if the view does not exist
  ///  it return an empty PointFeature array.
  const features::PointFeatures & getFeatures(const IndexT & id_view) const
  {
    // Have an empty feature set in order to deal with non existing view_id
    static const features::PointFeatures emptyFeats = features::PointFeatures();

    Hash_Map<IndexT, features::PointFeatures>::const_iterator it = feats_per_view.find(id_view);
    if (it != feats_per_view.end())
      return it->second;
    else
      return emptyFeats;
  }
}; // Features_Provider

} // namespace sfm
} // namespace i23dSFM

#endif // I23DSFM_SFM_FEATURES_PROVIDER_HPP
