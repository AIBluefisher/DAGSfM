
// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "i23dSFM/matching/matcher_type.hpp"
#include "i23dSFM/matching/indMatch.hpp"
#include "i23dSFM/matching_image_collection/Pair_Builder.hpp"
#include "i23dSFM/sfm/sfm_data.hpp"
#include "i23dSFM/sfm/pipelines/sfm_regions_provider.hpp"

#include <string>
#include <vector>

namespace i23dSFM {
namespace matching_image_collection {

/// Implementation of an Image Collection Matcher
/// Compute putative matches between a collection of pictures
class Matcher
{
  public:
  Matcher() {};

  virtual ~Matcher() {};

  /// Find corresponding points between some pair of view Ids
  virtual void Match(
    const sfm::SfM_Data & sfm_data,
    const std::shared_ptr<sfm::Regions_Provider> & regions_provider,
    const Pair_Set & pairs, // list of pair to consider for matching
    matching::PairWiseMatches & map_putatives_matches // the output pairwise photometric corresponding points
    )const = 0;
};

} // namespace i23dSFM
} // namespace matching_image_collection
