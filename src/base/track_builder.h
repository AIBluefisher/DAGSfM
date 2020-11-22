// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef COLMAP_SRC_BASE_TRACK_BUILDER_H
#define COLMAP_SRC_BASE_TRACK_BUILDER_H

#include <unordered_map>
#include <vector>

#include "base/track.h"
#include "graph/union_find.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

// Build tracks from feature correspondences across multiple images. Tracks are
// created with the connected components algorithm and have a maximum allowable
// size. If there are multiple features from one image in a track, we do not do
// any intelligent selection and just arbitrarily choose a feature to drop so
// that the tracks are consistent.
class TrackBuilder {
 public:
  // Build tracks for a given series of track elements.
  void Build(const std::vector<TrackElement>& track_elements,
             const std::vector<std::pair<track_t, track_t>>& pair_ids);

  // Remove bad tracks that are too short or have ids collision.
  bool Filter(const int min_track_length = 2, const int max_track_length = 100);

  // Return the number of connected set in the Union Find structure.
  size_t NumTracks() const;

  // Extract consistent tracks.
  std::unordered_map<track_t, std::vector<TrackElement>> GetConsistentTracks()
      const;

 private:
  std::unordered_map<track_t, std::vector<TrackElement>> consistent_tracks_;
};

}  // namespace DAGSfM

#endif