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

#include "base/track_builder.h"

#include <glog/logging.h>

namespace DAGSfM {

void TrackBuilder::Build(
    const std::vector<TrackElement>& track_elements,
    const std::vector<std::pair<track_t, track_t>>& pair_ids) {
  LOG(INFO) << "Build tracks";
  graph::UnionFind uf(track_elements.size());
  for (size_t i = 0; i < track_elements.size(); i++) {
    uf.Union(pair_ids[i].first, pair_ids[i].second);
  }

  for (size_t i = 0; i < track_elements.size(); i++) {
    const size_t parent_id = uf.FindRoot(i);
    consistent_tracks_[parent_id].emplace_back(track_elements[i]);
  }
}

bool TrackBuilder::Filter(const int min_track_length,
                          const int max_track_length) {
  LOG(INFO) << "Filter tracks.";
  size_t num_small_tracks = 0;
  size_t num_inconsistent_track_elements = 0;
  LOG(INFO) << "tracks size: " << consistent_tracks_.size();
  for (auto track_it = consistent_tracks_.begin();
       track_it != consistent_tracks_.end();) {
    LOG(INFO) << track_it->first;
    // If track.length < min_track_length or track.length > max_track_length,
    // we should discard this track.
    if (static_cast<int>(track_it->second.size()) < min_track_length ||
        static_cast<int>(track_it->second.size()) > max_track_length) {
      LOG(INFO) << "Remove short tracks";
      consistent_tracks_.erase(track_it++);
      num_small_tracks++;
      continue;
    }

    const std::vector<TrackElement>& candidate_track = track_it->second;
    std::vector<TrackElement> consistent_track;
    consistent_track.reserve(track_it->second.size());

    LOG(INFO) << "Remove inconsistent tracks";
    std::unordered_set<image_t> image_ids;
    for (size_t i = 0; i < candidate_track.size(); i++) {
      const TrackElement& track_element = candidate_track[i];
      // Do not add the track_element if the track already contains a
      // track element from the same image.
      if (image_ids.count(track_element.image_id) != 0) {
        num_inconsistent_track_elements++;
        continue;
      }

      image_ids.insert(track_element.image_id);
      consistent_track.emplace_back(track_element);
    }

    if (candidate_track.size() != consistent_track.size()) {
      LOG(INFO) << "Re-assign tracks elements";
      consistent_tracks_[track_it->first].clear();
      consistent_tracks_[track_it->first].assign(consistent_track.begin(),
                                                 consistent_track.end());
    }

    ++track_it;
  }

  LOG(INFO) << num_small_tracks << " small tracks are removed";
  LOG(INFO) << num_inconsistent_track_elements
            << " inconsistent track element are removed.";
  return true;
}

size_t TrackBuilder::NumTracks() const { return consistent_tracks_.size(); }

std::unordered_map<track_t, std::vector<TrackElement>>
TrackBuilder::GetConsistentTracks() const {
  return consistent_tracks_;
}

}  // namespace DAGSfM