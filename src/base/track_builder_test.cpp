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

#include <string>
#include <unordered_set>
#include <vector>

#include "base/track.h"
#include "gtest/gtest.h"
#include "util/types.h"

namespace {

std::vector<TrackElement> GenerateTrackElements(const int image_num,
                                                const int feature_num) {
  std::vector<TrackElement> track_elements;
  for (int i = 0; i < image_num; i++) {
    for (int k = 0; k < feature_num; k++)
      track_elements.emplace_back(TrackElement(i, k));
  }
  return track_elements;
}

std::vector<std::pair<track_t, track_t>> GenerateTrackPairs(
    const int image_num, const int feature_num) {
  std::vector<std::pair<track_t, track_t>> track_pairs = {
      {0, 3}, {3, 8}, {8, 11}, {2, 5}, {5, 6}, {1, 4}, {7, 9}};
  return track_pairs;
}

}  // namespace

namespace DAGSfM {
static const int kMinTrackLength = 2;

// Perfect tracks.
TEST(TrackBuilder, ConsistentTracks) {
  static const int kMaxTrackLength = 10;
  static const int kNumCorrespondences = 3;

  const std::vector<TrackElement> track_elements =
      GenerateTrackElements(4, kNumCorrespondences);
  const std::vector<std::pair<track_t, track_t>> track_pairs = {
      {0, 3}, {3, 8}, {8, 11}, {2, 5}, {5, 6}, {1, 4}, {7, 9}};

  TrackBuilder track_builder;
  track_builder.Build(track_elements, track_pairs);
  track_builder.Filter(kMinTrackLength, kMaxTrackLength);

  std::unordered_map<track_t, std::vector<TrackElement>> tracks =
      track_builder.GetConsistentTracks();
  EXPECT_EQ(tracks.size(), 4);

  // for (auto track_it : tracks) {
  //     std::cout << track_it.first << ": ";
  //     for (auto track_element : track_it.second) {
  //         std::cout << "(" << track_element.image_id
  //                   << ", " << track_element.point2D_idx << "); ";
  //     }
  //     std::cout << "\n";
  // }
}

// Inconsistent tracks.
TEST(TrackBuilder, InconsistentTracks) {
  static const int kMaxTrackLength = 10;
  static const int kNumCorrespondences = 3;

  const std::vector<TrackElement> track_elements =
      GenerateTrackElements(4, kNumCorrespondences);
  const std::vector<std::pair<track_t, track_t>> track_pairs = {
      {0, 3}, {3, 8}, {8, 11}, {2, 5}, {5, 6}, {1, 4}, {7, 9}, {10, 0}};

  TrackBuilder track_builder;
  track_builder.Build(track_elements, track_pairs);
  track_builder.Filter(kMinTrackLength, kMaxTrackLength);

  std::unordered_map<track_t, std::vector<TrackElement>> tracks =
      track_builder.GetConsistentTracks();
  EXPECT_EQ(tracks.size(), 4);

  // for (auto track_it : tracks) {
  //     std::cout << track_it.first << ": ";
  //     for (auto track_element : track_it.second) {
  //         std::cout << "(" << track_element.image_id
  //                   << ", " << track_element.point2D_idx << "); ";
  //     }
  //     std::cout << "\n";
  // }
}

}  // namespace DAGSfM
