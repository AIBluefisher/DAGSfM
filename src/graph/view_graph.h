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

#ifndef SRC_GRAPH_VIEW_GRAPH_H_
#define SRC_GRAPH_VIEW_GRAPH_H_

#include "graph/image_graph.h"
#include "sfm/twoview_info.h"
#include "util/hash.h"

namespace DAGSfM {

class ViewGraph : public ImageGraph {
 public:
  bool AddTwoViewGeometry(const ImagePair& image_pair,
                          const TwoViewInfo& twoview_info);
  bool RemoveTwoViewGeometry(const ImagePair& image_pair);

  void TwoviewGeometriesToImagePairs();

  // Finds all cycles of size 3 (i.e., "triplets") and sets each triplet to
  // "valid" if the loop rotation error is less than 2 degree. The loop rotation
  // error is defined as the angle of the concatenated rotations (compared to
  // the identity). This is because the concatenated rotations of a perfect loop
  // should result in a zero angle loop rotation. Any view pairs that do not
  // participate in a valid triplet are removed.
  void FilterViewGraphCyclesByRotation(const double max_loop_error_degrees);

  inline std::unordered_map<ImagePair, TwoViewInfo>& TwoViewGeometries();
  inline const std::unordered_map<ImagePair, TwoViewInfo>& TwoViewGeometries()
      const;

 private:
  std::unordered_map<ImagePair, TwoViewInfo> twoview_geometries_;
};

inline std::unordered_map<ImagePair, TwoViewInfo>&
ViewGraph::TwoViewGeometries() {
  return twoview_geometries_;
}

inline const std::unordered_map<ImagePair, TwoViewInfo>&
ViewGraph::TwoViewGeometries() const {
  return twoview_geometries_;
}

}  // namespace DAGSfM

#endif