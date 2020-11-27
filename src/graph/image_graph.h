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

#ifndef SRC_GRAPH_IMAGE_GRAPH_H_
#define SRC_GRAPH_IMAGE_GRAPH_H_

#include <glog/logging.h>

#include <unordered_map>
#include <utility>
#include <vector>

#include "graph/union_find.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

class ImageGraph {
 public:
  inline const std::vector<image_t>& ImageIds() const;
  inline std::vector<image_t>& ImageIds();

  inline const std::vector<ImagePair>& ImagePairs() const;
  inline std::vector<ImagePair>& ImagePairs();

  inline const std::vector<int>& Scores() const;
  inline std::vector<int>& Scores();

  inline const std::unordered_map<image_t, std::string> ImageIdToName() const;
  inline std::unordered_map<image_t, std::string>& ImageIdToName();

  inline const std::unordered_map<std::string, image_t> ImageNameToId() const;
  inline std::unordered_map<std::string, image_t>& ImageNameToId();

  inline bool AddImage(const image_t image_id, const std::string& image_name);
  inline bool AddImagePair(const ImagePair& image_pair);

  void ExtractLargestCC();

  void GetImagePairsSubset(
      const std::vector<image_t>& image_ids,
      std::vector<ImagePair>& image_pairs);
  void GetImagePairsSubset(
      const std::unordered_set<image_t>& image_ids,
      std::vector<ImagePair>& image_pairs);

  void OutputSVG(const std::string& filename);

 protected:
  std::vector<image_t> image_ids_;
  std::vector<ImagePair> image_pairs_;
  std::vector<int> scores_;
  std::unordered_map<image_t, std::string> image_id_to_name_;
  std::unordered_map<std::string, image_t> image_name_to_id_;
};

inline const std::vector<image_t>& ImageGraph::ImageIds() const {
  return image_ids_;
}

inline std::vector<image_t>& ImageGraph::ImageIds() { return image_ids_; }

inline const std::vector<ImagePair>& ImageGraph::ImagePairs()
    const {
  return image_pairs_;
}

inline std::vector<ImagePair>& ImageGraph::ImagePairs() {
  return image_pairs_;
}

inline const std::vector<int>& ImageGraph::Scores() const { return scores_; }

inline std::vector<int>& ImageGraph::Scores() { return scores_; }

inline const std::unordered_map<image_t, std::string>
ImageGraph::ImageIdToName() const {
  return image_id_to_name_;
}

inline std::unordered_map<image_t, std::string>& ImageGraph::ImageIdToName() {
  return image_id_to_name_;
}

inline const std::unordered_map<std::string, image_t>
ImageGraph::ImageNameToId() const {
  return image_name_to_id_;
}

inline std::unordered_map<std::string, image_t>& ImageGraph::ImageNameToId() {
  return image_name_to_id_;
}

inline bool ImageGraph::AddImage(const image_t image_id,
                                 const std::string& image_name) {
  if (image_id_to_name_.count(image_id) > 0) {
    return false;
  }

  image_ids_.push_back(image_id);
  image_id_to_name_.emplace(image_id, image_name);
  image_name_to_id_.emplace(image_name, image_id);

  return false;
}

inline bool ImageGraph::AddImagePair(
    const ImagePair& image_pair) {
  image_pairs_.emplace_back(image_pair);
  return true;
}

}  // namespace DAGSfM

#endif