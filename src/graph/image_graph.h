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

  inline const std::vector<std::pair<image_t, image_t>>& ImagePairs() const;
  inline std::vector<std::pair<image_t, image_t>>& ImagePairs();

  inline const std::vector<int>& Scores() const;
  inline std::vector<int>& Scores();

  inline const std::unordered_map<image_t, std::string> ImageIdToName() const;
  inline std::unordered_map<image_t, std::string>& ImageIdToName();

  inline const std::unordered_map<std::string, image_t> ImageNameToId() const;
  inline std::unordered_map<std::string, image_t>& ImageNameToId();

  inline bool AddImage(const image_t image_id, const std::string& image_name);
  inline bool AddImagePair(const std::pair<image_t, image_t>& image_pair);

  inline void ExtractLargestCC();

  inline void GetImagePairsSubset(
      const std::vector<image_t>& image_ids,
      std::vector<std::pair<image_t, image_t>>& image_pairs);
  inline void GetImagePairsSubset(
      const std::unordered_set<image_t>& image_ids,
      std::vector<std::pair<image_t, image_t>>& image_pairs);

 protected:
  std::vector<image_t> image_ids_;
  std::vector<std::pair<image_t, image_t>> image_pairs_;
  std::vector<int> scores_;
  std::unordered_map<image_t, std::string> image_id_to_name_;
  std::unordered_map<std::string, image_t> image_name_to_id_;
};

inline const std::vector<image_t>& ImageGraph::ImageIds() const {
  return image_ids_;
}

inline std::vector<image_t>& ImageGraph::ImageIds() { return image_ids_; }

inline const std::vector<std::pair<image_t, image_t>>& ImageGraph::ImagePairs()
    const {
  return image_pairs_;
}

inline std::vector<std::pair<image_t, image_t>>& ImageGraph::ImagePairs() {
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
    const std::pair<image_t, image_t>& image_pair) {
  image_pairs_.emplace_back(image_pair);
  return true;
}

inline void ImageGraph::ExtractLargestCC() {
  graph::UnionFind uf(image_ids_.size());
  std::vector<size_t> tmp_nodes(image_ids_.begin(), image_ids_.end());
  uf.InitWithNodes(tmp_nodes);

  for (auto image_pair : image_pairs_) {
    uf.Union(image_pair.first, image_pair.second);
  }

  std::unordered_map<size_t, std::vector<image_t>> components;
  for (auto image_id : image_ids_) {
    const size_t parent_id = uf.FindRoot(image_id);
    components[parent_id].push_back(image_id);
  }

  size_t num_largest_component = 0;
  size_t largest_component_id;
  for (const auto& it : components) {
    if (num_largest_component < it.second.size()) {
      num_largest_component = it.second.size();
      largest_component_id = it.first;
    }
  }

  image_ids_.clear();
  image_ids_.assign(components[largest_component_id].begin(),
                    components[largest_component_id].end());
  image_ids_.shrink_to_fit();
  std::sort(image_ids_.begin(), image_ids_.end());

  LOG(INFO) << "There are " << components.size() << " connected components.";
  int num_small_connected_components = 0;
  for (auto component : components) {
    if (component.second.size() < image_ids_.size()) {
      num_small_connected_components++;
    } else {
      LOG(INFO) << "Component #" << component.first << "# has "
                << component.second.size() << " images.";
    }
  }
  LOG(INFO) << "There are " << num_small_connected_components
            << " small connected components are discarded.";
}

inline void ImageGraph::GetImagePairsSubset(
    const std::vector<image_t>& image_ids,
    std::vector<std::pair<image_t, image_t>>& image_pairs) {
  std::unordered_set<image_t> image_sets(image_ids.begin(), image_ids.end());
  GetImagePairsSubset(image_sets, image_pairs);
}

inline void ImageGraph::GetImagePairsSubset(
    const std::unordered_set<image_t>& image_ids,
    std::vector<std::pair<image_t, image_t>>& image_pairs) {
  for (auto image_pair : image_pairs_) {
    if (image_ids.count(image_pair.first) > 0 &&
        image_ids.count(image_pair.second) > 0) {
      image_pairs.push_back(image_pair);
    }
  }
}

}  // namespace DAGSfM

#endif