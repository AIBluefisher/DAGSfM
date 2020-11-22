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

#ifndef SRC_GRAPH_SIMILARITY_GRAPH_H_
#define SRC_GRAPH_SIMILARITY_GRAPH_H_

#include <glog/logging.h>

#include <cstdlib>
#include <utility>
#include <vector>

#include "feature/feature.h"
#include "feature/matching.h"
#include "graph/image_graph.h"
#include "util/threading.h"
#include "util/timer.h"

using namespace colmap;

namespace DAGSfM {

struct VocabSimilaritySearchOptions {
  // Number of images to retrieve for each query image.
  int num_images = 100;

  // Number of nearest neighbors to retrieve per query feature.
  int num_nearest_neighbors = 5;

  // Number of nearest-neighbor checks to use in retrieval.
  int num_checks = 256;

  // How many images to return after spatial verification. Set to 0 to turn off
  // spatial verification.
  int num_images_after_verification = 0;

  // The maximum number of features to use for indexing an image. If an
  // image has more features, only the largest-scale features will be indexed.
  int max_num_features = -1;

  int num_threads = 8;

  // Path to the vocabulary tree.
  std::string vocab_tree_path = "";

  void Check();
};

class VocabSimilarityGraph : public Thread, public ImageGraph {
 public:
  VocabSimilarityGraph(const VocabSimilaritySearchOptions& options,
                       const Database& database);

  void Run() override;

 private:
  VocabSimilaritySearchOptions options_;

  FeatureMatcherCache cache_;
};

struct MirrorSimilaritySearchOptions {
  // Number of images to retrieve for each query image.
  int num_images = 100;

  // Script path to execute similarity search.
  std::string script_path = "";

  // dataset path.
  std::string dataset_path = "";

  // Output directory of matching pairs.
  std::string output_dir = "";

  // Path to the mirror library.
  std::string mirror_path = "";

  void Check();
};

class MirrorSimilarityGraph : public Thread, public ImageGraph {
 public:
  MirrorSimilarityGraph(const MirrorSimilaritySearchOptions& options);

  void Run() override;

 private:
  MirrorSimilaritySearchOptions options_;

  void LoadSearchResults();
};

}  // namespace DAGSfM

#endif