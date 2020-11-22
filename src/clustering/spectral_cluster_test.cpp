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

#include "clustering/spectral_cluster.h"

#include <gtest/gtest.h>

#include <iostream>
#include <unordered_set>

#include "util/map_util.h"
#include "util/random.h"
#include "util/types.h"

using namespace DAGSfM;
using namespace colmap;

class ImageClusteringTest : public ::testing::Test {
 protected:
  std::vector<image_t> image_ids_;
  std::vector<std::pair<int, int>> view_pairs_;
  std::vector<int> weights_;

 public:
  void TestImageClustering(const int num_views, const int num_view_pairs,
                           const int num_partitions) {
    ASSERT_LE(num_views + 1, num_view_pairs);

    // Initialize images
    for (int i = 0; i < num_views; i++) {
      image_ids_.push_back(i);
    }

    // Initialize view pairs
    LOG(INFO) << "Initializing view pairs...";
    CreateViewPairsFromSpanningTree(num_view_pairs);

    SpectralCluster cluster;
    cluster.InitIGraph(view_pairs_, weights_);
    cluster.ComputeCluster(view_pairs_, weights_, num_partitions);
  }

 protected:
  void SetUp() {}

  void CreateViewPairsFromSpanningTree(const int num_view_pairs) {
    std::unordered_set<ImagePair> view_pairs;
    RandomNumberGenerator rng;
    for (size_t i = 1; i < image_ids_.size(); i++) {
      const ImagePair view_id_pair(i - 1, i);
      view_pairs.insert(view_id_pair);
      view_pairs_.push_back(view_id_pair);
      weights_.push_back(rng.RandInt(30, 100));
    }

    // Add random edges
    while (view_pairs_.size() < (uint)num_view_pairs) {
      const ImagePair view_id_pair(rng.RandInt(0, image_ids_.size() - 1),
                                   rng.RandInt(0, image_ids_.size() - 1));

      // Ensure the first id is smaller than second id &&
      // do not add the views that already exists
      if (view_id_pair.first >= view_id_pair.second ||
          ContainsKey(view_pairs, view_id_pair)) {
        continue;
      }

      view_pairs.insert(view_id_pair);
      view_pairs_.push_back(view_id_pair);
      weights_.push_back(rng.RandInt(30, 100));
    }
  }
};

TEST_F(ImageClusteringTest, TestThirtyImages) {
  TestImageClustering(30, 40, 3);
}

TEST_F(ImageClusteringTest, TestOneHundredImages) {
  TestImageClustering(100, 2000, 10);
}

TEST_F(ImageClusteringTest, TestOneThousandImages) {
  TestImageClustering(1000, 100000, 10);
}

TEST_F(ImageClusteringTest, TestFiveThousandImages) {
  TestImageClustering(5000, 500000, 10);
}

TEST_F(ImageClusteringTest, TestTenThousandImages) {
  TestImageClustering(10000, 1000000, 20);
}

TEST_F(ImageClusteringTest, TestOneHundredThousandImages) {
  TestImageClustering(100000, 10000000, 20);
}
