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

#include "clustering/image_clustering.h"

#include <gtest/gtest.h>

#include <iostream>

#include "util/map_util.h"
#include "util/random.h"
#include "util/types.h"

using namespace colmap;
using namespace DAGSfM;

class ImageClusteringTest : public ::testing::Test {
 protected:
  std::vector<image_t> image_ids_;
  std::unordered_map<ImagePair, int> view_pairs_;

 public:
  void TestImageClustering(const int num_views, const int num_view_pairs,
                           const ImageClustering::Options& options) {
    ASSERT_LE(num_views + 1, num_view_pairs);

    // Initialize images
    for (int i = 0; i < num_views; i++) {
      image_ids_.push_back(i);
    }

    // Initialize view pairs
    LOG(INFO) << "Initializing view pairs...";
    CreateViewPairsFromSpanningTree(num_view_pairs);

    ImageCluster image_cluster;
    image_cluster.image_ids = image_ids_;
    image_cluster.edges = view_pairs_;

    ImageClustering image_clustering(options, image_cluster);
    // image_clustering.CutAndExpand();
    image_clustering.Cut();
    image_clustering.Expand();

    std::vector<ImageCluster> inter_clusters =
        image_clustering.GetInterClusters();
    LOG(INFO) << inter_clusters.size() << " clusters\n";
    for (uint i = 0; i < inter_clusters.size(); i++) {
      const auto& inter_cluster = inter_clusters[i];
      // CHECK_LE(inter_cluster.image_ids.size(), options.relax_ratio *
      // options.num_images_ub);
      LOG(INFO) << "Cluster " << i << ": \n";
      inter_cluster.ShowInfo();
    }

    image_clustering.OutputClusteringSummary();
  }

 protected:
  void SetUp() override {}

  void CreateViewPairsFromSpanningTree(const int num_view_pairs) {
    RandomNumberGenerator rng;
    for (size_t i = 1; i < image_ids_.size(); i++) {
      const ImagePair view_id_pair(i - 1, i);
      view_pairs_[view_id_pair] = rng.RandInt(30, 100);
    }

    // Add random edges
    while (view_pairs_.size() < (uint)num_view_pairs) {
      const ImagePair view_id_pair(rng.RandInt(0, image_ids_.size() - 1),
                                   rng.RandInt(0, image_ids_.size() - 1));

      // Ensure the first id is smaller than second id &&
      // do not add the views that already exists
      if (view_id_pair.first >= view_id_pair.second ||
          ContainsKey(view_pairs_, view_id_pair)) {
        continue;
      }

      view_pairs_[view_id_pair] = rng.RandInt(30, 100);
    }
  }
};

TEST_F(ImageClusteringTest, TestThirtyImages) {
  ImageClustering::Options options;
  options.num_images_ub = 10;
  options.completeness_ratio = 0.3;

  TestImageClustering(30, 40, options);
}

TEST_F(ImageClusteringTest, TestOneHundredImages) {
  ImageClustering::Options options;
  options.num_images_ub = 30;
  options.completeness_ratio = 0.3;

  TestImageClustering(100, 2000, options);
}

TEST_F(ImageClusteringTest, TestOneThousandImages) {
  const int num_views = 1000;
  const int num_edges = 400000;

  ImageClustering::Options options;
  options.num_images_ub = 100;
  options.completeness_ratio = 0.3;
  options.image_overlap = 15;
  options.max_num_cluster_pairs = num_views / options.num_images_ub + 3;

  TestImageClustering(num_views, num_edges, options);
}

TEST_F(ImageClusteringTest, TestFiveThousandImages) {
  ImageClustering::Options options;
  options.num_images_ub = 500;
  options.completeness_ratio = 0.3;

  TestImageClustering(5000, 10000000, options);
}

TEST_F(ImageClusteringTest, TestTenThousandImages) {
  const int num_views = 10000;
  const int num_edges = 10000000;

  ImageClustering::Options options;
  options.num_images_ub = 800;
  options.completeness_ratio = 0.3;
  options.image_overlap = 15;
  options.max_num_cluster_pairs = num_views / options.num_images_ub + 3;

  TestImageClustering(num_views, num_edges, options);
}

TEST_F(ImageClusteringTest, TestOneHundredThousandImages) {
  const int num_views = 100000;
  const int num_edges = 1e7;

  ImageClustering::Options options;
  options.num_images_ub = 500;
  options.completeness_ratio = 0.3;
  options.image_overlap = 15;
  options.max_num_cluster_pairs = num_views / options.num_images_ub + 3;

  TestImageClustering(num_views, num_edges, options);
}
