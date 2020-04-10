#include "clustering/spectral_cluster.h"
#include "sfm/types.h"
#include "util/random.h"
#include "util/map_util.h"

#include <iostream>
#include <unordered_set>
#include <gtest/gtest.h>

using namespace GraphSfM;
using namespace colmap;

class ImageClusteringTest : public ::testing::Test
{
protected:
    std::vector<image_t> image_ids_;
    std::vector<std::pair<int, int>> view_pairs_;
    std::vector<int> weights_;

public:
    void TestImageClustering(const int num_views,
                             const int num_view_pairs,
                             const int num_partitions)
    {
        ASSERT_LE(num_views + 1, num_view_pairs);

        // Initialize images
        for (int i = 0; i < num_views; i++) {
            image_ids_.push_back(i);
        }

        // Initialize view pairs
        LOG(INFO) << "Initializing view pairs...";
        CreateViewPairsFromSpanningTree(num_view_pairs);

        SpectralCluster cluster;
        cluster.ComputeCluster(view_pairs_, weights_, num_partitions);
    }

protected:
    void SetUp() { }

    void CreateViewPairsFromSpanningTree(const int num_view_pairs)
    {
        std::unordered_set<ViewIdPair> view_pairs;
        RandomNumberGenerator rng;
        for (size_t i = 1; i < image_ids_.size(); i++) {
            const ViewIdPair view_id_pair(i - 1, i);
            view_pairs.insert(view_id_pair);
            view_pairs_.push_back(view_id_pair);
            weights_.push_back(rng.RandInt(30, 100));
        }

        // Add random edges
        while (view_pairs_.size() < (uint)num_view_pairs) {
            const ViewIdPair view_id_pair(rng.RandInt(0, image_ids_.size() - 1),
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

TEST_F(ImageClusteringTest, TestThirtyImages)
{
    TestImageClustering(30, 40, 3);
}

TEST_F(ImageClusteringTest, TestOneHundredImages)
{
    TestImageClustering(100, 2000, 10);
}

TEST_F(ImageClusteringTest, TestOneThousandImages)
{
   TestImageClustering(1000, 100000, 10);
}

TEST_F(ImageClusteringTest, TestFiveThousandImages)
{
    TestImageClustering(5000, 500000, 10);
}

TEST_F(ImageClusteringTest, TestTenThousandImages)
{
    TestImageClustering(10000, 1000000, 20);
}

TEST_F(ImageClusteringTest, TestOneHundredThousandImages)
{
    TestImageClustering(100000, 10000000, 20);
}
