#include "clustering/image_clustering.h"
#include "sfm/types.h"
#include "util/random.h"
#include "util/map_util.h"

#include <iostream>
#include <gtest/gtest.h>

using namespace colmap;
using namespace GraphSfM;

class ImageClusteringTest : public ::testing::Test
{
protected:
    std::vector<image_t> image_ids_;
    std::unordered_map<ViewIdPair, int> view_pairs_;

public:
    void TestImageClustering(const int num_views,
                             const int num_view_pairs,
                             const ImageClustering::Options& options)
    {
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

        std::vector<ImageCluster> inter_clusters = image_clustering.GetInterClusters();
        LOG(INFO) << inter_clusters.size() << " clusters\n";
        for (uint i = 0; i < inter_clusters.size(); i++) {
            const auto& inter_cluster = inter_clusters[i];
            // CHECK_LE(inter_cluster.image_ids.size(), options.relax_ratio * options.num_images_ub);
            LOG(INFO) << "Cluster " << i << ": \n";
            inter_cluster.ShowInfo();
        }

        image_clustering.OutputClusteringSummary();
    }

protected:
    void SetUp() override { }

    void CreateViewPairsFromSpanningTree(const int num_view_pairs)
    {
        RandomNumberGenerator rng;
        for (size_t i = 1; i < image_ids_.size(); i++) {
            const ViewIdPair view_id_pair(i - 1, i);
            view_pairs_[view_id_pair] = rng.RandInt(30, 100);
        }

        // Add random edges
        while (view_pairs_.size() < (uint)num_view_pairs) {
            const ViewIdPair view_id_pair(rng.RandInt(0, image_ids_.size() - 1),
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

TEST_F(ImageClusteringTest, TestThirtyImages)
{
    ImageClustering::Options options;
    options.num_images_ub = 10;
    options.completeness_ratio = 0.3;

    TestImageClustering(30, 40, options);
}

TEST_F(ImageClusteringTest, TestOneHundredImages)
{
    ImageClustering::Options options;
    options.num_images_ub = 30;
    options.completeness_ratio = 0.3;

    TestImageClustering(100, 2000, options);
}

TEST_F(ImageClusteringTest, TestOneThousandImages)
{
    const int num_views = 1000;
    const int num_edges = 400000;

    ImageClustering::Options options;
    options.num_images_ub = 100;
    options.completeness_ratio = 0.3;
    options.image_overlap = 15;
    options.max_num_cluster_pairs = num_views / options.num_images_ub + 3;

    TestImageClustering(num_views, num_edges, options);
}

TEST_F(ImageClusteringTest, TestFiveThousandImages)
{
    ImageClustering::Options options;
    options.num_images_ub = 500;
    options.completeness_ratio = 0.3;

    TestImageClustering(5000, 10000000, options);
}

TEST_F(ImageClusteringTest, TestTenThousandImages)
{
    const int num_views = 10000;
    const int num_edges = 10000000;

    ImageClustering::Options options;
    options.num_images_ub = 800;
    options.completeness_ratio = 0.3;
    options.image_overlap = 15;
    options.max_num_cluster_pairs = num_views / options.num_images_ub + 3;

    TestImageClustering(num_views, num_edges, options);
}

TEST_F(ImageClusteringTest, TestOneHundredThousandImages)
{
    const int num_views = 100000;
    const int num_edges = 1e7;

    ImageClustering::Options options;
    options.num_images_ub = 500;
    options.completeness_ratio = 0.3;
    options.image_overlap = 15;
    options.max_num_cluster_pairs = num_views / options.num_images_ub + 3;

    TestImageClustering(num_views, num_edges, options);
}
