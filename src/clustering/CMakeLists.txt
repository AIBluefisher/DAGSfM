set(FOLDER_NAME "clustering")

COLMAP_ADD_SOURCES(
    cluster.h cluster.cpp
    community_detection_cluster.h community_detection_cluster.cpp
    hybrid_cluster.h hybrid_cluster.cpp
    image_clustering.h image_clustering.cpp
    kmeans_cluster.h kmeans_cluster.cpp
    ncut_cluster.h ncut_cluster.cpp
    scene_clustering.h scene_clustering.cc
    spectral_cluster.h spectral_cluster.cpp
)

COLMAP_ADD_TEST(community_detection_cluster_test community_detection_cluster_test.cpp)
COLMAP_ADD_TEST(scene_clustering_test scene_clustering_test.cc)
COLMAP_ADD_TEST(image_clustering_test image_clustering_test.cpp)
COLMAP_ADD_TEST(ncut_cluster_test ncut_cluster_test.cpp)
COLMAP_ADD_TEST(spectral_cluster_test spectral_cluster_test.cpp)