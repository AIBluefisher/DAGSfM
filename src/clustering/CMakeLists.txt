set(FOLDER_NAME "clustering")

COLMAP_ADD_SOURCES(
    cluster.h
    image_clustering.h image_clustering.cpp
    ncut_cluster.h ncut_cluster.cpp
    scene_clustering.h scene_clustering.cc
)

COLMAP_ADD_TEST(scene_clustering_test scene_clustering_test.cc)
COLMAP_ADD_TEST(image_clustering_test image_clustering_test.cpp)