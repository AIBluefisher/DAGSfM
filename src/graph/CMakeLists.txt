set(FOLDER_NAME "graph")

COLMAP_ADD_SOURCES(
    graph_cut.h graph_cut.cpp
    graph.h graph.inl
    image_graph.h
    similarity_graph.h similarity_graph.cpp
    view_graph.h view_graph.cpp
    union_find.h union_find.cpp
)

# COLMAP_ADD_TEST(graph_cut_test graph_cut_test.cpp)
COLMAP_ADD_TEST(graph_test graph_test.cpp)
COLMAP_ADD_TEST(union_find_test union_find_test.cpp)