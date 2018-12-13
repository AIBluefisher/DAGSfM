set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/Linux-x86_64-Release)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# compile options for vocabulary tree example (need c++11)
add_executable(i23dSFM_image_search image_search.cpp)
set_property(TARGET i23dSFM_image_search PROPERTY CXX_STANDARD 11)
set_property(TARGET i23dSFM_image_search PROPERTY CXX_STANDARD_REQUIRED on)
# set_property(TAEGET i23dSFM_image_search PROPERTY FOLDER I23dSFM/software)
target_link_libraries(i23dSFM_image_search ${CMAKE_THREAD_LIBS_INIT} vot_vocabtree vot_utils ${GFLAGS_LIBRARIES})
install (TARGETS i23dSFM_image_search DESTINATION bin/)

# graph_partition
add_executable(i23dSFM_graph_partition graph_partition.cpp ../graph/image_graph.cpp)
set_property(TARGET i23dSFM_graph_partition PROPERTY CXX_STANDARD 11)
set_property(TARGET i23dSFM_graph_partition PROPERTY CXX_STANDARD_REQUIRED on)
# set_property(TAEGET i23dSFM_graph_partition PROPERTY FOLDER I23dSFM/software)
target_link_libraries(i23dSFM_graph_partition ${CMAKE_THREAD_LIBS_INIT} vot_utils)
install (TARGETS i23dSFM_graph_partition DESTINATION bin/)

add_executable(i23dSFM_query_expansion query_expansion.cpp ../utils/io_utils.cpp ../graph/image_graph.cpp
../utils/data_structures.cpp)
set_property(TARGET i23dSFM_query_expansion PROPERTY CXX_STANDARD 11)
set_property(TARGET i23dSFM_query_expansion PROPERTY CXX_STANDARD_REQUIRED on)
# set_property(TAEGET i23dSFM_query_expansion PROPERTY FOLDER I23dSFM/software)
install (TARGETS i23dSFM_query_expansion DESTINATION bin/)

add_executable(i23dSFM_sequence_match sequence_match.cpp ../utils/io_utils.cpp)
set_property(TARGET i23dSFM_sequence_match PROPERTY CXX_STANDARD 11)
set_property(TARGET i23dSFM_sequence_match PROPERTY CXX_STANDARD_REQUIRED on)
# set_property(TAEGET i23dSFM_sequence_match PROPERTY FOLDER I23dSFM/software)
install (TARGETS i23dSFM_sequence_match DESTINATION bin/)

add_executable(i23dSFM_compute_stats compute_stats.cpp ../utils/io_utils.cpp)
set_property(TARGET i23dSFM_compute_stats PROPERTY CXX_STANDARD 11)
set_property(TARGET i23dSFM_compute_stats PROPERTY CXX_STANDARD_REQUIRED on)
# set_property(TAEGET i23dSFM_compute_stats PROPERTY FOLDER I23dSFM/software)
install (TARGETS i23dSFM_compute_stats DESTINATION bin/)

add_executable(i23dSFM_svt svt.cpp ../utils/io_utils.cpp)
set_property(TARGET i23dSFM_svt PROPERTY CXX_STANDARD 11)
set_property(TARGET i23dSFM_svt PROPERTY CXX_STANDARD_REQUIRED on)
# set_property(TAEGET i23dSFM_svt PROPERTY FOLDER I23dSFM/software)
install (TARGETS i23dSFM_svt DESTINATION bin/)

add_executable(i23dSFM_euclidean_matrix euclidean_matrix.cpp ../utils/io_utils.cpp)
set_property(TARGET i23dSFM_euclidean_matrix PROPERTY CXX_STANDARD 11)
set_property(TARGET i23dSFM_euclidean_matrix PROPERTY CXX_STANDARD_REQUIRED on)
# set_property(TAEGET i23dSFM_euclidean_matrix PROPERTY FOLDER I23dSFM/software)
install (TARGETS i23dSFM_euclidean_matrix DESTINATION bin/)

# output a single ranked list for each query sift file
add_executable(i23dSFM_web_search web_search.cpp)
set_property(TARGET i23dSFM_web_search PROPERTY CXX_STANDARD 11)
set_property(TARGET i23dSFM_web_search PROPERTY CXX_STANDARD_REQUIRED on)
# set_property(TAEGET i23dSFM_web_search PROPERTY FOLDER I23dSFM/software)
target_link_libraries(i23dSFM_web_search vot_utils vot_vocabtree ${GFLAGS_LIBRARIES})
install (TARGETS i23dSFM_web_search DESTINATION bin/)

# using vlfeat feature in libvot
add_executable(i23dSFM_libvot_feature libvot_feature.cpp)
# set_property(TAEGET i23dSFM_libvot_feature PROPERTY FOLDER I23dSFM/software)
target_link_libraries(i23dSFM_libvot_feature ${GFLAGS_LIBRARIES} ${GLOG_LIBRARY} vot_feature vot_utils)
target_include_directories(i23dSFM_libvot_feature PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/>
  $<INSTALL_INTERFACE:include>)
install (TARGETS i23dSFM_libvot_feature DESTINATION bin/)
