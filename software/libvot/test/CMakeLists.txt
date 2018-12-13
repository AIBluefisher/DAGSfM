# Tianwei Shen <shentianweipku@gmail.com> HKUST.
# All rights reserved.

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# set output built test executables back to RUNTIME_OUTPUT_DIRECTORY,
# which is different from src, so that add_test can find them
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})

set(UNIT_TEST_SOURCES
unit_test.cpp 
../src/graph/image_graph.cpp 
../src/utils/data_structures.cpp
../src/vocab_tree/vot_pipeline.cpp
../src/vocab_tree/clustering.cpp
../src/utils/global_params.cpp
../src/utils/io_utils.cpp
../src/vocab_tree/vocab_tree.cpp)

# compile options for unit test (need c++11)
add_executable(unit_test ${UNIT_TEST_SOURCES})
set_property(TARGET unit_test PROPERTY CXX_STANDARD 11)
set_property(TARGET unit_test PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(unit_test ${CMAKE_THREAD_LIBS_INIT} ${GTEST_LIBRARIES})
add_test(unit_test unit_test)

# tests related to opencv
if(LIBVOT_USE_OPENCV)
	# opencv basic test
    add_executable(opencv_test opencv_test.cpp)
    target_link_libraries(opencv_test ${OpenCV_LIBS})
    add_test(opencv_test opencv_test ${CMAKE_SOURCE_DIR}/test_data/lena.jpg)

	# libvot_feature test
    add_executable(libvot_feature_test libvot_feature_test.cpp)
    target_link_libraries(libvot_feature_test vot_feature vot_utils)
    add_test(libvot_feature_test libvot_feature_test ${CMAKE_SOURCE_DIR}/test_data/lena.jpg)

	# test matching functions
	add_executable(sift_match_test sift_match_test.cpp)
	target_link_libraries(sift_match_test vot_matching vot_utils vot_feature ${OpenCV_LIBS} ${GFLAGS_LIBRARIES})
	add_test(sift_match_test sift_match_test
			${CMAKE_SOURCE_DIR}/test_data/fountain_dense/0000.sift
			${CMAKE_SOURCE_DIR}/test_data/fountain_dense/0000.sift
			--opencv_feature=false
			--show_matching=false)
endif(LIBVOT_USE_OPENCV)

# tests related to cuda
if(LIBVOT_USE_CUDA)
	cuda_add_executable(cuda_basic_test cuda_basic_test.cu)
	add_test(cuda_basic_test cuda_basic_test)	# cuda hello world test
endif(LIBVOT_USE_CUDA)

# Boost basic test
if(LIBVOT_USE_BOOST)
	add_executable(boost_basic_test boost_basic_test.cpp)
	target_link_libraries(boost_basic_test ${Boost_LIBRARIES})
	add_test(boost_basic_test boost_basic_test)
endif(LIBVOT_USE_BOOST)
