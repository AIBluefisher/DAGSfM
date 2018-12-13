
ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

file(
	GLOB_RECURSE
	image_graph_files_header
	*.hpp
)
file(
	GLOB_RECURSE
	image_graph_files_cpp
	*.cpp
)
file(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)

#Remove the future main files
list(REMOVE_ITEM image_graph_files_cpp ${REMOVEFILESUNITTEST})

ADD_LIBRARY(i23dSFM_image_graph ${image_graph_files_header} ${image_graph_files_cpp})
SET_TARGET_PROPERTIES(i23dSFM_image_graph PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")
INSTALL(TARGETS i23dSFM_image_graph DESTINATION lib EXPORT i23dSFM-targets)

# UNIT_TEST(i23dSFM image "i23dSFM_image")
# UNIT_TEST(i23dSFM image_drawing "i23dSFM_image")
# UNIT_TEST(i23dSFM image_io "i23dSFM_image")
# UNIT_TEST(i23dSFM image_filtering "i23dSFM_image")
# UNIT_TEST(i23dSFM image_resampling "i23dSFM_image")