
ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

file(
	GLOB_RECURSE
	image_files_header
	*.hpp
)
file(
	GLOB_RECURSE
	image_files_cpp
	*.cpp
)
file(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)

#Remove the future main files
list(REMOVE_ITEM image_files_cpp ${REMOVEFILESUNITTEST})

ADD_LIBRARY(i23dSFM_image ${image_files_header} ${image_files_cpp})
TARGET_LINK_LIBRARIES(i23dSFM_image i23dSFM_numeric ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${TIFF_LIBRARIES})
SET_TARGET_PROPERTIES(i23dSFM_image PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")
INSTALL(TARGETS i23dSFM_image DESTINATION lib EXPORT i23dSFM-targets)

UNIT_TEST(i23dSFM image "i23dSFM_image")
UNIT_TEST(i23dSFM image_drawing "i23dSFM_image")
UNIT_TEST(i23dSFM image_io "i23dSFM_image")
UNIT_TEST(i23dSFM image_filtering "i23dSFM_image")
UNIT_TEST(i23dSFM image_resampling "i23dSFM_image")

