
file(
  GLOB_RECURSE
  matching_collection_images_files_header
  *.hpp
  *.h
)
file(
  GLOB_RECURSE
  matching_collection_images_files_cpp
  *.cpp
)
file(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)

#Remove the unit test files (not been used by the library)
list(REMOVE_ITEM matching_collection_images_files_cpp ${REMOVEFILESUNITTEST})

ADD_LIBRARY(i23dSFM_matching_image_collection
  ${matching_collection_images_files_header}
  ${matching_collection_images_files_cpp})
TARGET_LINK_LIBRARIES(i23dSFM_matching_image_collection i23dSFM_matching)
SET_TARGET_PROPERTIES(i23dSFM_matching_image_collection PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")
SET_PROPERTY(TARGET i23dSFM_matching_image_collection PROPERTY FOLDER I23dSFM)
INSTALL(TARGETS i23dSFM_matching_image_collection DESTINATION lib EXPORT i23dSFM-targets)

UNIT_TEST(i23dSFM Pair_Builder "")
