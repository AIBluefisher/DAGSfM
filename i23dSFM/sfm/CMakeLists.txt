
file(
	GLOB_RECURSE
	sfm_files_header
	*.hpp
)
file(
	GLOB_RECURSE
	sfm_files_cpp
	*.cpp
)
file(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)
#Remove the future main files
list(REMOVE_ITEM sfm_files_cpp ${REMOVEFILESUNITTEST})
#Remove the test header main files
file(GLOB_RECURSE REMOVEFILESUNITTEST *_test.hpp)
list(REMOVE_ITEM sfm_files_header ${REMOVEFILESUNITTEST})

add_library(i23dSFM_sfm ${sfm_files_header} ${sfm_files_cpp})
target_link_libraries(
  i23dSFM_sfm
  i23dSFM_multiview
  stlplus
  ${CERES_LIBRARIES}
  i23dSFM_lInftyComputerVision
  i23dSFM_system
  i23dSFM_matching
)
set_target_properties(i23dSFM_sfm PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")

install(TARGETS i23dSFM_sfm DESTINATION lib EXPORT i23dSFM-targets)

UNIT_TEST(i23dSFM sfm_data_io
  "i23dSFM_features;i23dSFM_sfm;i23dSFM_system;stlplus")
UNIT_TEST(i23dSFM sfm_data_BA
  "i23dSFM_multiview_test_data;i23dSFM_features;i23dSFM_multiview;i23dSFM_sfm;i23dSFM_system;stlplus")
UNIT_TEST(i23dSFM sfm_data_utils
  "i23dSFM_features;i23dSFM_multiview;i23dSFM_system;i23dSFM_sfm;stlplus")

add_subdirectory(pipelines)

