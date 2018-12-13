
file(
  GLOB_RECURSE
  multiview_files_header
  *.hpp
)
file(
  GLOB_RECURSE
  multiview_files_cpp
  *.cpp
)
file(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)
file(GLOB_RECURSE REMOVEFILESUNITTEST_HEADER *_test.hpp)
file(GLOB_RECURSE MULTIVIEWTESTDATA *_data_sets*)

#Remove the unit test files (not been used by the library)
list(REMOVE_ITEM multiview_files_cpp ${REMOVEFILESUNITTEST})
list(REMOVE_ITEM multiview_files_header ${REMOVEFILESUNITTEST_HEADER})
list(REMOVE_ITEM multiview_files_cpp ${MULTIVIEWTESTDATA})

ADD_LIBRARY(i23dSFM_multiview
  ${multiview_files_header}
  ${multiview_files_cpp})
SET_PROPERTY(TARGET i23dSFM_multiview PROPERTY FOLDER I23dSFM)
INSTALL(TARGETS i23dSFM_multiview DESTINATION lib EXPORT i23dSFM-targets)
TARGET_LINK_LIBRARIES(i23dSFM_multiview ${LEMON_LIBRARY})

#Make multiview library integrate numeric library
ADD_LIBRARY(i23dSFM_multiview_test_data ${MULTIVIEWTESTDATA})
SET_PROPERTY(TARGET i23dSFM_multiview_test_data PROPERTY FOLDER I23dSFM/I23dSFM)
TARGET_LINK_LIBRARIES(i23dSFM_multiview i23dSFM_numeric ${CERES_LIBRARIES})
SET_TARGET_PROPERTIES(i23dSFM_multiview PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")


# Triangulation routines
UNIT_TEST(i23dSFM triangulation "i23dSFM_multiview;i23dSFM_multiview_test_data")
UNIT_TEST(i23dSFM triangulation_nview "i23dSFM_multiview;i23dSFM_multiview_test_data")

# Solvers
UNIT_TEST(i23dSFM solver_affine "i23dSFM_multiview")
UNIT_TEST(i23dSFM solver_fundamental_kernel "i23dSFM_multiview")
UNIT_TEST(i23dSFM solver_essential_kernel "i23dSFM_multiview;i23dSFM_multiview_test_data")
UNIT_TEST(i23dSFM solver_homography_kernel "i23dSFM_multiview;i23dSFM_multiview_test_data")
UNIT_TEST(i23dSFM solver_essential_five_point "i23dSFM_multiview;i23dSFM_multiview_test_data")
UNIT_TEST(i23dSFM solver_resection_kernel "i23dSFM_multiview;i23dSFM_multiview_test_data")
UNIT_TEST(i23dSFM solver_translation_knownRotation_kernel "i23dSFM_multiview;i23dSFM_multiview_test_data")

# Averaging routines
UNIT_TEST(i23dSFM rotation_averaging "i23dSFM_multiview;i23dSFM_multiview_test_data;${CERES_LIBRARIES}")
UNIT_TEST(i23dSFM translation_averaging "i23dSFM_multiview;i23dSFM_multiview_test_data;${CERES_LIBRARIES}")

