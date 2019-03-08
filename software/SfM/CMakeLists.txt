
###
# Intrinsic image analysis and SfM_Data container initialization
###
ADD_EXECUTABLE(i23dSFM_main_SfMInit_ImageListing main_SfMInit_ImageListing.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_SfMInit_ImageListing
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  easyexif
  stlplus
  )
# Installation rules
SET_PROPERTY(TARGET i23dSFM_main_SfMInit_ImageListing PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_SfMInit_ImageListing DESTINATION bin/)

###
# Add executable that computes:
# - i23dSFM_main_ComputeFeatures: features and descriptors
# - i23dSFM_main_ComputeMatches: putative matches + geometric filtered matches
###


ADD_EXECUTABLE(i23dSFM_main_ComputeFeatures main_ComputeFeatures.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ComputeFeatures
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_multiview
  i23dSFM_sfm
  stlplus
  vlsift  
  )
SET_PROPERTY(TARGET i23dSFM_main_ComputeFeatures PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ComputeFeatures DESTINATION bin/)
#set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#
#set(CMAKE_C_COMPILER "/usr/bin/gcc")

ADD_EXECUTABLE(i23dSFM_main_ComputeMatches main_ComputeMatches.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ComputeMatches
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_multiview
  i23dSFM_sfm
  i23dSFM_matching_image_collection
  stlplus
  )
SET_PROPERTY(TARGET i23dSFM_main_ComputeMatches PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ComputeMatches DESTINATION bin/)

###
# SfM Pipelines
# - Incremental/Sequential
# - Global
###
# - Convert SfM_Data format (from one to another)
# - Pair filtering (from frustum)
# - Compute structure from known camera poses
# - Compute structure color
###
ADD_EXECUTABLE(i23dSFM_main_IncrementalSfM main_IncrementalSfM.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_IncrementalSfM
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

ADD_EXECUTABLE(i23dSFM_main_GlobalSfM main_GlobalSfM.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_GlobalSfM
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

ADD_EXECUTABLE(i23dSFM_main_ComputeStructureFromKnownPoses main_ComputeStructureFromKnownPoses.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ComputeStructureFromKnownPoses
  i23dSFM_system
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

ADD_EXECUTABLE(i23dSFM_main_ComputeSfM_DataColor main_ComputeSfM_DataColor.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ComputeSfM_DataColor
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

ADD_EXECUTABLE(i23dSFM_main_ComputeSfM_PhotoColor main_ComputeSfM_PhotoColor.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ComputeSfM_PhotoColor
        i23dSFM_system
        i23dSFM_image
        i23dSFM_features
        i23dSFM_sfm
        stlplus
        )

# Installation rules
SET_PROPERTY(TARGET i23dSFM_main_IncrementalSfM PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_IncrementalSfM DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_GlobalSfM PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_GlobalSfM DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_ComputeStructureFromKnownPoses PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ComputeStructureFromKnownPoses DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_ComputeSfM_DataColor PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ComputeSfM_DataColor DESTINATION bin/)


##
# Export tutorial with valid path thanks to a configuration file
##
SET(I23DSFM_SOFTWARE_SFM_SRC_DIR ${CMAKE_CURRENT_LIST_DIR})
IF(MSVC OR APPLE)
  SET(I23DSFM_SOFTWARE_SFM_BUILD_DIR "${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
ELSE()
  SET(I23DSFM_SOFTWARE_SFM_BUILD_DIR ${EXECUTABLE_OUTPUT_PATH})
ENDIF()

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_GlobalPipeline.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_GlobalPipeline.py" @ONLY)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_SequentialPipeline.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_SequentialPipeline.py" @ONLY)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_DC_Pipeline_singlematch.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_DC_Pipeline_singlematch.py" @ONLY)




