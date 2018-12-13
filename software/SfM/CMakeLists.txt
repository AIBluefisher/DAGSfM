
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

#convert a v0.6 lists.txt file to the new sfm_data.X format
ADD_EXECUTABLE(i23dSFM_main_ConvertList main_ConvertList.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ConvertList
  i23dSFM_system
  i23dSFM_features
  i23dSFM_sfm
  )


SET_PROPERTY(TARGET i23dSFM_main_ConvertList PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ConvertList DESTINATION bin/)


ADD_EXECUTABLE(i23dSFM_main_Calibration main_Calibration.cpp )
TARGET_LINK_LIBRARIES(i23dSFM_main_Calibration
        i23dSFM_system
        i23dSFM_features
        i23dSFM_sfm
        i23dSFM_image
        )


SET_PROPERTY(TARGET i23dSFM_main_ConvertList PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ConvertList DESTINATION bin/)

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

#INCLUDE_DIRECTORIES(SYSTEM)
#FIND_PACKAGE(MPI)
#INCLUDE_DIRECTORIES(SYSTEM ${MPI_INCLUDE_PATH})
#
#FIND_PACKAGE(OpenCV REQUIRED)
#set(CMAKE_CXX_COMPILER "mpicxx")
#set(CMAKE_C_COMPILER "mpicc")
#set(SERVER_FILE "hosts")
#CONFIGURE_FILE(${SERVER_FILE} "${CMAKE_CURRENT_BINARY_DIR}/hosts" )
#
#add_executable(i23dSFM_main_mipiComputeFeatures ../mpi/main_mipiComputeFeatures.cpp)
#target_link_libraries(
#        i23dSFM_main_mipiComputeFeatures
#        i23dSFM_system
#        i23dSFM_image
#        i23dSFM_features
#        i23dSFM_multiview
#        i23dSFM_sfm
#        stlplus
#        vlsift
#)
#SET_PROPERTY(TARGET i23dSFM_main_mipiComputeFeatures PROPERTY FOLDER I23dSFM/software)
#INSTALL(TARGETS i23dSFM_main_mipiComputeFeatures DESTINATION bin/)

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
  gms
  )



# Installation rules
SET_PROPERTY(TARGET i23dSFM_main_ComputeFeatures PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ComputeFeatures DESTINATION bin/)
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

  ADD_EXECUTABLE(i23dSFM_main_HybridSfM main_HybridSfM.cpp)
  TARGET_LINK_LIBRARIES(i23dSFM_main_HybridSfM
    i23dSFM_system
    i23dSFM_image
    i23dSFM_features
    i23dSFM_sfm
    stlplus
    )

ADD_EXECUTABLE(i23dSFM_main_ConvertSfM_DataFormat main_ConvertSfM_DataFormat.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ConvertSfM_DataFormat
  i23dSFM_system
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

ADD_EXECUTABLE(i23dSFM_main_FrustumFiltering main_FrustumFiltering.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_FrustumFiltering
  i23dSFM_system
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

ADD_EXECUTABLE(i23dSFM_main_GenerateMatchBlocks main_GenerateMatchBlocks.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_GenerateMatchBlocks
                      i23dSFM_features
                      i23dSFM_sfm
                      stlplus)

ADD_EXECUTABLE(i23dSFM_main_MergeBlockMatches main_MergeBlockMatches.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_MergeBlockMatches
                      i23dSFM_features
                      i23dSFM_sfm
                      i23dSFM_system
                      i23dSFM_multiview
                      i23dSFM_matching_image_collection
                      stlplus)

# Installation rules
SET_PROPERTY(TARGET i23dSFM_main_IncrementalSfM PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_IncrementalSfM DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_GlobalSfM PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_GlobalSfM DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_ConvertSfM_DataFormat PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ConvertSfM_DataFormat DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_FrustumFiltering PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_FrustumFiltering DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_ComputeStructureFromKnownPoses PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ComputeStructureFromKnownPoses DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_ComputeSfM_DataColor PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ComputeSfM_DataColor DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_GenerateMatchBlocks PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_GenerateMatchBlocks DESTINATION bin/)

###
# SfM tools to visualize feature tracking data
###

# - View extracted Keypoints
#
ADD_EXECUTABLE(i23dSFM_main_exportKeypoints main_exportKeypoints.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_exportKeypoints
  i23dSFM_system
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )


# - View computed matches (putatives, geometrics) per image pair
#
ADD_EXECUTABLE(i23dSFM_main_exportMatches main_exportMatches.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_exportMatches
  i23dSFM_system
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

# - View tracks per image pair
#
ADD_EXECUTABLE(i23dSFM_main_exportTracks main_exportTracks.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_exportTracks
  i23dSFM_system
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

# - Export undistorted images related to a sfm_data file
#
ADD_EXECUTABLE(i23dSFM_main_ExportUndistortedImages main_ExportUndistortedImages.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ExportUndistortedImages
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

# installation rules
SET_PROPERTY(TARGET i23dSFM_main_exportKeypoints PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_exportKeypoints DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_exportMatches PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_exportMatches DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_exportTracks PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_exportTracks DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_ExportUndistortedImages PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ExportUndistortedImages DESTINATION bin/)

###
# SfM export to X
# - PMVS + bundler data format
# - CMPMVS
# - Meshlab
# - MVE (File format v2)
###

# - Export a SfM i23dSFM scene to PMVS format
#
ADD_EXECUTABLE(i23dSFM_main_i23dSFM2PMVS main_i23dSFM2PMVS.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_i23dSFM2PMVS
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

# - Export a SfM i23dSFM scene to CMPMVS format
#
ADD_EXECUTABLE(i23dSFM_main_i23dSFM2CMPMVS main_i23dSFM2CMPMVS.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_i23dSFM2CMPMVS
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )


# - Export a SfM i23dSFM scene to MVE(v2) format
#
ADD_EXECUTABLE(i23dSFM_main_i23dSFM2MVE2 main_i23dSFM2MVE2.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_i23dSFM2MVE2 
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

# - Export a SfM i23dSFM scene to meshlab scene with rasters
# -
ADD_EXECUTABLE(i23dSFM_main_i23dSFM2MESHLAB main_i23dSFM2MESHLAB.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_i23dSFM2MESHLAB
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

# - Export a SfM i23dSFM scene to mvs-texturing scene folder
# -
ADD_EXECUTABLE(i23dSFM_main_i23dSFM2MVSTEXTURING main_i23dSFM2MVSTEXTURING.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_i23dSFM2MVSTEXTURING
  i23dSFM_system
  i23dSFM_image
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

# - Export SfM i23dSFM camera scene as triangle meshes
# -
ADD_EXECUTABLE(i23dSFM_main_ExportCameraFrustums main_ExportCameraFrustums.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ExportCameraFrustums
  i23dSFM_system
  i23dSFM_features
  i23dSFM_sfm
  easyexif
  stlplus
  )




# installation rules
SET_PROPERTY(TARGET i23dSFM_main_i23dSFM2PMVS PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_i23dSFM2PMVS DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_i23dSFM2CMPMVS PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_i23dSFM2CMPMVS DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_i23dSFM2MESHLAB PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_i23dSFM2MESHLAB DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_i23dSFM2MVSTEXTURING PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_i23dSFM2MVSTEXTURING DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_i23dSFM2MVE2 PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_i23dSFM2MVE2 DESTINATION bin/)
SET_PROPERTY(TARGET i23dSFM_main_ExportCameraFrustums PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_ExportCameraFrustums DESTINATION bin/)

# -
# Use openCV Feature detector to describe the images
# -
IF(I23dSFM_USE_OPENCV)
  ADD_EXECUTABLE(i23dSFM_main_ComputeFeatures_OpenCV
      main_ComputeFeatures_OpenCV.cpp)
  TARGET_LINK_LIBRARIES(i23dSFM_main_ComputeFeatures_OpenCV
      i23dSFM_system
      i23dSFM_image
      i23dSFM_features
      i23dSFM_multiview
      i23dSFM_sfm
      stlplus
      vlsift
      ${OpenCV_LIBS})

  SET_PROPERTY(TARGET i23dSFM_main_ComputeFeatures_OpenCV PROPERTY FOLDER I23dSFM/software)
  SET_PROPERTY(TARGET i23dSFM_main_ComputeFeatures_OpenCV PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
  INSTALL(TARGETS i23dSFM_main_ComputeFeatures_OpenCV DESTINATION bin/)
ENDIF(I23dSFM_USE_OPENCV)


###
# SfM Research tools
#
# - Quality comparison against a GT camera path (MultiView Evaluation dataset)
###
ADD_EXECUTABLE(i23dSFM_main_evalQuality main_evalQuality.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_evalQuality
  i23dSFM_system
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  )

#installation rules
SET_PROPERTY(TARGET i23dSFM_main_evalQuality PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_evalQuality DESTINATION bin/)

##
# Export tutorial with valid path thanks to a configuration file
##
SET(I23DSFM_SOFTWARE_SFM_SRC_DIR ${CMAKE_CURRENT_LIST_DIR})
IF(MSVC OR APPLE)
  SET(I23DSFM_SOFTWARE_SFM_BUILD_DIR "${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
ELSE()
  SET(I23DSFM_SOFTWARE_SFM_BUILD_DIR ${EXECUTABLE_OUTPUT_PATH})
ENDIF()

# SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/tutorial_demo.py.in")
# CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
#     "${CMAKE_CURRENT_BINARY_DIR}/tutorial_demo.py" @ONLY)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_GlobalPipeline.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_GlobalPipeline.py" @ONLY)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_SequentialPipeline.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_SequentialPipeline.py" @ONLY)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_HybridPipeline.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_HybridPipeline.py" @ONLY)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_DC_Pipeline_Dismatch.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_DC_Pipeline_Dismatch.py" @ONLY)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_DC_Pipeline_singlematch.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_DC_Pipeline_singlematch.py" @ONLY)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_EnftPipeline.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
   "${CMAKE_CURRENT_BINARY_DIR}/SfM_EnftPipeline.py" @ONLY)




