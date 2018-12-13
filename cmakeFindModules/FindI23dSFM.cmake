# Locate the i23dSFM libraries.
#
# Defines the following variables:
#
#   I23DSFM_FOUND        - TRUE if the i23dSFM headers and libs are found
#   I23DSFM_INCLUDE_DIRS - The path to i23dSFM headers
#
#   I23DSFM_LIBRARIES    - All i23dSFM libraries
#   I23DSFM_LIBRARY_DIR  - The directory where the libraries are located
#
# Accepts the following variables as input:
#
#   I23DSFM_DIR - (as a CMake or environment variable)
#                The root directory of the i23dSFM install prefix

MESSAGE(STATUS "Looking for I23dSFM.")

FIND_PATH(I23DSFM_INCLUDE_DIR i23dSFM/version.hpp
  HINTS
  $ENV{I23DSFM_DIR}/include
  ${I23DSFM_DIR}/include
  PATH_SUFFIXES
  i23dSFM
)

IF(I23DSFM_INCLUDE_DIR)
  MESSAGE(STATUS "I23dSFM headers found in ${I23DSFM_INCLUDE_DIR}")
ELSE()
  MESSAGE(STATUS "NOT FOUND")
ENDIF (I23DSFM_INCLUDE_DIR)

SET(I23DSFM_LIBRARIES_NAMES  
  i23dSFM_numeric
  i23dSFM_system
  i23dSFM_image
  i23dSFM_kvld
  i23dSFM_lInftyComputerVision
  i23dSFM_multiview
  #third_party libraries
  ceres
  flann_cpp_s
  lemon
  stlplus
  easyexif
  #optional third_party
  vlsift
  jpeg
  png
  tiff
  zlib)

FIND_LIBRARY(I23DSFM_LIBRARY NAMES ${I23DSFM_LIBRARIES_NAMES}
  HINTS
  $ENV{I23DSFM_DIR}/lib
  ${I23DSFM_DIR}/lib
  PATH_SUFFIXES
  i23dSFM
)
GET_FILENAME_COMPONENT(I23DSFM_LIBRARY_DIR "${I23DSFM_LIBRARY}" PATH)

SET(I23DSFM_LIBRARY "")
FOREACH(lib ${I23DSFM_LIBRARIES_NAMES})
 LIST(APPEND I23DSFM_LIBRARY ${lib})  
ENDFOREACH()

SET(I23DSFM_LIBRARIES ${I23DSFM_LIBRARY})
SET(I23DSFM_INCLUDE_DIRS ${I23DSFM_INCLUDE_DIR})

IF(I23DSFM_LIBRARY)
  MESSAGE(STATUS "I23dSFM libraries found: ${I23DSFM_LIBRARY}")
  MESSAGE(STATUS "I23dSFM libraries directories: ${I23DSFM_LIBRARY_DIR}")
ENDIF (I23DSFM_LIBRARY)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set I23DSFM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(I23dSFM  DEFAULT_MSG
                                  I23DSFM_LIBRARY I23DSFM_INCLUDE_DIR)

MARK_AS_ADVANCED(I23DSFM_INCLUDE_DIR I23DSFM_LIBRARY)

#Third parties:
# - include directories

IF(I23DSFM_FOUND)
  SET(I23DSFM_INCLUDE_DIRS
    ${I23DSFM_INCLUDE_DIR}
    ${I23DSFM_INCLUDE_DIR}/i23dSFM_third_party
    ${I23DSFM_INCLUDE_DIR}/i23dSFM_third_party/eigen
    #${I23DSFM_INCLUDE_DIR}/i23dSFM_third_party/lemon
    #${I23DSFM_INCLUDE_DIR}/i23dSFM_third_party/ceres-solver/include
    #${I23DSFM_INCLUDE_DIR}/i23dSFM_third_party/ceres-solver/internal/ceres/miniglog
    #${I23DSFM_INCLUDE_DIR}/i23dSFM_third_party/ceres-solver/config
    #${I23DSFM_INCLUDE_DIR}/i23dSFM_third_party/flann/src/cpp
  )
ENDIF(I23DSFM_FOUND)
