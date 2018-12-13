
FILE(
	GLOB
	lInftycomputervision_headers
	*.hpp
)

FILE(
	GLOB
	lInftycomputervision_cpp
	*.cpp
)

FILE(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)

# Remove the future main files
LIST(REMOVE_ITEM lInftycomputervision_cpp ${REMOVEFILESUNITTEST})

# Disable some warnings on MSVC.
IF (MSVC)
  # Disable Warning	C4267: conversion from 'size_t' to 'int', possible loss of data
  ADD_DEFINITIONS("/wd4267")
ENDIF (MSVC)

ADD_LIBRARY(i23dSFM_lInftyComputerVision  ${lInftycomputervision_headers} ${lInftycomputervision_cpp})
SET(LINEAR_PROGRAMMING_LIBRAIRIES 
  ${CLP_LIBRARIES}     # clp + solver wrapper
  ${COINUTILS_LIBRARY} # container tools
  ${OSI_LIBRARY}       # generic LP
)
IF (MOSEK_FOUND)
  LIST(APPEND LINEAR_PROGRAMMING_LIBRAIRIES
    i23dSFM_linearProgrammingMSK # i23dSFM Mosek wrapper
    ${MOSEK_LIB}    # Mosek lib
  )
ENDIF (MOSEK_FOUND)
TARGET_LINK_LIBRARIES(i23dSFM_lInftyComputerVision ${LINEAR_PROGRAMMING_LIBRAIRIES})
SET_TARGET_PROPERTIES(i23dSFM_lInftyComputerVision PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")
INSTALL(TARGETS i23dSFM_lInftyComputerVision DESTINATION lib EXPORT i23dSFM-targets)

SET(LOCAL_LIBS
  i23dSFM_multiview_test_data
  i23dSFM_multiview
  i23dSFM_lInftyComputerVision)

# Linfinity Triangulation
UNIT_TEST(i23dSFM_lInftyCV triangulation "${LOCAL_LIBS}")

# Linfinity Pose/Resection estimation
UNIT_TEST(i23dSFM_lInftyCV resection "${LOCAL_LIBS}")
UNIT_TEST(i23dSFM_lInftyCV resection_robust "${LOCAL_LIBS}")

# Linfinity Global translations from triplets of heading directions
UNIT_TEST(i23dSFM_lInftyCV global_translations_fromTriplets "${LOCAL_LIBS}")
# Linfinity Global translations from heading directions
UNIT_TEST(i23dSFM_lInftyCV global_translations_fromTij "${LOCAL_LIBS}")

# Linfinity Translation and Structure 
UNIT_TEST(i23dSFM_lInftyCV tijsAndXis "${LOCAL_LIBS}")
# Linfinity Translation and Structure with noise handling
UNIT_TEST(i23dSFM_lInftyCV tijsAndXis_outlier "${LOCAL_LIBS}")

