FILE(
	GLOB
	linearProgramming_headers
	*.hpp
)
FILE(
	GLOB
	linearProgramming_cpp
	*.cpp
)
FILE(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)

SET_SOURCE_FILES_PROPERTIES(${linearProgramming_headers} PROPERTIES LANGUAGE CXX)

#Remove the future main files
LIST(REMOVE_ITEM linearProgramming_cpp ${REMOVEFILESUNITTEST})

IF (NOT MOSEK_FOUND)

  FILE(GLOB_RECURSE REMOVEFILESMOSEK *MOSEK*)
  LIST(REMOVE_ITEM linearProgramming_cpp ${REMOVEFILESMOSEK})
  LIST(REMOVE_ITEM linearProgramming_headers ${REMOVEFILESMOSEK})
  
  SET(i23dSFM_linearProgramming
    ${CLP_LIBRARIES}     # clp + solver wrapper
    ${COINUTILS_LIBRARY} # container tools
    ${OSI_LIBRARY}       # generic LP
  )
  
ELSE (NOT MOSEK_FOUND)
  # MOSEK_FOUND is true
  ADD_LIBRARY(i23dSFM_linearProgrammingMSK ${linearProgramming_headers} ${linearProgramming_cpp})
  SET_PROPERTY(TARGET i23dSFM_linearProgrammingMSK PROPERTY FOLDER I23dSFM/I23dSFM)
  INSTALL(TARGETS i23dSFM_linearProgrammingMSK DESTINATION lib EXPORT i23dSFM-targets)

  SET(i23dSFM_linearProgramming
    lib_Osi_Msk          # OSI solver wrapper for the Mosek backend
    ${CLP_LIBRARIES}     # clp + solver wrapper
    ${COINUTILS_LIBRARY} # container tools
    ${OSI_LIBRARY}       # generic LP

    i23dSFM_linearProgrammingMSK # i23dSFM Mosek wrapper
    ${MOSEK_LIB}    # Mosek lib
  )
ENDIF (NOT MOSEK_FOUND)
UNIT_TEST(i23dSFM_linearProgramming linearProgramming "${i23dSFM_linearProgramming}")

ADD_SUBDIRECTORY(lInfinityCV)
