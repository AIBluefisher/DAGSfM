
project(features)

file(
	GLOB_RECURSE
	features_files_headers
	*.hpp
)

file(
	GLOB_RECURSE
	features_files_sources
	*.cpp
)

file(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)
#Remove the future main files
list(REMOVE_ITEM features_files_sources ${REMOVEFILESUNITTEST})
			
set_source_files_properties(${features_files_sources} PROPERTIES LANGUAGE CXX)
ADD_LIBRARY(i23dSFM_features ${features_files_sources} ${features_files_headers})
SET_TARGET_PROPERTIES(i23dSFM_features PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")
INSTALL(TARGETS i23dSFM_features DESTINATION lib EXPORT i23dSFM-targets)
SET_PROPERTY(TARGET i23dSFM_features PROPERTY FOLDER I23dSFM/I23dSFM)

UNIT_TEST(i23dSFM features "i23dSFM_features")

