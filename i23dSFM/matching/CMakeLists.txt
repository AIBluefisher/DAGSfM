
project(matching)

file(
	GLOB
	matching_files_header
	*.hpp
    *.h

)
file(
	GLOB
	matching_files_cpp
	*.cpp
)
file(GLOB REMOVEFILESUNITTEST *_test.cpp)

#Remove the future main files
list(REMOVE_ITEM matching_files_cpp ${REMOVEFILESUNITTEST})

set_source_files_properties(${matching_files_cpp} PROPERTIES LANGUAGE CXX)
add_library(i23dSFM_matching
  ${matching_files_header}
  ${matching_files_cpp})
set_target_properties(i23dSFM_matching PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")
target_link_libraries(i23dSFM_matching ${FLANN_LIBRARY})
set_property(TARGET i23dSFM_matching PROPERTY FOLDER I23dSFM/I23dSFM)
install(TARGETS i23dSFM_matching DESTINATION lib EXPORT i23dSFM-targets)

UNIT_TEST(i23dSFM matching "")
UNIT_TEST(i23dSFM matching_filters "")
UNIT_TEST(i23dSFM indMatch "")
UNIT_TEST(i23dSFM metric "")

add_subdirectory(kvld)
