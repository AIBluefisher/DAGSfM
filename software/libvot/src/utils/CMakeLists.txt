file(GLOB_RECURSE utils_files_header *.h)
file(GLOB_RECURSE utils_files_cpp *.cpp)

if(LIBVOT_BUILD_SHARED)
	add_library(vot_utils SHARED ${utils_files_header} ${utils_files_cpp})
elseif(LIBVOT_BUILD_SHARED)
	add_library(vot_utils STATIC ${utils_files_header} ${utils_files_cpp})
endif(LIBVOT_BUILD_SHARED)

set_target_properties(vot_utils PROPERTIES SOVERSION ${LIBVOT_VERSION_MAJOR}
VERSION "${LIBVOT_VERSION_MAJOR}.${LIBVOT_VERSION_MINOR}")

install (TARGETS vot_utils DESTINATION lib CONFIGURATIONS Release)
