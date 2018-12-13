
file(GLOB_RECURSE vocab_tree_files_header *.h)
file(GLOB_RECURSE vocab_tree_files_cpp *.cpp)

if(LIBVOT_BUILD_SHARED)
	add_library(vot_vocabtree SHARED ${vocab_tree_files_header} ${vocab_tree_files_cpp})
elseif(LIBVOT_BUILD_SHARED)
	add_library(vot_vocabtree STATIC ${vocab_tree_files_header} ${vocab_tree_files_cpp})
endif(LIBVOT_BUILD_SHARED)
target_link_libraries(vot_vocabtree ${CMAKE_THREAD_LIBS_INIT} vot_utils)

set_target_properties(vot_vocabtree PROPERTIES SOVERSION ${LIBVOT_VERSION_MAJOR}
VERSION "${LIBVOT_VERSION_MAJOR}.${LIBVOT_VERSION_MINOR}")

install(TARGETS vot_vocabtree DESTINATION lib CONFIGURATIONS Release)
