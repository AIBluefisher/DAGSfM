list(APPEND VOT_MATCHING_HEADER )
list(APPEND VOT_MATCHING_SOURCE )

if(LIBVOT_USE_OPENCV)
	list(APPEND VOT_MATCHING_HEADER opencv_matching_api.h matching_datatypes.h)
	list(APPEND VOT_MATCHING_SOURCE opencv_matching_api.cpp matching_datatypes.cpp)
endif(LIBVOT_USE_OPENCV)

# append libvot matching files
list(APPEND VOT_MATCHING_HEADER matching_datatypes.h feature_matching.h)
list(APPEND VOT_MATCHING_SOURCE matching_datatypes.cpp feature_matching.cpp)

if(LIBVOT_BUILD_SHARED)
    add_library(vot_matching SHARED ${VOT_MATCHING_HEADER} ${VOT_MATCHING_SOURCE})
else(LIBVOT_BUILD_SHARED)
    add_library(vot_matching STATIC ${VOT_MATCHING_HEADER} ${VOT_MATCHING_SOURCE})
endif(LIBVOT_BUILD_SHARED)

if(LIBVOT_USE_OPENCV)
	target_link_libraries(vot_matching ${OpenCV_LIBS})
endif(LIBVOT_USE_OPENCV)

target_link_libraries(vot_matching vot_utils vot_feature)
set_target_properties(vot_matching PROPERTIES SOVERSION ${LIBVOT_VERSION_MAJOR}
VERSION "${LIBVOT_VERSION_MAJOR}.${LIBVOT_VERSION_MINOR}")

install(TARGETS vot_matching DESTINATION lib CONFIGURATIONS Release)
