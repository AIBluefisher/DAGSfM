list(APPEND VOT_FEATURE_HEADER vlfeat_libvot_api.h)
list(APPEND VOT_FEATURE_SOURCE vlfeat_libvot_api.cpp)

if(LIBVOT_USE_OPENCV)
	list(APPEND VOT_FEATURE_HEADER opencv_libvot_api.h)
	list(APPEND VOT_FEATURE_SOURCE opencv_libvot_api.cpp)
endif(LIBVOT_USE_OPENCV)

if(LIBVOT_BUILD_SHARED)
    add_library(vot_feature SHARED ${VOT_FEATURE_HEADER} ${VOT_FEATURE_SOURCE})
else(LIBVOT_BUILD_SHARED)
    add_library(vot_feature STATIC ${VOT_FEATURE_HEADER} ${VOT_FEATURE_SOURCE})
endif(LIBVOT_BUILD_SHARED)
target_link_libraries(vot_feature ${VLFEAT_LIBRARY})

if(LIBVOT_USE_OPENCV)
	target_link_libraries(vot_feature ${OpenCV_LIBS})
endif(LIBVOT_USE_OPENCV)

set_target_properties(vot_feature PROPERTIES SOVERSION ${LIBVOT_VERSION_MAJOR}
VERSION "${LIBVOT_VERSION_MAJOR}.${LIBVOT_VERSION_MINOR}")

install(TARGETS vot_feature DESTINATION lib CONFIGURATIONS Release)
