project( libgms C CXX)

# OpenCV 
find_package( OpenCV REQUIRED core )

# 
include_directories(
${OpenCV_INCLUDE_DIRS} 
)


add_library( gms gms_matcher.h gms_matcher.cpp)
target_link_libraries( gms ${OpenCV_LIBS} )
SET_PROPERTY(TARGET stlplus PROPERTY FOLDER I23dSFM/3rdParty)

INSTALL(
        TARGETS gms
        DESTINATION lib
        EXPORT i23dSFM-targets
)
