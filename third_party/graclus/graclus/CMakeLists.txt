file(GLOB_RECURSE source . "*.cpp" "*.c" "*.h" "*.hpp" "*.inl")
#link_directories(${PROJECT_BINARY_DIR}/metisLib;${PROJECT_BINARY_DIR}/multilevelLib)
include_directories(${PROJECT_SOURCE_DIR}/metisLib;${PROJECT_SOURCE_DIR}/multilevelLib)
# add_executable(program  ${source})
# target_link_libraries(program metis multilevel)

add_library(graclus SHARED ${source})
target_link_libraries(graclus metis multilevel)
SET_PROPERTY(TARGET stlplus PROPERTY FOLDER I23dSFM/3rdParty)

INSTALL(
        TARGETS graclus
        DESTINATION lib
        EXPORT i23dSFM-targets
)
