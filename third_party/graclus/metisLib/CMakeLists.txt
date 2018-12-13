file(GLOB source . "*.cpp" "*.c" "*.h" "*.hpp" "*.inl")
include_directories(.;)
set(CMAKE_CXX_STANDARD 11)
add_library(metis SHARED ${source})
target_link_libraries(metis m)
SET_PROPERTY(TARGET stlplus PROPERTY FOLDER I23dSFM/3rdParty)

INSTALL(
        TARGETS metis
        DESTINATION lib
        EXPORT i23dSFM-targets
)
