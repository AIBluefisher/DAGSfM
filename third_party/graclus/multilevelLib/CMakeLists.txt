file(GLOB source . "wkkm.*" "mlkkm.*")

include_directories(${PROJECT_SOURCE_DIR}/metisLib)
add_library(multilevel SHARED ${source})
SET_PROPERTY(TARGET stlplus PROPERTY FOLDER I23dSFM/3rdParty)

INSTALL(
        TARGETS multilevel
        DESTINATION lib
        EXPORT i23dSFM-targets
)
