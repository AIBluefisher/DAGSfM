
###
# Localize images in an existing SfM reconstruction
###
ADD_EXECUTABLE(i23dSFM_main_SfM_Localization main_SfM_Localization.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_SfM_Localization
        i23dSFM_system
        i23dSFM_image
        i23dSFM_features
        i23dSFM_matching_image_collection
        i23dSFM_sfm
        easyexif
        vlsift
        )

# Installation rules
SET_PROPERTY(TARGET i23dSFM_main_SfM_Localization PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_main_SfM_Localization DESTINATION bin/)

