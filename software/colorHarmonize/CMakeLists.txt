
ADD_EXECUTABLE(i23dSFM_main_ColHarmonize main_ColHarmonize.cpp
  colorHarmonizeEngineGlobal.cpp)
TARGET_LINK_LIBRARIES(i23dSFM_main_ColHarmonize
  i23dSFM_system
  i23dSFM_image
  i23dSFM_kvld
  i23dSFM_features
  i23dSFM_sfm
  stlplus
  ${CLP_LIBRARIES} ${COINUTILS_LIBRARY}
  ${OSI_LIBRARY})

SET_PROPERTY(TARGET i23dSFM_main_ColHarmonize PROPERTY FOLDER I23dSFM/software)

INSTALL(TARGETS i23dSFM_main_ColHarmonize DESTINATION bin/)
