CMAKE_POLICY(SET CMP0011 NEW)
file(GLOB source . "*.cpp" "*.c" "*.h" "*.hpp" "*.inl")

add_executable(i23dSFM_SfMAligner  ${source})
target_link_libraries(i23dSFM_SfMAligner
  i23dSFM_system
  i23dSFM_features
  i23dSFM_image
  i23dSFM_sfm
  stlplus)  

SET_PROPERTY(TARGET i23dSFM_SfMAligner PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_SfMAligner DESTINATION bin/)

