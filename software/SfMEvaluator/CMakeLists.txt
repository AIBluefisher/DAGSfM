CMAKE_POLICY(SET CMP0011 NEW)
file(GLOB source . "*.cpp" "*.c" "*.h" "*.hpp" "*.inl")

add_executable(i23dSFM_SfMEvaluator  ${source})
target_link_libraries(i23dSFM_SfMEvaluator
  i23dSFM_system
  i23dSFM_sfm
  stlplus)  

SET_PROPERTY(TARGET i23dSFM_SfMEvaluator PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_SfMEvaluator DESTINATION bin/)

SET(I23DSFM_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/upload_t2_results.py.in")
CONFIGURE_FILE(${I23DSFM_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/upload_t2_results.py" @ONLY)

