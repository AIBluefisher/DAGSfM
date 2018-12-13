file(GLOB source . "*.cpp" "*.c" "*.h" "*.hpp" "*.inl")

add_executable(i23dSFM_BatchRename  ${source})
target_link_libraries(i23dSFM_BatchRename stlplus)  

SET_PROPERTY(TARGET i23dSFM_BatchRename PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_BatchRename DESTINATION bin/)