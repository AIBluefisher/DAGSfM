file(GLOB source . "*.cpp" "*.c" "*.h" "*.hpp" "*.inl")

include_directories(${PROJECT_SOURCE_DIR}/third_party/graclus/metisLib)
add_executable(i23dSFM_GraphCluster  ${source})
target_link_libraries(i23dSFM_GraphCluster 
  graclus 
  i23dSFM_image_graph
  i23dSFM_image
  i23dSFM_system
  i23dSFM_sfm
  i23dSFM_features
  stlplus)  
  

SET_PROPERTY(TARGET i23dSFM_GraphCluster PROPERTY FOLDER I23dSFM/software)
INSTALL(TARGETS i23dSFM_GraphCluster DESTINATION bin/)