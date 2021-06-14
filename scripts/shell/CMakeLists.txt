configure_file("${CMAKE_CURRENT_SOURCE_DIR}/colmap_sfm.sh.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/colmap_sfm.sh")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/distributed_sfm.sh.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/distributed_sfm.sh")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/exhaustive_matcher.sh.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/exhaustive_matcher.sh")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/extract_feature.sh.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/extract_feature.sh")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/vocabulary_tree_matcher.sh.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/vocabulary_tree_matcher.sh")