project(match_graph)

file(GLOB match_graph_header *.hpp *.h)
file(GLOB match_graph_src *.cpp *.c)

set_source_files_properties(${match_graph_src} PROPERTIES LANGUAGE CXX)
add_library(i23dSFM_match_graph ${match_graph_header} ${match_graph_src})
set_target_properties(i23dSFM_match_graph PROPERTIES SOVERSION ${I23DSFM_VERSION_MAJOR} VERSION "${I23DSFM_VERSION_MAJOR}.${I23DSFM_VERSION_MINOR}")
set_property(TARGET i23dSFM_match_graph PROPERTY FOLDER I23dSFM/I23dSFM)
install(TARGETS i23dSFM_match_graph DESTINATION lib EXPORT i23dSFM-targets)