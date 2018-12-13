cmake_minimum_required(VERSION 2.8)

project(libgraclus  C CXX)
set(CMAKE_CXX_STANDARD 11)
add_subdirectory(metisLib)
add_subdirectory(multilevelLib)
add_subdirectory(graclus)

#include_directories(${PROJECT_SOURCE_DIR}/metisLib;${PROJECT_SOURCE_DIR}/multilevelLib)

#include_directories(${PROJECT_SOURCE_DIR}/graclus)

#link_directories(${PROJECT_BINARY_DIR}/metisLib;${PROJECT_BINARY_DIR}/multilevelLib;${PROJECT_BINARY_DIR}/graclus)

#add_executable(program  main.cpp)
#target_link_libraries(program graclus)
