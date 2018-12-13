# Copyright (c) 2015 - 2016. Tianwei Shen (shentianweipku@gmail.com)
# Hong Kong University of Science and Technology (HKUST)

cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)  # newest gflags needs 2.8.12

project(libvot C CXX)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH is enabled by default.
endif()

# Disable in-source build
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source build is not allowed, please do out-of-source build.")
endif()

# The version number.
set(LIBVOT_VERSION_MAJOR 0)
set(LIBVOT_VERSION_MINOR 1)
set(LIBVOT_VERSION_PATCH 3)
set(LIBVOT_VERSION ${LIBVOT_VERSION_MAJOR}.${LIBVOT_VERSION_MINOR}.${LIBVOT_VERSION_PATCH})

# ================= libvot build options =============================
option(LIBVOT_BUILD_TESTS "Build libvot tests" ON)
option(LIBVOT_BUILD_SHARED "Build libvot shared libs" ON)
# option(LIBVOT_BUILD_DOC "Build libvot documentations" ON)
# option(LIBVOT_BUILD_PYTHON "Build libvot with python export" OFF)
option(LIBVOT_USE_BOOST "Build libvot with Boost" ON)
option(LIBVOT_USE_OPENCV "Build libvot with OpenCV support" ON)
option(LIBVOT_USE_CUDA "Build libvot with cuda support" OFF)
option(LIBVOT_USE_CUDNN "Build libvot with cuda and cuDNN" OFF)

# ==============================================================================
# set compiler options, output all warnings
# ==============================================================================
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(warnings "/W4 /WX /EHsc")
endif()

# Additional cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/scripts/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -pipe -march=native -mtune=native -mfpmath=sse")

# by default build in release mode
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

# ================= Configure libvot dependencies =========================
if(LIBVOT_USE_OPENCV)
    find_package( OpenCV REQUIRED )
    if (OpenCV_FOUND)
        include_directories(${OpenCV_INCLUDE_DIRS})
		add_definitions(-DLIBVOT_USE_OPENCV)
    elseif(OpenCV_FOUND)
        message(STATUS "Could not locate OpenCV, disabling OpenCV support.")
        set(LIBVOT_USE_OPENCV OFF)
		remove_definitions(-DLIBVOT_USE_OPENCV)
    endif (OpenCV_FOUND)
endif()

# configure cuda and cuDNN
set(CUDA_LINKER_LIBS "")
include(scripts/cmake/Cuda.cmake)
# fix issue #14: cannot find -lopencv_dep_cudart
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# Boost building option
if(LIBVOT_USE_BOOST)
    find_package(Boost 1.53 REQUIRED system serialization QUIET)
    if (Boost_FOUND)
        include_directories(${Boost_INCLUDE_DIR})
        add_definitions(-DLIBVOT_USE_BOOST)
  		set(Boost_USE_STATIC_LIBS OFF)
  		set(Boost_USE_MULTITHREADED ON)
  		set(Boost_USE_STATIC_RUNTIME OFF)
    elseif(Boost_FOUND)
        message(STATUS "Could not locate Boost, disabling Boost support.")
        set(LIBVOT_USE_BOOST OFF)
        remove_definitions(-DLIBVOT_USE_BOOST)
    endif(Boost_FOUND)
endif()
# ================= Configure libvot dependencies end =========================

# Specify output bin_path and lib_path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ==============================================================================
# Set includes and libraries begin
# ==============================================================================
# (This has to be specified before compiling the executables)
# include files
# set(LIBVOT_INCLUDES
#     ${CMAKE_SOURCE_DIR}/src
#     ${CMAKE_SOURCE_DIR}/src/third_party
#     ${CMAKE_SOURCE_DIR}/src/vocab_tree
#     ${CMAKE_SOURCE_DIR}/src/utils
#     ${CMAKE_SOURCE_DIR}/src/feature
#     ${CMAKE_SOURCE_DIR}/src/matching
#     )
set(LIBVOT_INCLUDES
    ${CMAKE_SOURCE_DIR}/software/libvot/src
    ${CMAKE_SOURCE_DIR}/software/libvot/src/third_party
    ${CMAKE_SOURCE_DIR}/software/libvot/src/vocab_tree
    ${CMAKE_SOURCE_DIR}/software/libvot/src/utils
    ${CMAKE_SOURCE_DIR}/software/libvot/src/feature
    ${CMAKE_SOURCE_DIR}/software/libvot/src/matching
    )
include_directories(BEFORE ${LIBVOT_INCLUDES})

# Setup vlfeat for feature detection
set(VLFEAT_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/software/libvot/src/third_party/vlfeat)
set(VLFEAT_LIBRARY vlfeat)
include_directories(BEFORE ${VLFEAT_INCLUDE_DIRS})

# Setup gflags and glog (Required)
find_package(gflags REQUIRED)
add_definitions(-DLIBVOT_USE_GFLAGS)
find_package(glog REQUIRED)
add_definitions(-DLIBVOT_USE_GLOG)

# Setup eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# output include directories for debug
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	message(STATUS "INCLUDE_DIRECTORIES:")
	get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
	foreach(dir ${dirs})
		message(STATUS "dir='${dir}'")
	endforeach()
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
# =============================== Set libraries end ==============================

# Specify output bin_path and lib_path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# The source files
add_subdirectory(src)

# Test source code
if(LIBVOT_BUILD_TESTS)
        enable_testing()
        find_package(GTest REQUIRED)
        include_directories(${GTEST_INCLUDE_DIRS})
	add_subdirectory(test)
endif(LIBVOT_BUILD_TESTS)

# python export (python export depends on boost)
# if(LIBVOT_USE_BOOST AND LIBVOT_BUILD_PYTHON)
# 	add_subdirectory(python)
# endif(LIBVOT_USE_BOOST AND LIBVOT_BUILD_PYTHON)


# ================= Output building configurations =========================
message("\n")
message("Building configurations:\n")
message("** libvot version: " ${LIBVOT_VERSION})
message("** Build shared libs: " ${LIBVOT_BUILD_SHARED})
message("** Build tests: " ${LIBVOT_BUILD_TESTS})
message("** Build docs: " ${LIBVOT_BUILD_DOC})
message("** Build type: " ${CMAKE_BUILD_TYPE})
# message("** Build python export: " ${LIBVOT_BUILD_PYTHON})
if(LIBVOT_USE_OPENCV)
    message("** Compile with OpenCV, headers found in ${OpenCV_INCLUDE_DIRS}.")
endif(LIBVOT_USE_OPENCV)
if(LIBVOT_USE_CUDA)
    message( "** Compile with CUDA ${CUDA_VERSION_STRING}, CUDA SDK root at ${CUDA_TOOLKIT_ROOT_DIR}, linker libs ${CUDA_LINKER_LIBS}")
endif(LIBVOT_USE_CUDA)
if(LIBVOT_USE_CUDNN)
    message( "** Compile with CUDNN ${CUDNN_VERSION}, include ${CUDNN_INCLUDE}, library ${CUDNN_LIBRARY}")
endif(LIBVOT_USE_CUDNN)
if(LIBVOT_USE_BOOST)
    message("** Compile with Boost, version " ${Boost_MAJOR_VERSION} "." ${Boost_MINOR_VERSION} ", "
            "headers found in " ${Boost_INCLUDE_DIRS} ".")
endif(LIBVOT_USE_BOOST)
message("\n")
