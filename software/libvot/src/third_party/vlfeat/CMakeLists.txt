# Skeleton of VLFeat (0.9.20)
# Tianwei Shen <shentianweipku@gmail.com>

# Change Log:
# * May 24, 2016 
# * Remove executables, exmaples, data, doc, etc.
# * Add CMake import

cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)
project(vlfeat)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH is enabled by default.
endif()

# disable Intel@avx 
add_definitions(-DVL_DISABLE_AVX)

file(GLOB_RECURSE vlfeat_header vl/*.h vl/*.tc)
file(GLOB_RECURSE vlfeat_cpp vl/*.c vl/*.th)

add_library(vlfeat SHARED ${vlfeat_header} ${vlfeat_cpp})

# link pthread into vlfeat, otherwise there might be errors regarding
# pthread_key_create, pthread_getspecific, pthread_key_delete, etc.
# Thanks to http://stackoverflow.com/questions/1620918/cmake-and-libpthread
find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
    set_property(TARGET vlfeat PROPERTY COMPILE_OPTIONS "-pthread")
    set_property(TARGET vlfeat PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(vlfeat "${CMAKE_THREAD_LIBS_INIT}")
endif()

set_property(TARGET vlfeat PROPERTY FOLDER libvot/third_party/vlfeat)
