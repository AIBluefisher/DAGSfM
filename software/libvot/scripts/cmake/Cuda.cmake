# configure cuda and cudnn

################################################################################################
# Short command for cuDNN detection, adapted from Caffe
# Usage:
#   detect_cuDNN()
function(detect_cuDNN)
    set(CUDNN_ROOT "" CACHE PATH "CUDNN root folder")

    find_path(CUDNN_INCLUDE cudnn.h
        PATHS ${CUDNN_ROOT} $ENV{CUDNN_ROOT} ${CUDA_TOOLKIT_INCLUDE}
        DOC "Path to cuDNN include directory." )

	# dynamic libs have different suffix in mac and linux
	if(APPLE)
		set(CUDNN_LIB_NAME "libcudnn.dylib")
	else()
		set(CUDNN_LIB_NAME "libcudnn.so")
	endif()

    get_filename_component(__libpath_hist ${CUDA_CUDART_LIBRARY} PATH)
    find_library(CUDNN_LIBRARY NAMES ${CUDNN_LIB_NAME}
       PATHS ${CUDNN_ROOT} $ENV{CUDNN_ROOT} ${CUDNN_INCLUDE} ${__libpath_hist} ${__libpath_hist}/../lib
       DOC "Path to cuDNN library.")

    if(CUDNN_INCLUDE AND CUDNN_LIBRARY)
        set(HAVE_CUDNN  TRUE PARENT_SCOPE)
        set(CUDNN_FOUND TRUE PARENT_SCOPE)

        file(READ ${CUDNN_INCLUDE}/cudnn.h CUDNN_VERSION_FILE_CONTENTS)

        # cuDNN v3 and beyond
        string(REGEX MATCH "define CUDNN_MAJOR * +([0-9]+)"
         CUDNN_VERSION_MAJOR "${CUDNN_VERSION_FILE_CONTENTS}")
        string(REGEX REPLACE "define CUDNN_MAJOR * +([0-9]+)" "\\1"
         CUDNN_VERSION_MAJOR "${CUDNN_VERSION_MAJOR}")
        string(REGEX MATCH "define CUDNN_MINOR * +([0-9]+)"
         CUDNN_VERSION_MINOR "${CUDNN_VERSION_FILE_CONTENTS}")
        string(REGEX REPLACE "define CUDNN_MINOR * +([0-9]+)" "\\1"
         CUDNN_VERSION_MINOR "${CUDNN_VERSION_MINOR}")
        string(REGEX MATCH "define CUDNN_PATCHLEVEL * +([0-9]+)"
         CUDNN_VERSION_PATCH "${CUDNN_VERSION_FILE_CONTENTS}")
        string(REGEX REPLACE "define CUDNN_PATCHLEVEL * +([0-9]+)" "\\1"
           CUDNN_VERSION_PATCH "${CUDNN_VERSION_PATCH}")

        if(NOT CUDNN_VERSION_MAJOR)
            set(CUDNN_VERSION "???")
        else()
            set(CUDNN_VERSION "${CUDNN_VERSION_MAJOR}.${CUDNN_VERSION_MINOR}.${CUDNN_VERSION_PATCH}")
        endif()

        string(COMPARE LESS "${CUDNN_VERSION_MAJOR}" 3 cuDNNVersionIncompatible)
        if(cuDNNVersionIncompatible)
            message(FATAL_ERROR "cuDNN version >3 is required.")
        endif()

        set(CUDNN_VERSION "${CUDNN_VERSION}" PARENT_SCOPE)
        mark_as_advanced(CUDNN_INCLUDE CUDNN_LIBRARY CUDNN_ROOT)
    endif()
endfunction()

if(LIBVOT_USE_CUDA)
    find_package(CUDA QUIET REQUIRED)
    if(CUDA_FOUND)
        include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
        add_definitions(-DLIBVOT_USE_CUDA)
        list(APPEND CUDA_LINKER_LIBS ${CUDA_CUDART_LIBRARY} ${CUDA_curand_LIBRARY} ${CUDA_CUBLAS_LIBRARIES})
    elseif(CUDA_FOUND)
        message(STATUS "Could not locate cuda, disabling cuda support.")
        set(LIBVOT_USE_CUDA OFF)
        set(LIBVOT_USE_CUDNN OFF)
        remove_definitions(-DLIBVOT_USE_CUDA)
        return()
    endif (CUDA_FOUND)
endif(LIBVOT_USE_CUDA)

# configure cuDNN
if(LIBVOT_USE_CUDNN)
    detect_cuDNN()
    if(HAVE_CUDNN)
        add_definitions(-DLIBVOT_USE_CUDNN)
        include_directories(SYSTEM ${CUDNN_INCLUDE})
        list(APPEND CUDA_LINKER_LIBS ${CUDNN_LIBRARY})
	else(HAVE_CUDNN)
        message(STATUS "Could not locate cuDNN, disabling cuDNN support.")
		remove_definitions(-DLIBVOT_USE_CUDNN)
        set(LIBVOT_USE_CUDNN OFF)
    endif()
endif(LIBVOT_USE_CUDNN)

# set cuda flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND CUDA_NVCC_FLAGS "-arch=sm_20;-std=c++11;-DVERBOSE;-g;-lineinfo;-Xcompiler;-ggdb")
else()
    list(APPEND CUDA_NVCC_FLAGS "-arch=sm_20;-std=c++11;-DVERBOSE;-O3;-DNDEBUG;-Xcompiler;-DNDEBU")
endif()
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
