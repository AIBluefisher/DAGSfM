# include gflags
set(GFLAGS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/src/third_party/gflags/include)
include_directories(BEFORE ${GFLAGS_INCLUDE_DIR})

set(GPU_INCLUDE .)
set(SYS_INCLUDE )

if(LIBVOT_USE_CUDA)
	# GPU lib is going to be header-file only.
	list(APPEND GPU_INCLUDE cuda_utils.h)
endif(LIBVOT_USE_CUDA)
