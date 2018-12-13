#ifndef VOT_CUDA_UTILS_H
#define VOT_CUDA_UTILS_H

#include <iostream>
#include <cstdint>
#include <cstddef>
#include <cublas_v2.h>
#include <cudnn.h>

namespace vot {

void FatalError(const int lineNumber=0) {
    std::cerr << "FatalError";
    if (lineNumber!=0) std::cerr<<" at LINE "<<lineNumber;
    std::cerr << ". Program Terminated.\n";
    cudaDeviceReset();
    exit(EXIT_FAILURE);
}

void checkCUDA(const int lineNumber, cudaError_t status) {
    if (status != cudaSuccess) {
        std::cerr << "CUDA failure at LINE " << lineNumber << ": " << status << std::endl;
        FatalError();
    }
}

void checkCUDNN(const int lineNumber, cudnnStatus_t status) {
    if (status != CUDNN_STATUS_SUCCESS) {
        std::cerr << "CUDNN failure at LINE " << lineNumber << ": ";
        switch (status) {
            case CUDNN_STATUS_SUCCESS:
				std::cerr << "CUDNN_STATUS_SUCCESS\n"; break;
            case CUDNN_STATUS_NOT_INITIALIZED:
				std::cerr << "CUDNN_STATUS_NOT_INITIALIZED\n"; break;
            case CUDNN_STATUS_ALLOC_FAILED:
				std::cerr << "CUDNN_STATUS_ALLOC_FAILED\n"; break;
            case CUDNN_STATUS_BAD_PARAM:
				std::cerr << "CUDNN_STATUS_BAD_PARAM\n"; break;
            case CUDNN_STATUS_INTERNAL_ERROR:
				std::cerr << "CUDNN_STATUS_INTERNAL_ERROR\n"; break;
            case CUDNN_STATUS_INVALID_VALUE:
				std::cerr << "CUDNN_STATUS_INVALID_VALUE\n"; break;
            case CUDNN_STATUS_ARCH_MISMATCH:
				std::cerr << "CUDNN_STATUS_ARCH_MISMATCH\n"; break;
            case CUDNN_STATUS_MAPPING_ERROR:
				std::cerr << "CUDNN_STATUS_MAPPING_ERROR\n"; break;
            case CUDNN_STATUS_EXECUTION_FAILED:
				std::cerr << "CUDNN_STATUS_EXECUTION_FAILED\n"; break;
            case CUDNN_STATUS_NOT_SUPPORTED:
				std::cerr << "CUDNN_STATUS_NOT_SUPPORTED\n"; break;
            case CUDNN_STATUS_LICENSE_ERROR:
				std::cerr << "CUDNN_STATUS_LICENSE_ERROR\n"; break;
        }
        FatalError();
    }
    checkCUDA(lineNumber, cudaGetLastError());

}
void checkCUBLAS(const int lineNumber, cublasStatus_t status) {
    if (status != CUBLAS_STATUS_SUCCESS) {
        std::cerr << "CUBLAS failure at LINE " << lineNumber << ": ";
        switch (status) {
            case CUBLAS_STATUS_SUCCESS:
				std::cerr << "CUBLAS_STATUS_SUCCESS\n"; break;
            case CUBLAS_STATUS_NOT_INITIALIZED:
				std::cerr << "CUBLAS_STATUS_NOT_INITIALIZED\n"; break;
            case CUBLAS_STATUS_ALLOC_FAILED:
				std::cerr << "CUBLAS_STATUS_ALLOC_FAILED\n"; break;
            case CUBLAS_STATUS_INVALID_VALUE:
				std::cerr << "CUBLAS_STATUS_INVALID_VALUE\n"; break;
            case CUBLAS_STATUS_ARCH_MISMATCH:
				std::cerr << "CUBLAS_STATUS_ARCH_MISMATCH\n"; break;
            case CUBLAS_STATUS_MAPPING_ERROR:
				std::cerr << "CUBLAS_STATUS_MAPPING_ERROR\n"; break;
            case CUBLAS_STATUS_EXECUTION_FAILED:
				std::cerr << "CUBLAS_STATUS_EXECUTION_FAILED\n"; break;
            case CUBLAS_STATUS_INTERNAL_ERROR:
				std::cerr << "CUBLAS_STATUS_INTERNAL_ERROR\n"; break;
            case CUBLAS_STATUS_NOT_SUPPORTED:
				std::cerr << "CUBLAS_STATUS_NOT_SUPPORTED\n"; break;
            case CUBLAS_STATUS_LICENSE_ERROR:
				std::cerr << "CUBLAS_STATUS_LICENSE_ERROR\n"; break;
        }
        FatalError();
    }
    checkCUDA(lineNumber, cudaGetLastError());
}

}	// end of namespace vot

#endif  // VOT_CUDA_UTILS_H
