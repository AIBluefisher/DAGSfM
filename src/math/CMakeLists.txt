set(FOLDER_NAME "math")

COLMAP_ADD_SOURCES(
    distribution.h
    gauss_jordan.h
    linear_operator.h
    lm.h
    matrix_square_root.h matrix_square_root.cpp
    rotation.h rotation.cpp
    rq_decomposition.h
    sparse_cholesky_llt.h sparse_cholesky_llt.cpp
    sparse_matrix.h sparse_matrix.cpp
    spectra_linear_operator.h
    util.h
)