set(FOLDER_NAME "rotation_estimation")

COLMAP_ADD_SOURCES(
    align_rotations.h align_rotations.cpp
    lagrange_dual_rotation_estimator.h lagrange_dual_rotation_estimator.cpp
    nonlinear_rotation_estimator.h nonlinear_rotation_estimator.cpp
    pairwise_rotation_error.h pairwise_rotation_error.cpp
    robust_rotation_estimator.h robust_rotation_estimator.cpp
    rotation_estimator.h
)

COLMAP_ADD_TEST(align_rotations_test align_rotations_test.cpp)
COLMAP_ADD_TEST(robust_rotation_estimator_test robust_rotation_estimator_test.cpp)
COLMAP_ADD_TEST(lagrange_dual_rotation_estimator_test lagrange_dual_rotation_estimator_test.cpp)