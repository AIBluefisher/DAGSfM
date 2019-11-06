set(FOLDER_NAME ransac)

COLMAP_ADD_SOURCES(
    estimator.h
    exhaustive_ransac.h
    exhaustive_sampler.h exhaustive_sampler.cpp
    inlier_support.h
    lmed_quality_measurement.h
    lmed.h
    mle_quality_measurement.h
    prosac_sampler.h prosac_sampler.cpp
    prosac.h
    quality_measurement.h
    random_sampler.h random_sampler.cpp
    ransac.h
    sample_consensus_estimator.h
    sampler.h
)

###############
## Unit Test ##
###############
# COLMAP_ADD_TEST(exhaustive_ransac_test exhaustive_ransac_test.cpp)
# COLMAP_ADD_TEST(exhaustive_sampler_test exhaustive_sampler_test.cpp)
# COLMAP_ADD_TEST(lmed_test lmed_test.cpp)
# COLMAP_ADD_TEST(prosac_test prosac_test.cpp)
# COLMAP_ADD_TEST(random_sampler_test random_sampler_test.cpp)
# COLMAP_ADD_TEST(ransac_test ransac_test.cpp)
