
UNIT_TEST(i23dSFM rigid_transformation3D_srt "i23dSFM_numeric")

UNIT_TEST(i23dSFM half_space_intersection "i23dSFM_numeric;${CLP_LIBRARIES};${COINUTILS_LIBRARY};${OSI_LIBRARY}")

UNIT_TEST(i23dSFM frustum_intersection "i23dSFM_numeric;i23dSFM_multiview_test_data;i23dSFM_multiview;${CLP_LIBRARIES};${COINUTILS_LIBRARY};${OSI_LIBRARY}")
