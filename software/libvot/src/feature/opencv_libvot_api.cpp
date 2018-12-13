
/** \file opencv_libvot_api.cpp
 *	\brief libvot-side APIs of opencv feature implementation
 *
 * 	It contains Some utility functions for the use of opencv in libvot
 */
#include "opencv_libvot_api.h"

#include "utils/global_params.h"
#include "utils/data_types.h"

namespace vot {
bool OpencvKeyPoints2libvotSift(std::vector<cv::KeyPoint> &key_points,
                                cv::Mat &descriptors,
                                SiftData &sift_data)
{
	sift_data.clear();
	int num_features = key_points.size();
	sift_data.setFeatureNum(num_features);
	DTYPE *&dp = sift_data.getDesPointer();
	LTYPE *&lp = sift_data.getLocPointer();

	int des_dim = sift_data.getDesDim();
	int loc_dim = sift_data.getLocDim();
	dp = new DTYPE [num_features * des_dim];
	lp = new LTYPE [num_features * loc_dim];

	for (int i = 0; i < num_features; i++) {
		cv::KeyPoint &kp = key_points[i];
		lp[i*loc_dim + 0] = kp.pt.x;	// x coordinate
		lp[i*loc_dim + 1] = kp.pt.y;	// y coordinate
		lp[i*loc_dim + 2] = 0;			// color (not available)
		lp[i*loc_dim + 3] = kp.size;	// scale
		lp[i*loc_dim + 4] = kp.angle;	// orientation

		// save descriptors
		for (int j = 0; j < des_dim; j++)
			dp[i * des_dim + j] = static_cast<DTYPE>(descriptors.at<float>(i,j));
	}

	return true;
}

bool LibvotSift2OpencvKeyPoints(SiftData &sift_data,
                                std::vector<cv::KeyPoint> &key_points,
                                cv::Mat &descriptors)
{
	int num_features = sift_data.getFeatureNum();
	key_points.resize(num_features);
	DTYPE *&dp = sift_data.getDesPointer();
	LTYPE *&lp = sift_data.getLocPointer();
	int des_dim = sift_data.getDesDim();
	int loc_dim = sift_data.getLocDim();

	descriptors.create(num_features, des_dim, CV_32FC1);
	for (int i = 0; i < num_features; i++) {
		cv::KeyPoint &kp = key_points[i];
		kp.pt.x = lp[i*loc_dim + 0];
		kp.pt.y = lp[i*loc_dim + 1];
		//color = lp[i*loc_dim + 2];
		kp.size = lp[i*loc_dim + 3];
		kp.angle = lp[i*loc_dim + 4];

		// load descriptors
		for (int j = 0; j < des_dim; j++) {
			descriptors.at<float>(i,j) = static_cast<float>(dp[i*des_dim + j]);
		}
	}

	return true;
}
} 	// end of namespace vot
