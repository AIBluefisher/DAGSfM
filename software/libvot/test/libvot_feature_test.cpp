#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <iostream>

#include "libvot_config.h"
#include "utils/io_utils.h"
#include "utils/global_params.h"
#include "utils/data_types.h"
#include "feature/opencv_libvot_api.h"
#include "feature/vlfeat_libvot_api.h"

extern "C" {
#include <vl/generic.h>
#include <vl/sift.h>
}

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
	if(argc != 2)
		return -1;

	// ----------    OPENCV feature ------------------
	const std::string image_filename = std::string(argv[1]);
	const std::string feat_filename = tw::IO::SplitPathExt(image_filename).first + ".sift";
	// load the image in BGR format
	const cv::Mat input = cv::imread(image_filename, CV_LOAD_IMAGE_COLOR);

	cv::SiftDescriptorExtractor cv_sift_detector;
	std::vector<cv::KeyPoint> cv_keypoints;
	cv::Mat sift_descriptors;
	cv_sift_detector.detect(input, cv_keypoints);
	cv_sift_detector.compute(input, cv_keypoints, sift_descriptors);
	vot::SiftData sift_data;
	vot::OpencvKeyPoints2libvotSift(cv_keypoints, sift_descriptors, sift_data);

	sift_data.SaveSiftFile(feat_filename);

	// ----------- VLfeat feature -----------------------
	vot::VlFeatParam vlfeat_param;
	vlfeat_param.edge_thresh = 10;
	vlfeat_param.peak_thresh = 2.5;
	int num_features = vot::RunVlFeature(input.data, input.cols, input.rows, 3, sift_data, vlfeat_param);
	if(!sift_data.SaveSiftFile(feat_filename))
	{
		cerr << "[Extract Feature] sift_data.SaveSiftFile error.\n";
		return -1;
	}

    return 0;
}
