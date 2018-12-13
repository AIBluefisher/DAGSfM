// Tianwei Shen, HKUST.

/** \file sift_match_test.cpp
 *	\brief testing feature matching
 */

#include <iostream>
#include <string>
#include "utils/data_types.h"
#include "matching/matching_datatypes.h"
#include "matching/feature_matching.h"
#include "feature/opencv_libvot_api.h"

#ifdef LIBVOT_USE_OPENCV
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#endif

#include "gflags/gflags.h"

using namespace std;
DEFINE_bool(opencv_feature, false, "generate opencv sift from images");
DEFINE_bool(show_matching, true, "show matching using opencv::imshow");

int main(int argc, char **argv)
{
	google::ParseCommandLineFlags(&argc, &argv, true);
	if (argc != 3)
		return -1;
	string sift_file1 = string(argv[1]);
	string sift_file2 = string(argv[2]);

	// (additional): if image files are in the same directory, show the match result
	bool is_image_exist = false;
	string sift_imagefile1_root = tw::IO::SplitPathExt(sift_file1).first;
	string sift_imagefile2_root = tw::IO::SplitPathExt(sift_file2).first;
	string sift_imagefile1, sift_imagefile2;
	vector<string> image_types;
	image_types.push_back(".jpg");
	image_types.push_back(".JPG");
	image_types.push_back(".png");
	image_types.push_back(".PNG");
	int i = 0;
	while(i < image_types.size() && !is_image_exist)
	{
		sift_imagefile1 = sift_imagefile1_root + image_types[i];
		sift_imagefile2 = sift_imagefile2_root + image_types[i];
		if(tw::IO::IsFileExist(sift_imagefile1) && tw::IO::IsFileExist(sift_imagefile2))
		{
			cout << "[sift_match_test] image files found: " << sift_imagefile1 << " and " << sift_imagefile2 << "\n";
			is_image_exist = true;
			break;
		}
		i++;
	}

	std::vector<cv::KeyPoint> key_points1, key_points2;
	cv::Mat desc1, desc2;
	cv::Mat img_1, img_2;
	if(is_image_exist)
	{
		img_1 = cv::imread(sift_imagefile1, CV_LOAD_IMAGE_COLOR);
		img_2 = cv::imread(sift_imagefile2, CV_LOAD_IMAGE_COLOR);
	}
	if(FLAGS_opencv_feature)	// generate opencv sift feature from images
	{
		if(!is_image_exist)
		{
			std::cerr << "[sift_match_test] image files not exist.\n";
			return -1;
		}

		cv::SiftDescriptorExtractor cv_sift_extractor;
		cv_sift_extractor.detect(img_1, key_points1);
		cv_sift_extractor.compute(img_1, key_points1, desc1);
		cv_sift_extractor.detect(img_2, key_points2);
		cv_sift_extractor.compute(img_2, key_points2, desc2);
		cout << "[sift_match_test] desc " << desc1.rows << " " << desc1.cols <<  " " << desc1.type() <<  endl;
		cout << "[sift_match_test] desc " << desc2.rows << " " << desc2.cols << " " << desc2.type() << endl;
		cout << "[sift_match_test] got " << key_points1.size() << " sift1 descriptors\n";
		cout << "[sift_match_test] got " << key_points2.size() << " sift2 descriptors\n";
	}
	else	// read libvot sift file
	{
		//// This file doesn't support other sift type
		vot::SiftData sift1, sift2;
		sift1.ReadSiftFile(sift_file1);
		sift2.ReadSiftFile(sift_file2);

		// prepare opencv matching
		vot::LibvotSift2OpencvKeyPoints(sift1, key_points1, desc1);
		vot::LibvotSift2OpencvKeyPoints(sift2, key_points2, desc2);
		assert(key_points1.size() == desc1.rows);
		assert(key_points2.size() == desc2.rows);
		cout << "[sift_match_test] Convert " << key_points1.size() << " sift1 descriptors\n";
		cout << "[sift_match_test] Convert " << key_points2.size() << " sift2 descriptors\n";

		// test libvot matching
		vot::SiftMatchPair match_pair(sift_file1);
		vot::MatchParam match_param;
		if(FLAGS_show_matching && is_image_exist)	// show matches
		{
			if(!vot::PairwiseSiftMatching(sift1, sift2, match_pair, match_param,
			                              sift_imagefile1, sift_imagefile2))
				return -1;
		}
		else	// don't show matches
		{
			if(!vot::PairwiseSiftMatching(sift1, sift2, match_pair, match_param))
				return -1;
		}
	}

	return 0;
}
