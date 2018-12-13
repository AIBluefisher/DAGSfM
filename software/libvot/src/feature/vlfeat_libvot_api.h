
/** \file vlfeat_libvot_api.h
 *	\brief libvot-side APIs of vlfeat feature
 *
 * 	It contains some utility functions for the use of vlfeat in libvot
 */
#ifndef VOT_VLFEAT_LIBVOT_API_H
#define VOT_VLFEAT_LIBVOT_API_H

extern "C" {
#include "vl/generic.h"
#include "vl/sift.h"
#include "vl/covdet.h"
}
#include "utils/data_types.h"
#include "utils/global_params.h"
#include <string>

namespace vot {
/**
 * @brief a parameter struct used in calling vlfeat sift function
 */
struct VlFeatParam {
	VlFeatParam () : edge_thresh(-1), peak_thresh(-1), magnif(-1), feature_type(OPENCV_SIFT) {}
	double edge_thresh;
	double peak_thresh;
	double magnif;
	LIBVOT_FEATURE_TYPE feature_type;
};

/**
 * @brief a struct containing the data of vlfeat features
 */
struct Vlfeature {
	float x;
	float y;
	float scale;
	float orientation;
	float descr[128];
};

int RunVlFeature(unsigned char *data, int image_width, int image_height, int num_channel,
                 SiftData &sift_data, VlFeatParam const & vlfeat_param);
}	// end of namespace vot

#endif	//VOT_VLFEAT_LIBVOT_API_H
