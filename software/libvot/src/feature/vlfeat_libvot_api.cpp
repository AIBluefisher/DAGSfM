
/** \file vlfeat_libvot_api.cpp
 *	\brief libvot-side APIs of vlfeat feature implementation
 *
 * 	It contains some utility functions for the use of vlfeat in libvot
 */
#include "vlfeat_libvot_api.h"

#include <cmath>
#include <cstring>

namespace vot {
bool Vlfeature2LibvotSift(std::vector<Vlfeature> const & vlfeature_vec,
                          unsigned char const * data,
                          int image_width, int image_height, int num_channel,
                          SiftData &sift_data)
{
	sift_data.clear();

	int sift_version;
	if (num_channel == 3)
		sift_version = ('V' + ('5' << 8) + ('.' << 16) + ('0' << 24));
	else    //num_channel = 1
		sift_version = ('V' + ('4' << 8) + ('.' << 16) + ('0' << 24));
	sift_data.setVersion(sift_version);
	int feature_num = vlfeature_vec.size();
	sift_data.setFeatureNum(feature_num);

	LTYPE *&lp = sift_data.getLocPointer();
	DTYPE *&dp = sift_data.getDesPointer();
	int loc_dim = sift_data.getLocDim();
	int des_dim = sift_data.getDesDim();
	lp = new LTYPE [loc_dim * feature_num];
	dp = new DTYPE [des_dim * feature_num];

	for (int i = 0; i < feature_num; i++) {
		//location (2 float), unsigned char rgb (1 float), scale (1 float), orientation (1 float);
		Vlfeature const & vlfeature = vlfeature_vec[i];
		lp[i*loc_dim + 0] = (float) vlfeature.x;
		lp[i*loc_dim + 1] = (float) vlfeature.y;

		if (num_channel == 3) {
			int x,y;
			x = (int)vlfeature.x;
			y = (int)vlfeature.y;
			int index = (y * image_width + x) * num_channel;
			memcpy(lp + i*loc_dim + 2, (data + index), num_channel * sizeof(unsigned char));
		}
		else {
			lp[i*loc_dim + 2] = 0;
		}
		lp[i*loc_dim + 3] = vlfeature.scale;
		lp[i*loc_dim + 4] = vlfeature.orientation;

		// save descriptors
		float const *desc = vlfeature.descr;
		for (int j = 0; j < des_dim; j++) {
			dp[i * des_dim + j] = (DTYPE) floor(0.5 + 512.0f * (*(desc+j)));
		}
	}

	return true;
}

void VlCovariantDetection(unsigned char const *data,
                          std::vector<Vlfeature> &vlfeature_vec,
                          int image_width, int image_height,
                          VlFeatParam const &vlfeat_param)
{
	// Copy image data
	vl_sift_pix *fdata =(vl_sift_pix*)malloc(image_width * image_height * sizeof(vl_sift_pix));
	for (int q = 0; q < image_width * image_height; ++q) {
		fdata[q] = (float)data[q];
	}

	// create a detector object
	VlCovDet *covdet = vl_covdet_new(VL_COVDET_METHOD_HESSIAN);
	// set various parameters (optional)
	vl_covdet_set_first_octave(covdet, 0);
	//vl_covdet_set_octave_resolution(covdet, octaveResolution);
	if (vlfeat_param.peak_thresh > 0)
		vl_covdet_set_peak_threshold(covdet, vlfeat_param.peak_thresh);
	if (vlfeat_param.edge_thresh > 0)
		vl_covdet_set_edge_threshold(covdet, vlfeat_param.edge_thresh);
	//vl_covdet_set_target_num_features(covdet, target_num_features);
	//vl_covdet_set_use_adaptive_suppression(covdet, use_adaptive_suppression);

	// process the image and run the detector
	vl_covdet_put_image(covdet, fdata, image_width, image_height);
	vl_covdet_detect(covdet);

	// drop features outside the image boundary (set margin to be 1 using recommended default)
	vl_covdet_drop_features_outside(covdet, 1);
	// compute the affine shape of the features
	vl_covdet_extract_affine_shape(covdet);
	// compute the orientation of the features
	vl_covdet_extract_orientations(covdet);

	// get feature descriptors
	vl_size numFeatures = vl_covdet_get_num_features(covdet);
	VlCovDetFeature const *feature = (VlCovDetFeature const *)vl_covdet_get_features(covdet);
	//std::cout << feature[0].frame.a11 << " " << feature[0].frame.a12 << " " << feature[0].frame.a22 << " " << feature[0].frame.a21 << std::endl;
	VlSiftFilt *sift = vl_sift_new(16, 16, 1, 3, 0);
	vl_index i;
	vl_size dimension = 128;
	vl_index patchResolution = 15;
	double patchRelativeExtent = 7.5;
	double patchRelativeSmoothing = 1;
	vl_size patchSide = 2 * patchResolution + 1;
	double patchStep = (double)patchRelativeExtent / patchResolution;
	std::vector<float> desc(dimension * numFeatures);
	std::vector<float> patch(patchSide * patchSide);
	std::vector<float> patchXY(2 * patchSide * patchSide);

	if (vlfeat_param.magnif > 0)
		vl_sift_set_magnif(sift, vlfeat_param.magnif);

	for (i = 0; i < (signed)numFeatures; ++i) {
		Vlfeature vlfeature;
		vlfeature.x = feature[i].frame.x;
		vlfeature.y = feature[i].frame.y;
		vlfeature.scale = feature[i].laplacianScaleScore;
		vlfeature.orientation = feature[i].orientationScore;

		vl_covdet_extract_patch_for_frame(covdet, &patch[0],
		        patchResolution, patchRelativeExtent,
		        patchRelativeSmoothing, feature[i].frame);

		vl_imgradient_polar_f(&patchXY[0], &patchXY[1],
		        2, 2 * patchSide,
		        &patch[0], patchSide, patchSide, patchSide);

		vl_sift_calc_raw_descriptor(sift, &patchXY[0],
		        &desc[dimension * i], (int)patchSide, (int)patchSide,
		        (double)(patchSide - 1) / 2, (double)(patchSide - 1) / 2,
		        (double)patchRelativeExtent / (3.0 * (4 + 1) / 2) / patchStep,
		        VL_PI / 2);

		for (int k = 0; k < 128; ++k) {
			vlfeature.descr[k] = (desc[dimension * i + k] < 0.5) ? desc[dimension * i + k] : 0.5;
		}

		vlfeature_vec.push_back(vlfeature);
	}
	// release resource
	vl_sift_delete(sift);
	vl_covdet_delete(covdet);
	if (fdata) {
		free(fdata);
		fdata = nullptr;
	}

	return;
}

void VlFeatureDetection(unsigned char const *data,
                        std::vector<Vlfeature> &vlfeature_vec,
                        int image_width, int image_height,
                        VlFeatParam const &vlfeat_param)
{
	// Copy image data
	vl_sift_pix * fdata =(vl_sift_pix*)malloc(image_width * image_height * sizeof(vl_sift_pix));
	for (int q = 0; q < image_width * image_height; ++q) {
		fdata[q] = (float)data[q];
	}

	// Initialize the sift detector
	VlSiftFilt * filt = vl_sift_new(image_width, image_height, -1, 3, -1);
	if (vlfeat_param.edge_thresh >= 0)
		vl_sift_set_edge_thresh(filt, vlfeat_param.edge_thresh);
	if (vlfeat_param.peak_thresh >= 0)
		vl_sift_set_peak_thresh(filt, vlfeat_param.peak_thresh);
	if (vlfeat_param.magnif >= 0)
		vl_sift_set_magnif(filt, vlfeat_param.magnif);

	if (!filt) {
		std::cerr << "[VlFeature] Fail to create SIFT filter.\n";
		return;
	}

	// Detect features
	bool first = 1;
	vl_bool err = VL_ERR_OK;
	while(1) {
		// Calculate the GSS for the next octave
		if (first) {
			first = 0;
			err = vl_sift_process_first_octave(filt, fdata);
		}
		else
			err = vl_sift_process_next_octave(filt);

		if(err)
			break;

		vl_sift_detect(filt);
		VlSiftKeypoint const * keys = vl_sift_get_keypoints(filt);
		int nkeys = vl_sift_get_nkeypoints(filt);

		for (int i = 0; i < nkeys; ++i) {
			double angles[4];
			VlSiftKeypoint const *k = keys + i;
			int nangles = vl_sift_calc_keypoint_orientations(filt, angles, k);

			vl_sift_pix descr[128];
			for (int q = 0; q < nangles; ++q) {
				vl_sift_calc_keypoint_descriptor(filt, descr, k, angles[q]);

				Vlfeature vlfeature;
				vlfeature.x = k->x;
				vlfeature.y = k->y;
				vlfeature.scale = k->sigma;
				vlfeature.orientation = angles[q];

				for (int k = 0; k < 128; ++k) {
					vlfeature.descr[k] = (descr[k] < 0.5) ? descr[k] : 0.5;
				}

				vlfeature_vec.push_back(vlfeature);
			}
		}
	}

	// release filter
	if (filt) {
		vl_sift_delete(filt);
		filt = nullptr;
	}

	// release image data
	if (fdata) {
		free(fdata);
		fdata = nullptr;
	}

	return;
}

int RunVlFeature(unsigned char *data, int image_width, int image_height, int num_channel,
                 SiftData &sift_data, VlFeatParam const &vlfeat_param)
{
	unsigned char *gray_data = nullptr;
	if (num_channel == 3) {      	//rgb images, convert to greyscale
		size_t num_pixel = image_width * image_height;
		gray_data = new unsigned char [num_pixel];
		size_t i = 0, j = 0;
		for (; i < num_pixel; i++, j+=3) {
			gray_data[i] = static_cast<unsigned char>(std::min(int(0.10454f * data[j+2]+0.60581f * data[j+1]+0.28965f * data[j]), 255));
		}
	}
	else {
		gray_data = data;
	}

	std::vector<Vlfeature> vlfeature_vec;
	switch (vlfeat_param.feature_type) {
		case LIBVOT_FEATURE_TYPE::VLFEAT_SIFT:
			VlFeatureDetection(gray_data, vlfeature_vec, image_width, image_height, vlfeat_param);
			break;
		case LIBVOT_FEATURE_TYPE::VLFEAT_COVDET:
			VlCovariantDetection(gray_data, vlfeature_vec, image_width, image_height, vlfeat_param);
			break;
		default:
			std::cerr << "[VlFeature] Unsupported feature type!\n";
			return -1;
	}

	if (vlfeature_vec.size() == 0) {
		std::cout << "[VlFeature] Fail to detect feature points.\n";
		return -1;
	}

	Vlfeature2LibvotSift(vlfeature_vec, data, image_width, image_height, num_channel, sift_data);

	if (gray_data != nullptr && num_channel == 3)
		delete[] gray_data;
	int num_features = (int)vlfeature_vec.size();
	return num_features;
}
}	// end of namespace vot
