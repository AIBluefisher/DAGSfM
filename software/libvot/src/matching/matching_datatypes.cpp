/** \file matching_datatypes.cpp
 *	\brief matching data structures implementation
 *
 */
// Author: Tianwei Shen <shentianweipku@gmail.com>

#include "matching_datatypes.h"

#include <iostream>
#include "utils/data_types.h"
#include "utils/io_utils.h"

#ifdef LIBVOT_USE_OPENCV
#include "opencv_matching_api.h"
#endif

namespace vot {

// ====================================================
// ---------------- SiftMatchPair class ---------------
// ====================================================
SiftMatchPair::SiftMatchPair(std::string filename1)
{
	filename1_ = filename1;
	match_pairs_ = NULL;
}

SiftMatchPair::SiftMatchPair(int match_buf[][2], const int nmatch,
const std::vector<bool>& homography_inlier_flag, const std::vector<bool>& fundamental_inlier_flag,
std::string filename1, std::string filename2,
const Eigen::Matrix3d &homography, const Eigen::Matrix3d &fundamental_matrix,
int homography_inlier_num, int fundamental_inlier_num)
{
	match_pairs_ = new FeatureMatchPair[nmatch];
	nmatch_ = nmatch;
	filename1_ = filename1;
	filename2_ = filename2;
	homography_ = homography;
	fundamental_matrix_ = fundamental_matrix;
	homography_inlier_num_ = homography_inlier_num;
	fundamental_inlier_num_ = fundamental_inlier_num;

	for(size_t i = 0; i < (size_t)(nmatch_); i++)
	{
		match_pairs_[i].first = match_buf[i][0];
		match_pairs_[i].second = match_buf[i][1];
		match_pairs_[i].flag = (FeatureMatchPair::OUTLIER | homography_inlier_flag[i] | (fundamental_inlier_flag[i] << 1));
	}
}

// copy constructor
SiftMatchPair::SiftMatchPair(const SiftMatchPair &obj):
    filename1_(obj.filename1_), filename2_(obj.filename2_),
    nmatch_(obj.nmatch_), homography_inlier_num_(obj.homography_inlier_num_),
    fundamental_inlier_num_(obj.fundamental_inlier_num_),
    homography_(obj.homography_), fundamental_matrix_(obj.fundamental_matrix_)
{
	match_pairs_ = new FeatureMatchPair [obj.nmatch_];
	for (int i = 0; i < obj.nmatch_; i++) {
		match_pairs_[i] = obj.match_pairs_[i];
	}
}

bool SiftMatchPair::operator==(const SiftMatchPair &rhs)
{
	//NOTE(tianwei): this is a incomplete comparison
	if (this->filename1_ == rhs.filename1_ && this->filename2_ == rhs.filename2_ &&
	    this->nmatch_ == rhs.nmatch_ &&
	    this->fundamental_inlier_num_ == rhs.fundamental_inlier_num_ &&
	    this->homography_inlier_num_ == rhs.homography_inlier_num_)
		return true;
	else
		return false;
}

// copy assignment operator
SiftMatchPair & SiftMatchPair::operator=(const SiftMatchPair & rhs)
{
	if (*this == rhs)
		return *this;
	filename1_ = rhs.filename1_;
	filename2_ = rhs.filename2_;
	nmatch_ = rhs.nmatch_;
	homography_inlier_num_ = rhs.homography_inlier_num_;
	fundamental_inlier_num_ = rhs.fundamental_inlier_num_;
	homography_ = rhs.homography_;
	fundamental_matrix_ = rhs.fundamental_matrix_;
	FeatureMatchPair *originMatchPairs = match_pairs_;
	if (rhs.match_pairs_ != NULL) {
		match_pairs_ = new FeatureMatchPair [rhs.nmatch_];
		for(int i = 0; i < rhs.nmatch_; i++)
			match_pairs_[i] = rhs.match_pairs_[i];
	}
	else
		match_pairs_ = NULL;
	if (originMatchPairs != NULL)
		delete [] originMatchPairs;
	return *this;
}

// destructor
SiftMatchPair::~SiftMatchPair()
{
	if (match_pairs_ != NULL)
		delete [] match_pairs_;
}

bool SiftMatchPair::WriteSiftMatchPair(FILE *file) const
{
	// write information
	int filename2_length = filename2_.size();
	fwrite((void*)&filename2_length, sizeof(int), 1, file);
	fwrite((void*)filename2_.c_str(), sizeof(char), filename2_length, file);
	fwrite((void*)&nmatch_, sizeof(int), 1, file);
	fwrite((void*)&homography_inlier_num_, sizeof(int), 1, file);
	fwrite((void*)&fundamental_inlier_num_, sizeof(int), 1, file);

	fwrite((void*)homography_.data(), sizeof(double), 9, file);
	fwrite((void*)fundamental_matrix_.data(), sizeof(double), 9, file);

	// write match correspondence (int, int, char)
	fwrite((void*)match_pairs_, sizeof(FeatureMatchPair), nmatch_, file);

	return true;
}

bool SiftMatchPair::ReadSiftMatchPair(FILE *file)
{
	char filename2_temp[256];
	int filename2_length;

	int check = fread((void*)&filename2_length, sizeof(int), 1, file);
	if (check != 1)  return false;

	fread((void*)filename2_temp, sizeof(char), filename2_length, file);
	filename2_temp[filename2_length] = '\0';
	filename2_ = std::string(filename2_temp);

	fread((void*)&nmatch_, sizeof(int), 1, file);
	fread((void*)&homography_inlier_num_, sizeof(int), 1, file);
	fread((void*)&fundamental_inlier_num_, sizeof(int), 1, file);

	fread((void*)homography_.data(), sizeof(double), 9, file);
	fread((void*)fundamental_matrix_.data(), sizeof(double), 9, file);

	// read match correspondence (int, int, char)
	if (match_pairs_ != NULL) {
		delete [] match_pairs_;
		match_pairs_ = NULL;
	}
	match_pairs_ = new FeatureMatchPair [nmatch_];
	fread((void*)match_pairs_, nmatch_, sizeof(FeatureMatchPair), file);
	return true;
}

void SiftMatchPair::showInfo() const
{
	std::cout << "filename1: " << filename1_ << '\n';
	std::cout << "filename2: " << filename2_ << '\n';
	std::cout << "nmatch: " << nmatch_ << '\n';
	std::cout << "homography_inlier_num: " << homography_inlier_num_ << '\n';
	std::cout << "fundamental_inlier_num: " << fundamental_inlier_num_ << '\n';
	std::cout << "homography: " << homography_ << '\n';
	std::cout << "fundamental_matrix: " << fundamental_matrix_ << '\n';

	// random shuffle
	std::vector<int> index_array(nmatch_);
	for (size_t i = 0; i < index_array.size(); ++i) index_array[i] = static_cast<int>(i);
	std::random_shuffle(index_array.begin(), index_array.end());

	for (size_t i = 0; i < 10; i++) {
		int index = index_array[i];
		match_pairs_[index].showInfo();
	}
}

// Helper functions
const FeatureMatchPair* SiftMatchPair::matchPairs() const { return match_pairs_; }
FeatureMatchPair* SiftMatchPair::matchPairs() { return match_pairs_; }


// ====================================================
// ---------------- SiftMatchFile class ---------------
// ====================================================
SiftMatchFile::SiftMatchFile()
{
	image_num_ = 0;
}

SiftMatchFile::SiftMatchFile(std::string mat_path)
{
	mat_filename_ = mat_path;
	image_num_ = 0;
}

bool SiftMatchFile::ReadMatchFile(std::string mat_path)
{
	FILE *fd = fopen(mat_path.c_str(), "rb");
	if (fd == NULL) {
		std::cerr << "Can't read .mat file. Exit...\n";
		return false;
	}
	std::string sift_path = tw::IO::SplitPath(tw::IO::SplitPathExt(mat_path).first + ".sift").second;
	SiftMatchPair temp_mp = SiftMatchPair(sift_path);
	while (temp_mp.ReadSiftMatchPair(fd)) {
		match_pairs_.push_back(temp_mp);
	}
	image_num_ = match_pairs_.size();
	fclose(fd);
	return true;
}

const int & SiftMatchFile::getMatchNum() const { return image_num_; }
const std::string & SiftMatchFile::getMatFilename() const { return mat_filename_; }
const std::vector<SiftMatchPair> & SiftMatchFile::getSiftMatchPairs() const { return match_pairs_; }


// ====================================================
// ---------------- SiftMatcher class ---------------
// ====================================================
SiftMatcher::SiftMatcher(int max_sift,
                         MatcherDevice match_device,
                         int feature_dim,
                         DistanceType dist_type):
    max_sift_(max_sift),
    dist_type_(dist_type),
    feature_dim_(feature_dim),
    match_device_(match_device),
    matcher_(nullptr) {}

SiftMatcher::~SiftMatcher()
{
	if (matcher_) delete matcher_;
}

bool SiftMatcher::Init()
{
	if (matcher_) delete matcher_;
	switch (match_device_) {
		case SiftMatcher::SIFT_MATCH_CPU:
			matcher_ = new SiftMatcherCPU(max_sift_);
			break;
		case SiftMatcher::SIFT_MATCH_CUDA:
#ifdef LIBVOT_USE_CUDA
			matcher_ = new SiftMatcherCUDA(max_sift_);
			break;
#else
			matcher_ = NULL;
			return false;
#endif
		case SiftMatcher::SIFT_MATCH_GLSL:
			matcher_ = new SiftMatcherGL(max_sift_);
			break;
		case SiftMatcher::SIFT_MATCH_OPENCV:
#ifdef LIBVOT_USE_OPENCV
			matcher_ = new SiftMatcherOpencv(max_sift_);
			break;
#else
			match_device_ = SiftMatcher::SIFT_MATCH_CPU;
			return false;
#endif
		default:
			return false;
	}

	return true;
}

void SiftMatcher::SetMaxSift(int max_sift) { max_sift_ = max_sift; }

const int SiftMatcher::GetMaxSift() const { return (const int) max_sift_; }

bool SiftMatcher::SetMatchDevice(int device)
{
	switch (device) {
		case SiftMatcher::SIFT_MATCH_CPU:
			match_device_ = device;
			return true;
		case SiftMatcher::SIFT_MATCH_CUDA:
#ifdef LIBVOT_USE_CUDA				// if cude is available
			match_device_ = device;
			return true;
#else
			match_device_ = SiftMatcher::SIFT_MATCH_CPU;
			return false;
#endif
		case SiftMatcher::SIFT_MATCH_GLSL:
			match_device_ = device;
			return true;
		case SiftMatcher::SIFT_MATCH_OPENCV:
#ifdef LIBVOT_USE_OPENCV
			match_device_ = device;
			return true;
#else
			match_device_ = SiftMatcher::SIFT_MATCH_CPU;
			return false;
#endif
		default:
			match_device_ = SiftMatcher::SIFT_MATCH_CPU;
			return false;
	}
}

int SiftMatcher::GetMatchDevice() const { return (int) match_device_; }

bool SiftMatcher::SetDescriptors(int index, int num, const float *descriptors)
{
	if (matcher_)
		return matcher_->SetDescriptors(index, num, descriptors);
	return false;
}

bool SiftMatcher::SetDescriptors(int index, int num, const unsigned char *descriptors)
{
	if (matcher_)
		return matcher_->SetDescriptors(index, num, descriptors);
	return false;
}

bool SiftMatcher::SetLocation(int index, int feature_num, const float *loc, const int loc_dim)
{
	if (matcher_)
		return matcher_->SetLocation(index, feature_num, loc, loc_dim);
	return true;
}

int SiftMatcher::GetSiftMatch(int max_match,  int match_buffer[][2], int mutual_best_match)
{
	if (matcher_)
		return matcher_->GetSiftMatch(max_match, match_buffer, mutual_best_match);
	return false;
}

bool SiftMatcher::ShowMatches(std::string imagefile1, std::string imagefile2)
{
	if (matcher_)
		return matcher_->ShowMatches(imagefile1, imagefile2);
	return false;
}

// ====================================================
// ---------------- SiftMatcherCPU class ---------------
// ====================================================
SiftMatcherCPU::SiftMatcherCPU(int max_sift): SiftMatcher(max_sift)
{
}

SiftMatcherCPU::~SiftMatcherCPU() {}

bool SiftMatcherCPU::SetDescriptors(int index, int num, const unsigned char *descriptors)
{
	if (index != 0 && index != 1) {
		std::cerr << "[SiftMatcherCPU] SetDescriptors get index other than 0/1\n";
		return false;
	}
	if (num > max_sift_)
		num = max_sift_;
	num_sift_[index] = num;

	assert(sizeof(float) == 4);		// a float is 4 char
	sift_buffer_[index].resize(feature_dim_ * num/4);
	memcpy(&sift_buffer_[index][0], descriptors, feature_dim_ * num);

	return true;
}

bool SiftMatcherCPU::SetDescriptors(int index, int num, const float *descriptors)
{
	if (index != 0 || index != 1) {
		std::cerr << "[SiftMatcherCPU] SetDescriptors get index other than 0/1\n";
		return false;
	}
	if (num > max_sift_)
		num = max_sift_;
	num_sift_[index] = num;

	assert(sizeof(float) == 4);		// a float is 4 char
	sift_buffer_[index].resize(feature_dim_ * num/4);
	// convert float sift to unsigned char sift
	unsigned char *pub = (unsigned char*) &sift_buffer_[index][0];
	const int num_char = feature_dim_ * num;
	for (size_t i = 0; i < num_char; i++)
		pub[i] = int(512 * descriptors[i] + 0.5);

	return true;
}

float SiftMatcherCPU::GetDescriptorDist(std::vector<float> vec1, std::vector<float> vec2)
{
	float distance;
	switch(dist_type_) {
		case ARCCOS:
		{
			// compute l2 norm of each descriptors
			float vec1_norm = 0, vec2_norm = 0, dot_prod = 0;
			for (int i = 0; i < feature_dim_; i++) {
				vec1_norm += vec1[i] * vec2[i];
				vec2_norm += vec2[i] * vec2[i];
			}
			vec1_norm = std::sqrt(vec1_norm);
			vec2_norm = std::sqrt(vec2_norm);
			// normalize  each descriptor using l2norm
			for (int i = 0; i < feature_dim_; i++) {
				vec1[i] /= vec1_norm;
				vec2[i] /= vec2_norm;
				dot_prod += vec1[i] * vec2[i];
			}

			distance = dot_prod;
			break;
		}
		default:
			break;
	}
	return distance;
}

// brute force cpu matcher
int SiftMatcherCPU::GetSiftMatch(int max_match,
                                 int match_buffer[][2],
int mutual_best_match)
{
	int num_matches = 0;
	std::vector<float> vec1, vec2;
	vec1.resize(feature_dim_);
	vec2.resize(feature_dim_);
	unsigned char *pv1 = (unsigned char*) &sift_buffer_[0][0];
	for (int i = 0, c1 = 0; i < num_sift_[0]; i++) {
		for(int j = 0; j < feature_dim_; j++, c1++)
			vec1[j] = pv1[c1];

		float best_match, sbest_match;
		int best_index = -1, sbest_index = -1;
		// initialize match distance
		switch (dist_type_) {
			case ARCCOS:
				best_match = -1; sbest_match = -1;
				break;
			default:	// TODO(tianwei): other distance type is invalid
				break;
		}
		unsigned char *pv2 = (unsigned char*) &sift_buffer_[1][0];
		for (int j = 0, c2 = 0; j < num_sift_[1]; j++) {
			for (int k = 0; k < feature_dim_; k++, c2++)
				vec2[k] = pv2[c2];
			float dist = GetDescriptorDist(vec1, vec2);
			if (dist > best_match) {
				sbest_match = best_match;
				sbest_index = best_index;
				best_match = dist;
				best_index = j;
			}
		}
		switch (dist_type_) {
			case ARCCOS:
			{
				const float distmax = 0.7;
				const float ratiomax = 0.8;
				if (best_match > distmax && (sbest_match/best_match) < ratiomax) {
					if (num_matches < max_match) {
						match_buffer[num_matches][0] = i;
						match_buffer[num_matches][1] = best_index;
						num_matches++;
					}
				}
				break;
			}
			default:
				break;
		}
	}

	return num_matches;
}

// ====================================================
// ---------------- SiftMatcherCUDA class ---------------
// ====================================================
SiftMatcherCUDA::SiftMatcherCUDA(int max_sift): SiftMatcher(max_sift)
{
}

SiftMatcherCUDA::~SiftMatcherCUDA()
{

}

int SiftMatcherCUDA::GetSiftMatch(int max_match,
                                  int match_buffer[][2],
int mutual_best_match)
{
	int num_matches = 0;

	return num_matches;
}

// ====================================================
// ---------------- SiftMatcherGLSL class ---------------
// ====================================================
SiftMatcherGL::SiftMatcherGL(int max_sift): SiftMatcher(max_sift)
{
}

SiftMatcherGL::~SiftMatcherGL()
{

}

int SiftMatcherGL::GetSiftMatch(int max_match,
                                int match_buffer[][2],
int mutual_best_match)
{
	int num_matches = 0;

	return num_matches;
}

}	// end of namespace vot
