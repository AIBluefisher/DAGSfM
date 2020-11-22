// Tianwei Shen <shentianweipku@gmail.com>
// adapted from http://www.robots.ox.ac.uk/~vgg/data/oxbuildings/compute_ap.cpp

/** \file oxford5k.h
 *	\brief contain slightly modified functions (mainly code style) of oxford5k benchmark dataset.
 */

#ifndef VOT_BENCHMARK_OXFORD5K_H
#define VOT_BENCHMARK_OXFORD5K_H
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <cassert>

std::vector<std::string> load_list(const std::string &fname)
{
	std::vector<std::string> ret;
	std::ifstream fobj(fname.c_str());
	if (!fobj.good()) { std::cerr << "File " << fname << " not found!\n"; exit(-1); }
	std::string line;
	while (getline(fobj, line)) {
		ret.push_back(line);
	}
	return ret;
}

template<class T>
std::set<T> vector_to_set(const std::vector<T>& vec)
{
	return std::set<T>(vec.begin(), vec.end());
}

float compute_ap(const std::set<std::string> &pos, const std::set<std::string> &amb,
                 const std::vector<std::string> &ranked_list)
{
	float old_recall = 0.0;
	float old_precision = 1.0;
	float ap = 0.0;

	size_t intersect_size = 0;
	size_t i = 0;
	size_t j = 0;
	for ( ; i < ranked_list.size(); ++i) {
		if (amb.count(ranked_list[i])) continue;
		if (pos.count(ranked_list[i])) intersect_size++;

		float recall = intersect_size / (float)pos.size();
		float precision = intersect_size / (j + 1.0);

		ap += (recall - old_recall)*((old_precision + precision)/2.0);

		old_recall = recall;
		old_precision = precision;
		j++;
	}
	return ap;
}

struct Oxford5kGT
{
	std::string filename;	// image file name without prefix and suffix
	int id;					// image id that corresponds to the rank in image list
	float  xl, yl, xu, yu;	// lower and upper point of the region
};

std::ostream &operator<<(std::ostream &os, const Oxford5kGT &gt)
{
	os << gt.filename << " " << gt.id << " " << gt.xl << " " << gt.yl << " " << gt.xu << " " << gt.yu;
    return os;
}

#endif	//VOT_BENCHMARK_OXFORD5K_H
