/*
Copyright (c) 2015, Tianwei Shen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of libvot nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file compute_stats.cpp
 *	\brief compute statistics of the retrieval results (exe)
 */
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <cmath>
#include <cassert>
#include <cstdlib>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "utils/io_utils.h"

using namespace std;

float Fnorm(Eigen::MatrixXf &f)
{
	float norm = 0;
	for (int i = 0; i < f.rows(); i++) {
		for (int j = 0; j < f.cols(); j++) {
			norm += f(i, j) * f(i, j);
		}
	}
	return sqrt(norm);
}

int main(int argc, char **argv)
{
	if (argc != 4)
	{
		printf("Usage: %s <sift_file> <groud_truth_match> <match_file>\n", argv[0]);
		printf("Each line of the ground_truth_match file consists of a 5-tuple of the form <pmatch, fmatch, hmatch, index1, index2>\n");
		printf("Each line of the match_file consists of a 2-tuple of the form <index1, index2>\n");
		return -1;
	}

	const char *sift_file = argv[1];
	const char *ground_truth_match = argv[2];
	const char *match_file = argv[3];
	const int inlier_thresh = 20;

	vector<string> sift_filenames;
	tw::IO::ExtractLines(sift_file, sift_filenames);
	size_t image_num = sift_filenames.size();
	vector<unordered_set<size_t> > true_matches;
	true_matches.resize(image_num);

	// read ground truth match file
	ifstream fin(ground_truth_match);
	if (!fin.is_open()) {
		cout << "Failed to open the ground truth match\n";
		return -1;
	}

	string line;
	size_t index1, index2;
	int nmatch, finlier, hinlier;
	size_t ground_truth_count = 0;
	while (!fin.eof()) {
		std::getline(fin, line);
		if (line == "")
			continue;
		std::stringstream ss;
		ss << line;
		ss >> nmatch >> finlier >> hinlier >> index1 >> index2;
		if (index1 > image_num || index2 > image_num) {
			cout << "Error: invalid index pair, exit...\n";
			return -1;
		}
		if (finlier > inlier_thresh && index1 < index2) {
			// note that index1 < index2
			true_matches[index1].insert(index2);
			ground_truth_count++;
		}
	}
	fin.close();

	// read match file
	std::vector<std::unordered_set<int> > match_log;
	match_log.resize(image_num);
	size_t match_count = 0;
	size_t hit_count = 0;
	ifstream fin1(match_file);
	if (!fin1.is_open()) {
		cout << "Failed to open the match file\n";
		return -1;
	}
	while (!fin1.eof()) {
		std::getline(fin1, line);
		if (line == "")
			continue;
		std::stringstream ss;
		ss << line;
		ss >> index1 >> index2;

		if (index1 < index2) {
			match_count++;
			match_log[index1].insert(index2);
			std::unordered_set<size_t>::iterator it = true_matches[index1].find(index2);
			if (it != true_matches[index1].end()) {
				hit_count++;
			}
		}
		else { 			// index1 > index2
			std::unordered_set<int>::iterator it = match_log[index2].find(index1);
			if (it == match_log[index2].end()) {
				match_count++;
				std::unordered_set<size_t>::iterator it1 = true_matches[index2].find(index1);
				if (it1 != true_matches[index2].end()) {
					hit_count++;
				}
			}
		}
	}
	fin1.close();

	// output the result
	double precision = (double) hit_count / match_count;
	double recall = (double) hit_count / ground_truth_count;
	cout << "hit / match / ground_truth: " << hit_count << " " << match_count << " " << ground_truth_count << "\n";
	cout << "precision / recall: " << precision << " " << recall << "\n";

	return 0;
}
