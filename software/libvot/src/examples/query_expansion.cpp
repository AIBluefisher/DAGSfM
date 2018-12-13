/*
Copyright (c) 2015, Tianwei Shen, HKUST
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

/** \file query_expansion.cpp
 *	\brief query expansion on the image graph (exe)
 */
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <sstream>
#include <fstream>
#include <unordered_set>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "graph/image_graph.h"
#include "utils/io_utils.h"

using namespace std;

bool MatchJump(int state, int tolerance)
{
	if (state >= tolerance)
		return true;
	else
		return false;
}

int main(int argc, char **argv)
{
	if (argc < 4) {
		printf("Usage: %s <sift_file> <groud_truth_match> <match_file> [query_expansion_level] [qe_inlier_threshold]\n", argv[0]);
		printf("Each line of the ground_truth_match file consists of a 5-tuple of the form <pmatch, fmatch, hmatch, index1, index2>\n");
		printf("Each line of the match_file conssits of a 2-tuple of the form <index1, index2>\n");
		return -1;
	}

	const char *sift_file = argv[1];
	const char *ground_truth_match = argv[2];
	const char *match_file = argv[3];
	int query_level = 2;
	int qe_inlier_thresh = 200;

	if (argc == 5)
		query_level = atoi(argv[4]);
	if (argc == 6)
		qe_inlier_thresh = atoi(argv[5]);

	const int inlier_thresh = 20;
	const int min_finlier = 5;
	const float C = 10000;

	vector<string> sift_filenames;
	tw::IO::ExtractLines(sift_file, sift_filenames);
	size_t image_num = sift_filenames.size();

	vector<vector<vot::LinkEdge> > true_matches;
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

		if (index1 < index2) { 	// note that index1 < index2 in the first place since we only do match when index1 < index2
			// fill in the ground truth match matrix
			if (finlier > inlier_thresh) {
				vot::LinkEdge temp(index1, index2, 0.0, nmatch, finlier);
				true_matches[index1].push_back(temp);
				ground_truth_count++;
			}
		}
	}
	fin.close();

	// read match file
	vot::ImageGraph image_graph(image_num);
	size_t match_count = 0;
	size_t hit_count = 0;

	// read the original rank list of vocabulary tree
	vector<vector<size_t> > rank_list;
	rank_list.resize(image_num);
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

		rank_list[index1].push_back(index2);
	}
	fin1.close();

	// (index1, index2) and (index2, index1) are the same. In case that they were counted twice, use a unordered_set
	std::vector<std::unordered_set<int> > vocab_matches;
	vocab_matches.resize(image_num);

	// select the useful first layer connection
	for (int i = 0; i < image_num; i++) {
		int jump_state = 0;     // a state related to whether to jump of out the current match
		for (int j = 0; j < rank_list[i].size(); j++) {
			// if the previous one is a true match, then continue; Otherwise stop matching for this image
			int index1 = i, index2 = rank_list[i][j];
			if (index1 == index2) continue;
			if (index2 < index1) {
				std::swap(index1, index2);
			}
			std::unordered_set<int>::iterator it = vocab_matches[index1].find(index2);
			if (it == vocab_matches[index1].end()) {	// this pair (index1, index2) hasn't been matched
				vocab_matches[index1].insert(index2);
				match_count++;
				// this searches in the ground-truth match list, which simulates the matching process
				int k = 0;
				for (; k < true_matches[index1].size(); k++) {
					if (true_matches[index1][k].dst == index2) {
						hit_count++;
						image_graph.addEdgeu(true_matches[index1][k]);
						jump_state = 0;
						break;
					}
				}
				if (k == true_matches[index1].size()) {   	// no match for this image pair
					jump_state++;
				}
				if (MatchJump(jump_state,3))
					break;
			}
		}
	}

	// output the result
	double precision = (double) hit_count / match_count;
	double recall = (double) hit_count / ground_truth_count;
	cout << "hit / match / ground_truth: " << hit_count << " " << match_count << " " << ground_truth_count << "\n";
	cout << "precision / recall: " << precision << " " << recall << "\n";

	// iterative query expansion
	for (int iter = 0; iter < 2; iter++) {
		vector<vector<vot::LinkEdge> > expansion_lists;
		image_graph.queryExpansion(expansion_lists, query_level, qe_inlier_thresh);

		// recompute precision and recall after query expansion
		for (int i = 0; i < image_num; i++) {
			for (int j = 0; j < expansion_lists[i].size(); j++) {
				int index1 = i, index2 = expansion_lists[i][j].dst;
				if (index1 > index2)
					std::swap(index1, index2);
				std::unordered_set<int>::iterator it = vocab_matches[index1].find(index2);
				if (it == vocab_matches[index1].end()) {
					vocab_matches[index1].insert(index2);
					match_count++;
					for (int k = 0; k < true_matches[index1].size(); k++) {
						if (true_matches[index1][k].dst == index2) {
							hit_count++;
							image_graph.addEdgeu(true_matches[index1][k]);
							break;
						}
					}
				}
			}
		}
		precision = (double) hit_count / match_count;
		recall = (double) hit_count / ground_truth_count;
		cout << "hit / match / ground_truth: " << hit_count << " " << match_count << " " << ground_truth_count << "\n";
		cout << "precision / recall: " << precision << " " << recall << "\n";
	}

	// write the final match pair file
	ofstream fout("match_pairs_file");
	ofstream fout1("expansion_add");
	ofstream fout2("expansion_true");
	if (!fout.is_open()) {
		std::cout << "[MatchFile] Error opening file for writing\n" << std::endl;
		return -1;
	}
	for (int i = 0; i < image_num; i++) {
		for (int j = i+1; j < image_num; j++) {
			std::unordered_set<int>::iterator it = vocab_matches[i].find(j);
			if (it != vocab_matches[i].end()) {
				fout << sift_filenames[i] << " " << sift_filenames[j] << endl;
				int k = 0;
				for (k = 0; k < rank_list[i].size(); k++) {
					if (j == rank_list[i][k])
						break;
				}
				if (k == rank_list[i].size()) {			// not in the rank_list
					fout1 << sift_filenames[i] << " " << sift_filenames[j] << endl;
					for (int l = 0; l < true_matches[i].size(); l++) {
						if (j == true_matches[i][l].dst) {	// is a true match
							fout2 << sift_filenames[i] << " " << sift_filenames[j] << " " << true_matches[i][l].g_match << endl;
							break;
						}
					}
				}
			}
		}
	}
	fout.close();
	fout1.close();
	return 0;
}
