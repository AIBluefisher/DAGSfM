/*
Copyright (c) 2015 - 2016, Tianwei Shen
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

/** \file web_search.cpp
 *	\brief an image search interface specifically used for single image query (exe)
 */
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <numeric>
#include <algorithm>

#include "gflags/gflags.h"
#include "vocab_tree/vocab_tree.h"
#include "utils/io_utils.h"
#include "utils/data_structures.h"

using namespace std;

DEFINE_string(output_folder, "", "Output folder for ranked list");
DEFINE_int32(match_num, 200, "The length of the ranked list (top-k)");
DEFINE_bool(output_filename, true, "Output image name instead of image index");

int main(int argc, char **argv)
{
	google::ParseCommandLineFlags(&argc, &argv, true);
	if (argc < 4) {
		printf("Usage: %s <input_sift_file> <db_image_list> <db_path> "
		       "[--output_folder <output_folder>] [num_matches <num_matches>]\n", argv[0]);
		return -1;
	}

	const char *sift_filename = argv[1];
	const char *db_image_list = argv[2];
	const char *image_db = argv[3];
	const std::string output_folder = FLAGS_output_folder;
	const int num_matches = FLAGS_match_num;
	const bool is_output_name = FLAGS_output_filename;

	// get db image filenames
	std::vector<std::string> db_image_filenames;
	tw::IO::ExtractLines(db_image_list, db_image_filenames);

	vot::VocabTree *tree = new vot::VocabTree();
	tree->ReadTree(image_db);
	std::cout << "[VocabMatch] Successfully read vocabulary tree (with image database) file " << image_db << std::endl;
	tree->Show();

	vot::SiftData sift;
	std::string sift_str(sift_filename);
	const std::string sift_str_ext = tw::IO::SplitPathExt(sift_str).second;
	if (sift_str_ext == "sift") {
		if (!sift.ReadSiftFile(sift_str)) {
			std::cout << "[VocabMatch] ReadSiftFile error\n";
			exit(-1);
		}
	}
	else if(sift_str_ext == "desc") {
		if (!sift.ReadOpenmvgDesc<DTYPE, FDIM>(sift_str)) {
			std::cout << "[VocabMatch] ReadOpenmvgDesc error\n";
			exit(-1);
		}
	}
	else
		std::cout << "[VocabMatch] Ext not supported by libvot\n";

	// get rank list output path from sift_filename
	std::string output_path = sift_str;
	output_path = tw::IO::GetFilename(sift_str) + ".rank";
	output_path = tw::IO::JoinPath(output_folder, output_path);

	FILE *match_file = fopen(output_path.c_str(), "w");
	if (match_file == NULL) {
		std::cout << "[VocabMatch] Fail to open the match file.\n";
		return -1;
	}
	int db_image_num = tree->database_image_num;
	float *scores = new float[db_image_num];
	size_t *indexed_scores = new size_t[db_image_num];
	memset(scores, 0.0, sizeof(float) * db_image_num);
	tree->Query(sift, scores);

	std::iota(indexed_scores, indexed_scores+db_image_num, 0);
	std::sort(indexed_scores, indexed_scores+db_image_num, [&](size_t i0, size_t i1) {
		return scores[i0] > scores[i1];
	});
	int top = num_matches > db_image_num ? db_image_num : num_matches;
	if (is_output_name) {
		for (size_t j = 0; j < top; j++) {
			std::string image_name = tw::IO::GetFilename(db_image_filenames[indexed_scores[j]]);
			fprintf(match_file, "%s\n", image_name.c_str());
		}
	}
	else {
		for (size_t j = 0; j < top; j++)
			fprintf(match_file, "%zu\n", indexed_scores[j]);
	}
	std::cout << "[VocabMatch] Successful query and the rank list is output to " << output_path << ".\n";

	delete [] scores;
	delete [] indexed_scores;
	return 0;
}
