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

/** \file sequence_match.cpp
 *	\brief generate sequential match pair (exe)
 */
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

#include "utils/io_utils.h"

using namespace std;

int main(int argc, char **argv)
{
	if (argc != 3) {
		printf("Usage: %s <sift_file_list> <output_file>\n", argv[0]);
		return -1;
	}
	const char *sift_input_file = argv[1];
	const char *output_file = argv[2];
	int window_size = 3;

	std::vector<std::string> sift_filenames;
	tw::IO::ExtractLines(sift_input_file, sift_filenames);
	int image_num = sift_filenames.size();

	FILE *fp;
	fp = fopen(output_file, "w");
	if (fp == NULL) {
		printf("Reading error\n");
		return -1;
	}

	for (int i = 0; i < image_num; i++) {
		for (int j = 1; j < window_size+1 && i+j < image_num; j++) {
			fprintf(fp, "%s %s\n", sift_filenames[i].c_str(), sift_filenames[i+j].c_str());
		}
	}

	fclose(fp);

	return 0;
}
