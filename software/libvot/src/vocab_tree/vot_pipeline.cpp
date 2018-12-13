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

/*! \file vot_pipeline.cpp
 * \brief vocabulary tree pipeline implementation.
 *
 * This file contains the complete pipeline of vocabulary tree.
 */
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cassert>
#include <unordered_map>
#include <algorithm>
#include <set>
#include <sstream>
#include <thread>
#include <mutex>
#include <numeric>

#include "vot_pipeline.h"
#include "vocab_tree.h"
#include "utils/io_utils.h"
#include "utils/data_types.h"
#include "utils/data_structures.h"

#include "vectorGraphics/svgDrawer.hpp"


#define MAX_ARRAY_SIZE 8388608  // 2^23

namespace
{
	/** Random sample "sample_num" number of numbers from [0, 1, ... , total] (total >= sample_num)*/
	std::vector<size_t> RandomSample(size_t total, size_t sample_num)
	{
		std::vector<size_t> total_numbers(total, 0);
		std::iota(total_numbers.begin(), total_numbers.end(), 0);
		std::random_shuffle(total_numbers.begin(), total_numbers.end());
		std::vector<size_t> samples(total_numbers.begin(), total_numbers.begin() + sample_num);

		return samples;
	}

	/// Export an adjacency matrix as a SVG file
	void AdjacencyMatrixToSVG(const size_t NbImages,
	                          const std::set<std::pair<size_t,size_t> > & corresponding_indexes,
	                          const std::string & sOutName)
	{
		using namespace svg;
		if (!corresponding_indexes.empty()) {
			float scaleFactor = 5.0f;
			svgDrawer svgStream((NbImages+3)*5, (NbImages+3)*5);
			// Go along all possible pair
			for (size_t I = 0; I < NbImages; ++I) {
				for (size_t J = 0; J < NbImages; ++J) {
					// If the pair have matches display a blue boxes at I,J position.
					const auto iterSearch = corresponding_indexes.find(std::make_pair(I,J));
					if (iterSearch != corresponding_indexes.end()) {
						svgStream.drawSquare(J*scaleFactor, I*scaleFactor, scaleFactor/2.0f,
						                     svgStyle().fill("blue").noStroke());
					}
				}
			}
			// Display axes with 0 -> NbImages annotation : _|
			std::ostringstream osNbImages;
			osNbImages << NbImages;
			svgStream.drawText((NbImages+1)*scaleFactor, scaleFactor, scaleFactor, "0", "black");
			svgStream.drawText((NbImages+1)*scaleFactor,
			                   (NbImages)*scaleFactor - scaleFactor, scaleFactor, osNbImages.str(), "black");
			svgStream.drawLine((NbImages+1)*scaleFactor, 2*scaleFactor,
			                   (NbImages+1)*scaleFactor, (NbImages)*scaleFactor - 2*scaleFactor,
			                   svgStyle().stroke("black", 1.0));

			svgStream.drawText(scaleFactor, (NbImages+1)*scaleFactor, scaleFactor, "0", "black");
			svgStream.drawText((NbImages)*scaleFactor - scaleFactor,
			                   (NbImages+1)*scaleFactor, scaleFactor, osNbImages.str(), "black");
			svgStream.drawLine(2*scaleFactor, (NbImages+1)*scaleFactor,
			                   (NbImages)*scaleFactor - 2*scaleFactor, (NbImages+1)*scaleFactor,
			                   svgStyle().stroke("black", 1.0));

			std::ofstream svgFileStream(sOutName.c_str());
			svgFileStream << svgStream.closeSvgFile().str();
		}
	}
}

namespace vot
{
	bool BuildVocabTree(const char *sift_list,
						const char *output_filename,
						int depth, int branch_num,
						SiftType sift_type, int thread_num)
	{
		// read sift filenames, get the total number of sift keys, and allocate memory
		std::vector<std::string> sift_filenames;
		tw::IO::ExtractLines(sift_list, sift_filenames);
		size_t siftfile_num = sift_filenames.size();
		// sample a part of sift files
		size_t memory_size = tw::IO::GetAvailMem() / (1024*1024);	// convert to mb
		size_t tree_memory_size = FDIM * sizeof(DTYPE) * (size_t)pow((double)branch_num, (double)(depth+1)) / (1024*1024);
		size_t max_siftfile_num = (memory_size - tree_memory_size) / 2;
		size_t sample_size = siftfile_num > max_siftfile_num ? max_siftfile_num : siftfile_num;
		std::vector<size_t> siftfile_samples = RandomSample(siftfile_num, sample_size);

		size_t total_keys = 0;
		std::vector<vot::SiftData> sift_data;
		sift_data.resize(sample_size);

		std::cout << "[Build Tree] Reading sift (type " << (int)sift_type << ") files...\n";
		for (size_t i = 0; i < sample_size; i++) {
			switch (sift_type) {
				case E3D_SIFT:
				{
					std::string file_type = tw::IO::SplitPathExt(sift_filenames[i]).second;
					if (file_type != "sift") {
						std::cout << "[Build Tree] Wrong sift type, should match 'sift'\n";
						return false;
					}
					if (sizeof(DTYPE) == 1) {
						if (!sift_data[i].ReadSiftFile(sift_filenames[siftfile_samples[i]]))
							return false;
					}
					else {
						if (!sift_data[i].ReadChar2DTYPE(sift_filenames[siftfile_samples[i]]))
							return false;
					}
					break;
				}
				case OPENMVG_FEAT:
				{
					sift_data[i].ReadOpenmvgDesc<DTYPE, FDIM>(sift_filenames[i]);
					break;
				}
				default:
				{
					std::cout << "[Build Tree] Sift type is not supported. Exit...\n";
					return false;
				}
			}
			total_keys += sift_data[i].getFeatureNum();
		}
		std::cout << "[Build Tree] Total sift keys (Type SIFT5.0): " << total_keys << std::endl;
		if (total_keys == 0) {
			std::cerr << "[Build Tree] Error: No sift keys input, maybe the sift type is wrong. Exit...\n";
			return false;
		}

		const size_t len = (size_t) total_keys * FDIM;
		const size_t num_arrays = len / MAX_ARRAY_SIZE + ((len % MAX_ARRAY_SIZE) == 0 ? 0 : 1);

		// allocate a big chunk of memory to each array
		std::cout << "[Build Tree] Allocate " << len << " bytes memory into " << num_arrays << " arrays\n";
		DTYPE **mem = new DTYPE *[num_arrays];
		size_t remain_length = len;
		for (size_t i = 0; i < num_arrays; i++) {
			const size_t len_curr = remain_length > MAX_ARRAY_SIZE ? MAX_ARRAY_SIZE : remain_length;
			mem[i] = new DTYPE [len_curr];
			remain_length -= len_curr;
		}
		assert(remain_length == 0);

		// allocate a pointer array to sift data
		DTYPE **mem_pointer = new DTYPE *[total_keys];
		size_t off = 0;
		size_t curr_key = 0;
		int curr_array = 0;
		for (size_t i = 0; i < sample_size; i++) {
			int num_keys = sift_data[i].getFeatureNum();
			if (num_keys > 0) {
				DTYPE *dp = sift_data[i].getDesPointer();
				for (int j = 0; j < num_keys; j++) {
					for (int k = 0; k < FDIM; k++) {
						mem[curr_array][off+k] = dp[j * FDIM + k];
					}
					mem_pointer[curr_key] = mem[curr_array] + off;
					curr_key++;
					off += FDIM;
					if (off == MAX_ARRAY_SIZE) {
						off = 0;
						curr_array++;
					}
				}
			}
		}

		// build a vocabulary tree using sift keys
		vot::VocabTree vt;
		if(vt.BuildTree(total_keys, FDIM, depth, branch_num, mem_pointer, thread_num))
			vt.WriteTree(output_filename);

		vot::VocabTree vt1;
		vt1.ReadTree(output_filename);
		if (vt1.Compare(vt)) {
			std::cout << "[VocabTree IO test] vt and vt1 are the same\n";
		}
		else {
			std::cout << "[vocabTree IO test] vt and vt1 are different\n";
		}

		vt.ClearTree();
		// free memory
		delete [] mem_pointer;
		for (size_t i = 0; i < num_arrays; i++)
			delete [] mem[i];
		delete [] mem;
		return true;
	}

	bool BuildImageDatabase(const char *sift_list,
							const char *input_tree,
							const char *output_filename,
							SiftType sift_type,
							size_t start_id,
							int thread_num)
	{
		// read sift filenames, get the total number of sift keys, and allocate memory
		std::vector<std::string> sift_filenames;
		tw::IO::ExtractLines(sift_list, sift_filenames);
		const size_t siftfile_num = sift_filenames.size();

		// TODO(tianwei): a filter key step
		// add image into the database using the inverted list
		vot::VocabTree tree;
		tree.ReadTree(input_tree);
		std::cout << "[BuildDB] Successfully read vocabulary tree file " << input_tree << std::endl;
		tree.Show();
		tree.SetConstantWeight();
		for (size_t i = 0; i < siftfile_num; ++i) {
			SiftData sift_data;
			switch(sift_type) {
				case E3D_SIFT:
				{
					if(sizeof(DTYPE) == 1)		// unsigned char
						sift_data.ReadSiftFile(sift_filenames[i]);
					else
						sift_data.ReadChar2DTYPE(sift_filenames[i]);
				}
					break;
				case OPENMVG_FEAT:
				{
					sift_data.ReadOpenmvgDesc<DTYPE, FDIM>(sift_filenames[i]);
				}
					break;
				default:
				{
					std::cout << "[BuildDB] Sift type is wrong (should be 0 or 1). Exit...\n";
					return false;
				}
			}
			const double mag = tree.AddImage2Tree(start_id + i, sift_data, thread_num);
			std::cout << "[BuildDB] Add image #" << start_id + i << " to database\n";
		}
		tree.ComputeTFIDFWeight(siftfile_num);
		tree.NormalizeDatabase(start_id, siftfile_num);

		std::cout << "[BuildDB] Write vocabulary tree (with image database) to " << output_filename << '\n';
		tree.WriteTree(output_filename);
		tree.ClearTree();

		return true;
	}

	void MultiQueryDatabase(vot::VocabTree *tree,
							std::vector<std::string> *sift_filenames,
							int sift_type,
							size_t first_index,
							size_t num_images, 			// the number of images in the database
							float *scores, 				// the similarity scores given by the query process
							size_t *indexed_scores,		// the rank index of scores
							FILE *match_file,
							std::mutex *match_file_mutex)
	{
		size_t db_image_num = tree->database_image_num;
		for (size_t i = first_index; i < first_index + num_images; i++) {
			match_file_mutex->lock();
			std::cout << "[VocabMatch] Querying image #" << i << " to database\n";
			match_file_mutex->unlock();

			memset(scores, 0, sizeof(float) * db_image_num);
			// read sift data
			SiftData sift_data;
			if (sift_type == E3D_SIFT) {
				if (sizeof(DTYPE) == 1)
					sift_data.ReadSiftFile((*sift_filenames)[i]);
				else
					sift_data.ReadChar2DTYPE((*sift_filenames)[i]);
			}
			else if (sift_type == OPENMVG_FEAT) {
				sift_data.ReadOpenmvgDesc<DTYPE, FDIM>((*sift_filenames)[i]);
			}
			else {
				std::cout << "[VocabMatch] Sift type is wrong (should be 0). Exit...\n";
				exit(-1);
			}
			tree->Query(sift_data, scores);
			std::sort(indexed_scores, indexed_scores+db_image_num, [&](size_t i0, size_t i1) {
				return scores[i0] > scores[i1];
			});
			match_file_mutex->lock();
			for (size_t j = 0; j < db_image_num; j++)
				fprintf(match_file, "%zd %zd %0.4f\n", i, indexed_scores[j], scores[indexed_scores[j]]);
			match_file_mutex->unlock();
		}
	}

	bool QueryDatabase(const char *image_db,
					   const char *query_sift_list,
					   const char *match_output,
					   SiftType sift_type, int thread_num)
	{
		// read tree and image database
		vot::VocabTree *tree = new vot::VocabTree();
		tree->ReadTree(image_db);
		std::cout << "[VocabMatch] Successfully read vocabulary tree (with image database) file " << image_db << std::endl;
		tree->Show();

		// read query image sift data
		std::vector<std::string> sift_filenames;
		tw::IO::ExtractLines(query_sift_list, sift_filenames);
		size_t siftfile_num = sift_filenames.size();

		FILE *match_file = fopen(match_output, "w");
		if (match_file == NULL) {
			std::cout << "[VocabMatch] Fail to open the match file.\n";
		}
		size_t db_image_num = tree->database_image_num;
		if (thread_num == 1) {
			std::vector<float> scores(db_image_num);
			std::vector<size_t> indexed_scores(db_image_num);
			std::iota(indexed_scores.begin(), indexed_scores.end(), 0);

			for (size_t i = 0; i < siftfile_num; i++) {
				std::cout << "[VocabMatch] Querying image #" << i << " to database\n";
				std::fill(scores.begin(), scores.end(), 0);

				// read sift data
				SiftData sift_data;
				switch(sift_type) {
					case E3D_SIFT:
					{
						if(sizeof(DTYPE) == 1)
							sift_data.ReadSiftFile(sift_filenames[i]);
						else
							sift_data.ReadChar2DTYPE(sift_filenames[i]);
						break;
					}
					case OPENMVG_FEAT:
					{
						sift_data.ReadOpenmvgDesc<DTYPE, FDIM>(sift_filenames[i]);
						break;
					}
					default:
					{
						std::cout << "[VocabMatch] Sift type is wrong (should be 0). Exit...\n";
						exit(-1);
					}
				}
				tree->Query(sift_data, &scores[0]);
				std::sort(indexed_scores.begin(), indexed_scores.end(),
				          [&](size_t i0, size_t i1) {return scores[i0] > scores[i1];});
				for(size_t j = 0; j < db_image_num; j++)
					fprintf(match_file, "%zd %zd %0.4f\n", i, indexed_scores[j], scores[indexed_scores[j]]);
			}
		}
		else
		{
			std::mutex match_file_mutex;
			std::vector<std::thread> threads;
			std::vector<std::vector<float> > scores(thread_num);
			std::vector<std::vector<size_t> > indexed_scores(thread_num);
			for (int i = 0; i < thread_num; i++) {
				scores[i].resize(db_image_num);
				indexed_scores[i].resize(db_image_num);
				std::iota(indexed_scores[i].begin(), indexed_scores[i].end(), 0);
			}

			size_t off = 0;
			for (int i = 0; i < thread_num; i++) {
				size_t thread_image = siftfile_num / thread_num;
				if (i == thread_num - 1)
					thread_image = siftfile_num - (thread_num - 1) * thread_image;
				threads.push_back(std::thread(MultiQueryDatabase, tree, &sift_filenames, sift_type, off, thread_image,
				                              &scores[i][0], &indexed_scores[i][0], match_file, &match_file_mutex));
				off += thread_image;
			}
			std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
		}
		fclose(match_file);

		// release memory
		tree->ClearTree();
		delete tree;
		return true;
	}

	bool FilterMatchList(const char *sift_list,
	                     const char *match_list,
	                     const char *output,
	                     size_t num_matches,
	                     const char * output_adjacency_svg_list)
	{
		std::vector<std::string> sift_filenames;
		tw::IO::ExtractLines(sift_list, sift_filenames);
		const size_t image_num = sift_filenames.size();
		num_matches = std::min(image_num, num_matches);

		std::ifstream fin(match_list);
		if (!fin.is_open()) {
			std::cout << "[FilterMatchList] Fail to open the vocabulary tree match output\n";
			return false;
		}
		else {
			std::cout << "[FilterMatchList] Read the vocabulary tree match file: " << match_list << '\n';
		}

		std::string line;
		size_t index0, index1;
		float score;
		std::vector<std::vector<size_t> > top_matches;
		top_matches.resize(image_num);
		while (!fin.eof()) {
			std::getline(fin, line);
			if(line == "")
				continue;
			std::stringstream line_stream;
			line_stream << line;
			line_stream >> index0 >> index1 >> score;
			if (index0 >= sift_filenames.size() || index1 >= sift_filenames.size()) {
				std::cout << "[FilterMatchList] Invalid index pair: " << index0 << " " << index1 << '\n';
				continue;
			}
			// find top-'num_matches' pairs
			if (top_matches[index0].size() < num_matches && index0 != index1 && score > 0) {
				top_matches[index0].push_back(index1);
			}
		}
		fin.close();

		std::vector<std::set<size_t> > filtered_matches(image_num); // get unique match pairs
		for (size_t i = 0; i < image_num; i++) {
			for (const size_t & j : top_matches[i]) {
				index0 = std::min(i, j);
				index1 = std::max(i, j);
				filtered_matches[index0].insert(index1);
			}
		}

		// write match pairs (names and indexes) to two files
		std::string sift_index_file(output);
		std::string sift_name_file = sift_index_file+".txt";
		std::ofstream fout(sift_name_file);				// used for match (sift filenames pairs)
		std::ofstream fout1(sift_index_file);			// used for query expansion (sift index pairs)
		if (!fout.is_open() || !fout1.is_open()) {
			std::cout << "[FilterMatchList] Fail to open the filtered match file\n";
			return false;
		}
		else {
			std::cout << "[FilterMatchList] Write the filtered index file " << sift_index_file << '\n';
		}

		// Build from the pair an adjacency matrix to lot it on disk
		std::set<std::pair<size_t,size_t> > adjacency_matrix;
		for (size_t i = 0; i < image_num; i++) {
			for (const size_t & j : filtered_matches[i]) {
				fout << sift_filenames[i] << " " << sift_filenames[j] << '\n';
				adjacency_matrix.insert( std::make_pair(i, j) );
			}
		}
		fout.close();

		if (output_adjacency_svg_list != nullptr) {
			// Export adjacency matrix as a SVG file
			AdjacencyMatrixToSVG(image_num, adjacency_matrix, output_adjacency_svg_list);

			// Export top matches
			for (size_t i = 0; i < image_num; i++) {
				for (const size_t & j : top_matches[i]) {
					fout1 << i << " " << j << '\n';
				}
			}
			fout1.close();
		}

		return true;
	}
}	// end of namespace vot
