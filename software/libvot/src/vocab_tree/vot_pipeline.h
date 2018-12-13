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

/*! \file vot_pipeline.h
 * \brief vocabulary tree pipeline, including training, building and querying.
 *
 * This file contains the complete pipeline of vocabulary tree.
 */
#ifndef VOT_PIPELINE_H
#define VOT_PIPELINE_H

#include "utils/global_params.h"

namespace vot
{
///
/// \brief BuildVocabTree: build a vocabulary tree using a set of sift files
/// \param sift_list: the file path of the sift list
/// \param output_filename: the file path of the output tree file
/// \param depth: depth of the vocabulary tree
/// \param branch_num: branch number
/// \param sift_type: feature type
/// \param thread_num: thread number for multi-thread processing
/// \return true if success
///
bool BuildVocabTree(const char *sift_list,
                    const char *output_filename,
                    int depth = 6, int branch_num = 8,
                    SiftType sift_type = E3D_SIFT,
                    int thread_num = 1);

///
/// \brief BuildImageDatabase: build a image database with pre-trained vocabulary tree
/// \param sift_list: the file path of the sift list
/// \param input_tree: the file path of the tree file
/// \param output_filename: the file path of the output image database
/// \param sift_type: feature type
/// \param start_id: start id for the images, if not built from new (append)
/// \param thread_num: thread number for multi-thread processing
/// \return true if success
///
bool BuildImageDatabase(const char *sift_list,
                        const char *input_tree,
                        const char *output_filename,
                        SiftType sift_type = E3D_SIFT,
                        size_t start_id = 0,
                        int thread_num = 1);

///
/// \brief QueryDatabase: query the image database with a sift list file
/// \param image_db: the file path of the input image database
/// \param query_sift_list: the file path of the query sift list file
/// \param match_output: the rank list output
/// \param sift_type: feature type
/// \param thread_num: thread number for multi-thread processing
/// \return true if success
///
///
bool QueryDatabase(const char *image_db,
                   const char *query_sift_list,
                   const char *match_output,
                   SiftType sift_type = E3D_SIFT,
                   int thread_num = 1);

///
/// \brief FilterMatchList: filter output match file and output TOP-k rank lists
/// \param sift_list: the file path of the sift list
/// \param match_list: the file path of the output match file
/// \param output: the output top-k matched pairs
/// \param num_matches: the number top-k images
/// \param output_adjacency_svg_list: the file path to the svg output adjacency list
/// \return true if success
///
bool FilterMatchList(const char *sift_list, const char *match_list, const char *output, size_t num_matches, const char * output_adjacency_svg_list = nullptr);

}

#endif	// VOT_PIPELINE_H
