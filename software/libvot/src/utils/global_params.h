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

/*! \file global_params.h
 * \brief global parameters and utility functions
 *
 * This file contains some global parameters shared by the whole libvot project, such as feature type enum, etc.
 */

#ifndef VOT_GLOBAL_PARAMS_H
#define VOT_GLOBAL_PARAMS_H

#define DTYPE unsigned char
#define LTYPE float
#define FDIM 128

namespace vot{
/**
 * @brief global parameters
 */
class GlobalParam
{
    public:
        static int Verbose;
};

/**
 * @brief feature type used in libvot_feature
 */
enum LIBVOT_FEATURE_TYPE
{
	OPENCV_SIFT = 0,	//!< opencv sift feature type
	VLFEAT_SIFT = 1,	//!< vlfeat sift feature type
	VLFEAT_COVDET = 2,	//!< vlfeat covariant detector (affine adaptation) with sift detector
};

/**
 * @brief sift feature type used in vocabulary tree
 */
enum SiftType
{
	E3D_SIFT = 0,		//!< the standard internal sift data structure of libvot
	OPENMVG_FEAT = 1	//!< the sift data structure used in openmvg
};

}	// end of namespace vot
#endif  //VOT_GLOBAL_PARAMS_H
