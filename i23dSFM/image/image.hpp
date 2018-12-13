// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_IMAGE_HPP
#define I23DSFM_IMAGE_HPP

// Get rid of the specific MSVC compiler warnings.
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
# define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <vector>

#include "i23dSFM/numeric/numeric.h"

#include "i23dSFM/image/image_container.hpp"
#include "i23dSFM/image/pixel_types.hpp"
#include "i23dSFM/image/image_converter.hpp"
#include "i23dSFM/image/image_drawing.hpp"
#include "i23dSFM/image/image_concat.hpp"
#include "i23dSFM/image/image_io.hpp"
#include "i23dSFM/image/sample.hpp"

#include "i23dSFM/image/image_convolution_base.hpp"
#include "i23dSFM/image/image_convolution.hpp"
#include "i23dSFM/image/image_filtering.hpp"
#include "i23dSFM/image/image_resampling.hpp"
#include "i23dSFM/image/image_diffusion.hpp"

#endif /* I23DSFM_IMAGE_HPP */
