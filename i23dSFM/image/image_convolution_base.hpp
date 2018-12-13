// Copyright (c) 2014 Romuald Perrot, Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_IMAGE_IMAGE_CONVOLUTION_BASE_HPP
#define I23DSFM_IMAGE_IMAGE_CONVOLUTION_BASE_HPP

namespace i23dSFM {
namespace image {
  /**
   ** Filter an extended row [halfKernelSize][row][halfKernelSize]
   ** @param buffer data to filter
   ** @param kernel kernel array
   ** @param rsize buffer length
   ** @param ksize kernel length
  **/
  template<class T1, class T2> inline
  void conv_buffer_( T1* buffer, const T2* kernel, int rsize, int ksize )
  {
    for ( size_t i = 0; i < rsize; ++i )
    {
      T2 sum( 0 );
      for ( size_t j = 0; j < ksize; ++j )
      {
        sum += buffer[i + j] * kernel[j];
      }
      buffer[i] = sum;
    }
  }
} // namespace image
} // namespace i23dSFM

#endif // I23DSFM_IMAGE_IMAGE_CONVOLUTION_BASE_HPP
