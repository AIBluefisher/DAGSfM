// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ESTIMATORS_RIGID_TRANSFORMATION_3D_SRT_H_
#define ESTIMATORS_RIGID_TRANSFORMATION_3D_SRT_H_

#include "math/lm.h"
// #include "math/eigen_alias_definition.h"
#include "util/alignment.h"

namespace DAGSfM {

/** 3D rigid transformation estimation (7 dof)
 * Compute a Scale Rotation and Translation rigid transformation.
 * This transformation provide a distortion-free transformation
 * using the following formulation Xb = S * R * Xa + t.
 * "Least-squares estimation of transformation parameters between two point
 * patterns", Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573
 *
 * \param[in] x1 The first 3xN matrix of euclidean points
 * \param[in] x2 The second 3xN matrix of euclidean points
 * \param[out] S The scale factor
 * \param[out] t The 3x1 translation
 * \param[out] R The 3x3 rotation
 *
 * \return true if the transformation estimation has succeeded
 *
 * \note Need at least 3 points
 */
bool FindRTS(const Mat &x1, const Mat &x2, double *S, Vec3 *t, Mat3 *R);

/**
 * @brief Eigen Levemberg-Marquardt functor to refine translation, Rotation and
 * Scale parameter.
 */
struct lm_SRTRefine_functor : Functor<double> {
  /**
   * @brief Constructor
   * @param inputs Number of inputs (nb elements to refine)
   * @param values Number of samples
   * @param x1 Input samples for first dataset
   * @param x2 Input samples for second dataset
   * @param S Scale
   * @param R Rotation
   * @param t Translation
   */
  lm_SRTRefine_functor(int inputs, int values, const Mat &x1, const Mat &x2,
                       const double &S, const Mat3 &R, const Vec &t);

  /**
   * @brief Computes error given a sample
   * @param x a Sample
   * @param[out] fvec Error for each values
   */
  int operator()(const Vec &x, Vec &fvec) const;

  Mat x1_, x2_;
  Vec3 t_;
  Mat3 R_;
  double S_;
};

/**
 * @brief Eigen LM functor to refine Rotation.
 */
struct lm_RRefine_functor : Functor<double> {
  /**
   * @brief Constructor
   * @param inputs Number of inputs (elements to refine)
   * @param values Number of samples
   * @param x1 Input samples for first dataset
   * @param x2 Input samples for second dataset
   * @param S Scale
   * @param R Rotation
   * @param t Translation
   */
  lm_RRefine_functor(int inputs, int values, const Mat &x1, const Mat &x2,
                     const double &S, const Mat3 &R, const Vec &t);

  /**
   * @brief Computes error given a sample
   * @param x a Sample
   * @param[out] fvec Error for each values
   */
  int operator()(const Vec &x, Vec &fvec) const;

  Mat x1_, x2_;
  Vec3 t_;
  Mat3 R_;
  double S_;
};

/** 3D rigid transformation refinement using LM
 * Refine the Scale, Rotation and translation rigid transformation
 * using a Levenberg-Marquardt opimization.
 *
 * \param[in] x1 The first 3xN matrix of euclidean points
 * \param[in] x2 The second 3xN matrix of euclidean points
 * \param[out] S The initial scale factor
 * \param[out] t The initial 3x1 translation
 * \param[out] R The initial 3x3 rotation
 *
 * \return none
 */
void Refine_RTS(const Mat &x1, const Mat &x2, double *S, Vec3 *t, Mat3 *R);

}  // namespace DAGSfM

#endif  // DAGSfM_GEOMETRY_RIGID_TRANSFORMATION_3D_SRT_HPP
