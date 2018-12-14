// BSD 3-Clause License

// Copyright (c) 2018, 陈煜
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SFM_ALIGNED_RANSAC_SIMILARITY_HPP
#define SFM_ALIGNED_RANSAC_SIMILARITY_HPP

#include <vector>
#include <random>
#include <ctime>
#include <algorithm>

#include "i23dSFM/geometry/rigid_transformation3D_srt.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"

#include "Ransac.hpp"

using namespace std;
using namespace Eigen;

namespace i23dSFM {
inline double ReprojectionErr(const Vector3d& point1,
                              const Vector3d& point2,
                              const Matrix3d& R,
                              const Vector3d& t,
                              const double& scale)
{
    Vector3d transformed_point = scale * R * point1 + t;
    return (transformed_point - point2).norm();
}

inline void RansacSimilarity(const vector<Vector3d>& points1,
                             const vector<Vector3d>& points2,
                             vector<Vector3d>& inliers1,
                             vector<Vector3d>& inliers2,
                             Matrix3d& R,
                             Vector3d& t,
                             double &scale,
                             const double threshold = 0.001,
                             const double p = 0.99)
{
    int N = 10000;
    const int max_iteration = 3000;
    int sample_count = 0;
    int s = 5;   // every time we sample 4 points pair
    int size = points1.size();
    int inlier_num = 0;

    srand((unsigned int)time(NULL));

    vector<int> indeces;
    for (int i = 0; i < size; i++) { indeces.push_back(i); }

    while (max_iteration > sample_count) {
        vector<Vector3d> putative_inliers1, putative_inliers2;
        random_shuffle(indeces.begin(), indeces.end());
        for (int i = 0; i < s; i++) {
            putative_inliers1.push_back(points1[indeces[i]]);
            putative_inliers2.push_back(points2[indeces[i]]); 
        }    

        MatrixXd x1 = MatrixXd::Zero(3, s);
        MatrixXd x2 = MatrixXd::Zero(3, s);
        for(int i = 0; i < s; i++) {
            x1.col(i) = putative_inliers1[i];
            x2.col(i) = putative_inliers2[i];
        }

        Matrix3d R = Matrix3d::Identity();
        Vector3d t = Vector3d::Zero();
        double scale = 1.0;
        i23dSFM::geometry::FindRTS(x1, x2, &scale, &t, &R);
        i23dSFM::geometry::Refine_RTS(x1, x2, &scale, &t, &R);

        vector<Vector3d>().swap(putative_inliers1);
        vector<Vector3d>().swap(putative_inliers2);
        for (int i = 0; i < size; i++) {
            double reproj_err = ReprojectionErr(points1[i], points2[i], R, t, scale);
            if (reproj_err < threshold) {
                putative_inliers1.push_back(points1[i]);
                putative_inliers2.push_back(points2[i]);
            }
        }

        if (inlier_num < putative_inliers1.size()) {
            inlier_num = putative_inliers1.size();
            inliers1.swap(putative_inliers1);
            inliers2.swap(putative_inliers2);
        }

        double ol_ratio = 1.0 - (double)inlier_num / (double)size;
        // cout << "inlier ratio: " << (double)inlier_num / (double)size << endl;
        // N = GetSampleCount(s, ol_ratio, p);
        sample_count++;
    }
}

} // namespace i23dSFM

#endif