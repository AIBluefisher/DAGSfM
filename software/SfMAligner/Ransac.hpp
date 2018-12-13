/*
Copyright (c) 2018, Yu Chen
All rights reserved.
*/

#ifndef SFM_ALIGNER_RANSAC_HPP
#define SFM_ALIGNER_RANSAC_HPP

#include <vector>
#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

namespace i23dSFM {

inline int GetSampleCount(int s, double ol_ratio, double p = 0.99)
{
    return log(1 - p) / log(1 - pow(1 - ol_ratio, (double)s));
}
}   // namespace i23dSFM

#endif
