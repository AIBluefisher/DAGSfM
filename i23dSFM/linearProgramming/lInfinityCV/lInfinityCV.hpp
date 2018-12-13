// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_L_INFINITY_COMPUTER_VISION_H_
#define I23DSFM_L_INFINITY_COMPUTER_VISION_H_

// Structure and motion problem solver
#include "i23dSFM/linearProgramming/lInfinityCV/tijsAndXis_From_xi_Ri.hpp"
#include "i23dSFM/linearProgramming/lInfinityCV/tijsAndXis_From_xi_Ri_noise.hpp"
#include "i23dSFM/linearProgramming/lInfinityCV/triplet_tijsAndXis_kernel.hpp"

// Pose estimation solver
#include "i23dSFM/linearProgramming/lInfinityCV/resection.hpp"
#include "i23dSFM/linearProgramming/lInfinityCV/resection_kernel.hpp"
// N-View Triangulation solver
#include "i23dSFM/linearProgramming/lInfinityCV/triangulation.hpp"

//-------------
//-- Global SfM
//-------------
// Compute from global translation by using 2-views relative translations guess
#include "i23dSFM/linearProgramming/lInfinityCV/global_translations_fromTij.hpp"
// Compute from global translation by using 3-views relative translations guess
#include "i23dSFM/linearProgramming/lInfinityCV/global_translations_fromTriplets.hpp"

#endif // I23DSFM_L_INFINITY_COMPUTER_VISION_H_
