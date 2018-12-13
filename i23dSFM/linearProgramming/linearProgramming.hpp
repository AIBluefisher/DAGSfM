// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_LINEAR_PROGRAMMING_H_
#define I23DSFM_LINEAR_PROGRAMMING_H_

#include "i23dSFM/linearProgramming/linearProgrammingInterface.hpp"
#include "i23dSFM/linearProgramming/linearProgrammingOSI_X.hpp"
#ifdef I23DSFM_HAVE_MOSEK
#include "i23dSFM/linearProgramming/linearProgrammingMOSEK.hpp"
#endif

#include "i23dSFM/linearProgramming/bisectionLP.hpp"

// Multiple View Geometry solver that rely on Linear programming formulations
#include "i23dSFM/linearProgramming/lInfinityCV/lInfinityCV.hpp"

#endif // I23DSFM_LINEAR_PROGRAMMING_H_
