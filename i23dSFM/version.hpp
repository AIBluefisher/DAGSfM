// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef I23DSFM_VERSION_H_
#define I23DSFM_VERSION_H_

#define I23DSFM_VERSION_MAJOR 0
#define I23DSFM_VERSION_MINOR 9
#define I23DSFM_VERSION_REVISION 0

// Preprocessor to string conversion
#define I23DSFM_TO_STRING_HELPER(x) #x
#define I23DSFM_TO_STRING(x) I23DSFM_TO_STRING_HELPER(x)

// I23dSFM version as a string; for example "0.9.0".
#define I23DSFM_VERSION_STRING I23DSFM_TO_STRING(I23DSFM_VERSION_MAJOR) "." \
                             I23DSFM_TO_STRING(I23DSFM_VERSION_MINOR) "." \
                             I23DSFM_TO_STRING(I23DSFM_VERSION_REVISION)

#endif  // I23DSFM_VERSION_H_
