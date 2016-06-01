// Copyright (c) 2013, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef COVIS_CONFIG_H
#define COVIS_CONFIG_H

// CoViS version in various formats
#define COVIS_VERSION_STRING "1.0.0"
#define COVIS_VERSION 1.0.0
#define COVIS_VERSION_MAJOR 1
#define COVIS_VERSION_MINOR 0
#define COVIS_VERSION_PATCH 0

// CoViS version suitable for numerical comparison: MAJOR*10000+MINOR*100+PATCH
#define COVIS_VERSION_CMP 10000

// CoViS revision (will be set to -1 if not detected)
#define COVIS_REVISION 465

// CoViS build type
#define COVIS_BUILD_TYPE "Release"

// Defined if CoViS is statically built
/* #undef COVIS_BUILD_STATIC */

// Log level, possible values defined in core/macros.h
#define COVIS_LOG_LEVEL COVIS_LOG_LEVEL_WARN

// Defined if OpenCV version >= 3 was found
/* #undef OPENCV_3 */

// Defined if FLANN version >= 1.8 was found
/* #undef FLANN_1_8 */

// Defined if Visualization Toolkit was found
#define VTK_FOUND 1

// Defined if CUDA was found
/* #undef CUDA_FOUND */

// Defined if OpenMP was found
#define OPENMP_FOUND 1

// Defined if OpenNI2 was found
#define OPENNI2_FOUND 1

// Defined if NurbsPP was found
/* #undef NURBSPP_FOUND */

#endif
