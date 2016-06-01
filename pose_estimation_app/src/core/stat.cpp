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

#include "stat.h"
#include "macros.h"

size_t covis::core::otsu(const size_t* hist, size_t bins) {
    // Sanity check
    COVIS_ASSERT(bins > 0);

    // Sum and mean of histogram bin values
    float sum = 0.0f;
    float meann = 0.0f;
    for(size_t x = 0; x < bins; ++x) {
        sum += float(hist[x]);
        meann += float(x * hist[x]);
    }
    meann /= sum;

    // Partial forward/backward sums of histogram probabilities
    float omega1 = 0.0f;
    float omega2 = 1.0f;

    // Partial forward/backward sums of histogram probabilities weighted by bin values
    float px1 = 0.0f;
    float px2 = meann;

    // Maximal inter-class variance
    float interClassVarianceMax = 0.0f;

    // Resulting threshold
    size_t t = 0;

    // Start
    for(size_t x = 0; x < bins; ++x) {
        // Bin probability
        const float pi = float(hist[x]) / sum;

        // Update class probabilities
        omega1 += pi;
        if(omega1 <= 0.0f || omega1 >= 1.0f)
            continue;

        omega2 -= pi;
        if(omega2 <= 0.0f || omega2 >= 1.0f)
            continue;

        // Update partial sums
        const float pxi = pi * x;
        px1 += pxi;
        px2 -= pxi;

        // Compute class means
        const float mu1 = px1 / omega1;
        const float mu2 = px2 / omega2;

        // Compute inter-class variance
        const float mu12sq = (mu1 - mu2) * (mu1 - mu2);
        const float interClassVariance = omega1 * omega2 * mu12sq;

        // Update results if inter-class variance is maximized
        if(interClassVariance > interClassVarianceMax) {
            interClassVarianceMax = interClassVariance;
            t = x;
        }
    }

    return t;
}
