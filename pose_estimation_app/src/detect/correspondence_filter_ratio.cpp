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

#include "correspondence_filter_ratio.h"

using namespace covis::detect;

covis::core::Correspondence::VecPtr CorrespondenceFilterRatio::filter(const core::Correspondence::Vec& corr) {
    // Sanity checks
    COVIS_ASSERT(_threshold >= 0.0f && _threshold <= 1.0f);
    
    covis::core::Correspondence::VecPtr result(new covis::core::Correspondence::Vec);
    result->reserve(corr.size());
    for(covis::core::Correspondence::Vec::const_iterator it = corr.begin(); it != corr.end(); ++it) {
        COVIS_ASSERT_MSG(it->size() >= 2, "Correspondences must have two matches for Lowe's ratio!");
        const float ratio = it->distance[0] / it->distance[1];
        if(ratio <= _threshold)
            result->push_back(core::Correspondence(it->query, it->match[0], ratio));
    }
    
    return result;
}
