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

#ifndef COVIS_DETECT_CORRESPONDENCE_FILTER_DISTANCE_H
#define COVIS_DETECT_CORRESPONDENCE_FILTER_DISTANCE_H

// Own
#include "correspondence_filter_base.h"

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class CorrespondenceFilterDistance
         * @brief Correspondence filter based on matching distance
         * 
         * This class requires a set of correspondences containing the nearest matching distance between e.g. features
         * or points. Then it re-ranks all correspondences using the distances.
         * 
         * Remember to set the distance threshold using @ref setThreshold().
         * 
         * @author Anders Glent Buch
         */
        class CorrespondenceFilterDistance : public CorrespondenceFilterBase {
            public:
                /// Empty constructor
                CorrespondenceFilterDistance() : _threshold(-1) {}
                
                /// Empty destructor
                virtual ~CorrespondenceFilterDistance() {}
                
                /// @copydoc CorrespondenceFilterBase::filter()
                core::Correspondence::VecPtr filter(const core::Correspondence::Vec& corr);
                
                /**
                 * Set distance threshold, must be positive
                 * @note Remember to specify this squared, since e.g. @ref PointSearch and @ref FeatureSearch produce
                 * squared distances
                 * @param threshold distance threshold
                 */
                inline void setThreshold(float threshold) {
                    _threshold = threshold;
                }
                
                /**
                 * Get distance threshold
                 * @return distance threshold
                 */
                inline float getThreshold() const {
                    return _threshold;
                }
                
            private:
                /// Distance threshold
                float _threshold;
        };
        
        /**
         * @ingroup filter
         * @brief Filter correspondences based on nearest neighbor matching distance
         * @param corr input correspondences
         * @param threshold distance threshold
         * @return filtered correspondences
         */
        inline core::Correspondence::VecPtr filterCorrespondencesDistance(const core::Correspondence::Vec& corr,
                float threshold) {
            CorrespondenceFilterDistance cfr;
            cfr.setThreshold(threshold);
            
            return cfr.filter(corr);
        }
    }
}

#endif
