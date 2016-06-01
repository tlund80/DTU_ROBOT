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

#ifndef COVIS_DETECT_CORRESPONDENCE_FILTER_RATIO_H
#define COVIS_DETECT_CORRESPONDENCE_FILTER_RATIO_H

// Own
#include "correspondence_filter_base.h"

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class CorrespondenceFilterRatio
         * @brief Correspondence filter class using Lowe's nearest to second nearest feature distance ratio
         * 
         * This class requires a set of correspondences, all containing both the nearest and the second nearest matching
         * distance. Then it re-ranks all correspondences using the ratio of these two distances. A low value thus means
         * a good correspondence since the best match is unique. All correspondences with a ratio below 0.8 are returned
         * by default, but this can be specified using the threshold parameter. All output correspondences are re-ranked
         * by the computed ratio.
         * 
         * For more information, see @cite lowe2004distinctive.
         * 
         * @author Anders Glent Buch
         */
        class CorrespondenceFilterRatio : public CorrespondenceFilterBase {
            public:
                /**
                 * Default constructor: sets upper ratio threshold to the default value of 0.8^2
                 */
                CorrespondenceFilterRatio() : _threshold(0.8 * 0.8) {}
                
                /// Empty destructor
                virtual ~CorrespondenceFilterRatio() {}
                
                /// @copydoc CorrespondenceFilterBase::filter()
                core::Correspondence::VecPtr filter(const core::Correspondence::Vec& corr);
                
                /**
                 * Set ratio threshold
                 * @note Remember to specify this squared, since e.g. @ref FeatureSearch produces squared distances
                 * @param threshold ratio threshold
                 */
                inline void setThreshold(float threshold) {
                    _threshold = threshold;
                }
                
                /**
                 * Get ratio threshold
                 * @return ratio threshold
                 */
                inline float getThreshold() const {
                    return _threshold;
                }
                
            private:
                /// Ratio threshold
                float _threshold;
        };
        
        /**
         * @ingroup filter
         * @brief Filter correspondences using Lowe's ratio
         * @param corr input correspondences
         * @param threshold ratio threshold
         * @return filtered correspondences
         */
        inline core::Correspondence::VecPtr filterCorrespondencesRatio(const core::Correspondence::Vec& corr,
                float threshold = 0.8) {
            CorrespondenceFilterRatio cfr;
            cfr.setThreshold(threshold);
            
            return cfr.filter(corr);
        }
    }
}

#endif
