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

#ifndef COVIS_CORE_DETECTION_H
#define COVIS_CORE_DETECTION_H

// Own
#include "macros.h"

// STL
#include <limits>
#include <map>
#include <vector>

// Eigen
#include <Eigen/Core>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @class Detection
         * @brief Structure for representing a single object detection
         *
         * This struct has three types of quality measures of a detection:
         *   - RMSE: the root mean square error of the inlier points
         *   - Penalty: an internal error function used for ranking detections
         *   - Inlier fraction: the number of inlier object points relative to the total number of points
         *   
         * @author Anders Glent Buch
         */
        struct Detection {
                /// Matrix type
                typedef Eigen::Matrix4f MatrixT;
                
                /// Vector type
                typedef std::vector<Detection> Vec;
                
                /// Extra user-specifiable parameters
                std::map<std::string, float> params;
                
                /**
                 * Constructor for setting all members
                 * @param pose pose
                 * @param rmse RMSE
                 * @param penalty penalty
                 * @param inlierfrac inlier fraction
                 * @param idx index
                 * @param label label
                 */
                Detection(const MatrixT& pose = MatrixT::Identity(),
                        float rmse = std::numeric_limits<float>::max(),
                        float penalty = std::numeric_limits<float>::max(),
                        float inlierfrac = 0.0f,
                        int idx = 0,
                        const std::string& label = "") {
                    this->pose = pose;
                    this->rmse = rmse;
                    this->penalty = penalty;
                    this->inlierfrac = inlierfrac;
                    this->idx = idx;
                    this->label = label;
                }
                
                /// Alignment pose
                MatrixT pose;
                
                /// RMSE of the alignment
                float rmse;
                
                /// Penalty function evaluation of the fit, can be equal to RMSE
                float penalty;
                
                /// Inlier fraction of the detected object
                float inlierfrac;
                
                /// Index of the detected object
                int idx;
                
                /// Name of the detected object
                std::string label;
                
                /**
                 * Return true if this detection is valid, i.e. has more than zero inliers
                 * This can be used for easy check of a detection:
                 * @code
                 * Detection d;
                 * if(d)
                 *   // Do something...
                 * @endcode
                 */
                inline operator bool() const {
                    return inlierfrac > 0.0f;
                }
        };

        /**
         * @ingroup core
         * 
         * Comparator function for sorting detections in ascending order according to penalty
         *
         * @param d1 first detection
         * @param d2 second detection
         * @return comparison
         */
        inline bool cmpDetectionPenaltyAscend(const Detection& d1, const Detection& d2) {
            return d1.penalty < d2.penalty;
        }

        /**
         * @ingroup core
         * 
         * Sort a vector of detections in ascending order according to penalty
         *
         * @param d input/output detections
         */
        inline void sort(Detection::Vec& d) {
            std::sort(d.begin(), d.end(), cmpDetectionPenaltyAscend);
        }
        
        /**
         * @ingroup core
         * 
         * Print a detection to a stream
         * 
         * @param os stream to print to
         * @param d detection
         * @return modified stream
         */
        std::ostream& operator<<(std::ostream& os, const Detection& d);
    }
}

#endif
