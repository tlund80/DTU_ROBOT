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

#ifndef COVIS_CORE_IDS_H
#define COVIS_CORE_IDS_H

// OpenCV
#include <opencv2/core/core.hpp>

// PCL
#include <pcl/register_point_struct.h>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @struct Ids
         * @brief Structure for representing intrinsic dimensions
         * This struct is also registered as a PCL point type
         */
        struct Ids {
                /// Default constructor: set all values to zero
                Ids() : id0(0), id1(0), id2(0), scale(0) {}
                
                /**
                 * Parameter constructor: set all values
                 * @param id0 ID 0
                 * @param id1 ID 1
                 * @param id2 ID 2
                 * @param scale scale
                 */
                Ids(float id0, float id1, float id2, float scale = 0.0) {
                    this->id0 = id0;
                    this->id1 = id1;
                    this->id2 = id2;
                    this->scale = scale;
                }
                
                /**
                 * Parameter constructor: set all IDs and scale to zero
                 * @param ids OpenCV vector of IDs
                 * @sa @ref covis::filter::IntrinsicDimension
                 */
                Ids(const cv::Vec3d& ids) {
                    id0 = ids[0];
                    id1 = ids[1];
                    id2 = ids[2];
                    scale = 0.0;
                }
                
                union {
                        struct {
                                /// ID 0
                                float id0;
                                
                                /// ID 1
                                float id1;
                                
                                /// ID 2
                                float id2;
                        };
                        
                        /// ID vector
                        float ids[3];
                };
                
                union {
                        /// Optional scale parameter for ID structures, @sa @ref covis::filter::IntrinsicDimension3D
                        float scale;
                        
                        /// @copydoc scale
                        float size;
                };
        };
    }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(covis::core::Ids,
        (float, id0, id0)
        (float, id1, id1)
        (float, id2, id2)
        (float, scale, scale))


#endif
