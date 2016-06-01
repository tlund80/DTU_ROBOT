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

#ifndef COVIS_CORE_DIST_H
#define COVIS_CORE_DIST_H

// Own
#include "macros.h"

// STL
#include <cmath>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @brief Squared Euclidean distance
         * @param p1 first point
         * @param p2 second point
         * @return squared Euclidean distance
         */
        template<typename PointT, typename RealT>
        inline RealT distsq(const PointT& p1, const PointT& p2) {
            const RealT dx = p2.x - p1.x;
            const RealT dy = p2.y - p1.y;
            const RealT dz = p2.z - p1.z;
            return dx * dx + dy * dy + dz * dz;
        }
        
        /**
         * @ingroup core
         * @brief Euclidean distance
         * @param p1 first point
         * @param p2 second point
         * @return Euclidean distance
         */
        template<typename PointT, typename RealT>
        inline RealT dist(const PointT& p1, const PointT& p2) {
            return std::sqrt(distsq<PointT, RealT>(p1, p2));
        }
        
        /**
         * @ingroup core
         * @brief Dot product of two N-d vectors
         * @param v1 first vector
         * @param v2 second vector
         * @return dot product
         */
        template<typename RealT, int N>
        inline RealT dot(const RealT* v1, const RealT* v2) {
            RealT sum(0);
            for(int i = 0; i < N; ++i)
                sum += v1[i] * v2[i];
            return sum;
        }
        
        /**
         * @ingroup core
         * @brief Dot product of two normals
         * @param n1 first normal
         * @param n2 second normal
         * @return dot product
         */
        template<typename NormalT>
        inline float dot(const NormalT& n1, const NormalT& n2) {
            return dot<float, 3>(n1.data_n, n2.data_n);
        }
    }
}
#endif