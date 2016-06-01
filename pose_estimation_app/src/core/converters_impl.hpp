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

#ifndef COVIS_CORE_CONVERTERS_IMPL_HPP
#define COVIS_CORE_CONVERTERS_IMPL_HPP

// Own
#include "macros.h"

namespace covis {
    namespace core {
        template<typename PointT>
        Eigen::MatrixX3f mapxyz(const pcl::PointCloud<PointT>& cloud) {
            Eigen::MatrixX3f m(cloud.size(), 3);
            for (std::size_t i = 0; i < cloud.size(); ++i) {
                const PointT& p = cloud[i];
                m(i, 0) = p.x;
                m(i, 1) = p.y;
                m(i, 2) = p.z;
            }

            return m;
        }

        template<typename PointT>
        pcl::PointCloud<PointT> mapxyz(const Eigen::MatrixX3f& m) {
            pcl::PointCloud<PointT> cloud(m.rows(), 1);
            for (Eigen::MatrixX3f::Index i = 0; i < m.rows(); ++i) {
                PointT& p = cloud[i];
                p.x = m(i, 0);
                p.y = m(i, 1);
                p.z = m(i, 2);
            }

            return cloud;
        }

        template<typename PointT, typename RFT>
        Eigen::Matrix4f maprf(const PointT& p, const RFT& rf) {
            Eigen::Matrix4f T;
            
            // For PCL < 1.7, pcl::ReferenceFrame was padded to 12 elements
            const size_t elems = sizeof(rf.rf) / sizeof(float);
            COVIS_ASSERT(elems == 9 || elems == 12);
            const bool padding = (elems == 12);
            
            // Rotation, column-major
            size_t idx = 0;
            T(0,0) = rf.rf[idx++];
            T(1,0) = rf.rf[idx++];
            T(2,0) = rf.rf[idx++];
            if(padding)
                idx++;
            T(0,1) = rf.rf[idx++];
            T(1,1) = rf.rf[idx++];
            T(2,1) = rf.rf[idx++];
            if(padding)
                idx++;
            T(0,2) = rf.rf[idx++];
            T(1,2) = rf.rf[idx++];
            T(2,2) = rf.rf[idx++];
            
            // Translation
            T(0,3) = p.x;
            T(1,3) = p.y;
            T(2,3) = p.z;

            // Bottom row
            T(3,0) = T(3,1) = T(3,2) = 0.0f;
            T(3,3) = 1.0f;
            
            return T;
        }

        template<typename PointT, typename RFT>
        Eigen::Matrix4f maprfi(const PointT& p, const RFT& rf) {
            Eigen::Matrix4f T;
            
            // For PCL < 1.7, pcl::ReferenceFrame was padded to 12 elements
            const size_t elems = sizeof(rf.rf) / sizeof(float);
            COVIS_ASSERT(elems == 9 || elems == 12);
            const bool padding = (elems == 12);
            
            // Inverse rotation
            size_t idx = 0;
            T(0,0) = rf.rf[idx++];
            T(0,1) = rf.rf[idx++];
            T(0,2) = rf.rf[idx++];
            if(padding)
                idx++;
            T(1,0) = rf.rf[idx++];
            T(1,1) = rf.rf[idx++];
            T(1,2) = rf.rf[idx++];
            if(padding)
                idx++;
            T(2,0) = rf.rf[idx++];
            T(2,1) = rf.rf[idx++];
            T(2,2) = rf.rf[idx++];
            
            // Get translation
            T.block<3,1>(0,3) = -T.topLeftCorner<3,3>() * Eigen::Vector3f(p.x, p.y, p.z);
            
            // Bottom row
            T(3,0) = T(3,1) = T(3,2) = 0.0f;
            T(3,3) = 1.0f;
            
            return T;
        }
    }
}

#endif
