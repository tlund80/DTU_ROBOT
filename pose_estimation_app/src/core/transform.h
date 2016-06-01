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

#ifndef COVIS_CORE_TRANSFORM_H
#define COVIS_CORE_TRANSFORM_H

// Own
#include "macros.h"
#include "traits.h"

// Eigen
#include <Eigen/Core>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @brief Translate a point
         * @param p point
         * @param x x-axis translation
         * @param y y-axis translation
         * @param z z-axis translation
         */
        template<typename PointT>
        inline void translate(PointT& p, float x, float y, float z) {
            p.x += x;
            p.y += y;
            p.z += z;
        }
        
        /// @cond
        template<typename PointT, bool HasNormal>
        struct Rotator {
            static void rotate(PointT& p, const Eigen::Matrix3f& R);
        };
        
        template<typename PointT>
        struct Rotator<PointT,false> {
            static inline void rotate(PointT& p, const Eigen::Matrix3f& R) {
                const float x = p.x;
                const float y = p.y;
                const float z = p.z;
                p.x = R(0,0) * x + R(0,1) * y + R(0,2) * z;
                p.y = R(1,0) * x + R(1,1) * y + R(1,2) * z;
                p.z = R(2,0) * x + R(2,1) * y + R(2,2) * z;
            }
        };
        
        template<typename PointT>
        struct Rotator<PointT,true> {
            static inline void rotate(PointT& p, const Eigen::Matrix3f& R) {
                Rotator<PointT, false>::rotate(p, R);

                const float nx = p.normal_x;
                const float ny = p.normal_y;
                const float nz = p.normal_z;
                p.normal_x = R(0,0) * nx + R(0,1) * ny + R(0,2) * nz;
                p.normal_y = R(1,0) * nx + R(1,1) * ny + R(1,2) * nz;
                p.normal_z = R(2,0) * nx + R(2,1) * ny + R(2,2) * nz;
            }
        };
        /// @endcond

        /**
         * @ingroup core
         * @brief Rotate a point, and the normal if present
         * @param p point
         * @param R rotation matrix
         * @tparam PointT point type (possibly with normal)
         */
        template<typename PointT>
        inline void rotate(PointT& p, const Eigen::Matrix3f& R) {
            Rotator<PointT, HasNormal<PointT>::value>::rotate(p, R);
        }

        /**
         * @ingroup core
         * @brief Transform a point, and rotate the normal if present
         * @param p point
         * @param T transformation matrix
         * @tparam PointT point type (possibly with normal)
         */
        template<typename PointT>
        inline void transform(PointT& p, const Eigen::Matrix4f& T) {
            rotate<PointT>(p, T.block<3,3>(0,0));
            translate<PointT>(p, T(0,3), T(1,3), T(2,3));
        }

        /**
         * @ingroup core
         * @brief Rotate a point cloud, and the normals if present
         * @param input input/ouptut point cloud
         * @param R rotation matrix
         * @tparam PointT point type (possibly with normal)
         */
        template<typename PointT>
        inline void rotate(typename pcl::PointCloud<PointT>& input, const Eigen::Matrix3f& R) {
            for(size_t i = 0; i < input.size(); ++i)
                rotate<PointT>(input[i], R);
        }

        /**
         * @ingroup core
         * @brief Rotate a point cloud, and the normals if present
         * @param input input/ouptut point cloud
         * @param T transformation matrix
         * @tparam PointT point type (possibly with normal)
         */
        template<typename PointT>
        inline void rotate(typename pcl::PointCloud<PointT>& input, const Eigen::Matrix4f& T) {
            rotate<PointT>(input, T.block<3,3>(0,0));
        }

        /**
         * @ingroup core
         * @brief Transform a point cloud, and rotate the normals if present
         * @param input input/output point cloud
         * @param T transformation matrix
         * @tparam PointT point type (possibly with normal)
         */
        template<typename PointT>
        inline void transform(typename pcl::PointCloud<PointT>& input, const Eigen::Matrix4f& T) {
            const Eigen::Matrix3f& R = T.block<3,3>(0,0);
            const float tx = T(0,3);
            const float ty = T(1,3);
            const float tz = T(2,3);
            for(size_t i = 0; i < input.size(); ++i) {
                rotate<PointT>(input[i], R);
                translate<PointT>(input[i], tx, ty, tz);
            }
        }

        /**
         * @ingroup core
         * @brief Rotate a point cloud, and the normals if present
         * @note This function is most efficient if input == output
         * @param input input point cloud
         * @param output result
         * @param T transformation matrix
         * @tparam PointT point type (possibly with normal)
         */
        template<typename PointT>
        inline void rotate(const typename pcl::PointCloud<PointT>& input,
                typename pcl::PointCloud<PointT>& output,
                const Eigen::Matrix4f& T) {
            if(&output != &input) // Copy
                output = input;
            rotate<PointT>(output, T);
        }

        /**
         * @ingroup core
         * @brief Transform a point cloud, and rotate the normals if present
         * @note This function is most efficient if input == output
         * @param input input point cloud
         * @param output result
         * @param T transformation matrix
         * @tparam PointT point type (possibly with normal)
         */
        template<typename PointT>
        inline void transform(const typename pcl::PointCloud<PointT>& input,
                typename pcl::PointCloud<PointT>& output,
                const Eigen::Matrix4f& T) {
            if(&output != &input) // Copy
                output = input;
            transform<PointT>(output, T);
        }
    }
}

#endif
