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

#ifndef COVIS_CORE_CONVERTERS_H
#define COVIS_CORE_CONVERTERS_H

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Core>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         *
         * @brief Convert the XYZ data of a point cloud to an Eigen matrix
         *
         * @param cloud point cloud
         * @return Eigen matrix, with each point stored in rows
         * @tparam point type with members x, y and z
         */
        template<typename PointT>
        Eigen::MatrixX3f mapxyz(const pcl::PointCloud<PointT>& cloud);

        /**
         * @ingroup core
         *
         * @brief Copy the rows of an Eigen matrix into the XYZ data of a point cloud
         *
         * @param m Eigen matrix, with each point stored in rows
         * @return point cloud
         * @tparam point type with members x, y and z
         */
        template<typename PointT>
        pcl::PointCloud<PointT> mapxyz(const Eigen::MatrixX3f& m);
        
        /**
         * Convert a point and an RF to a 4-by-4 homogeneous transformation matrix
         * @param p point
         * @param rf RF
         * @return matrix
         */
        template<typename PointT, typename RFT>
        Eigen::Matrix4f maprf(const PointT& p, const RFT& rf);
        
        /**
         * Convert a point and an RF to a 4-by-4 homogeneous transformation matrix representing the inverse
         * @param p point
         * @param rf RF
         * @return matrix inverse
         */
        template<typename PointT, typename RFT>
        Eigen::Matrix4f maprfi(const PointT& p, const RFT& rf);
    }
}

#include "converters_impl.hpp"

#endif