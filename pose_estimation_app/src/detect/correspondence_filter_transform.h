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

#ifndef COVIS_DETECT_CORRESPONDENCE_FILTER_TRANFORM_H
#define COVIS_DETECT_CORRESPONDENCE_FILTER_TRANFORM_H

// Own
#include "correspondence_filter_base.h"

// PCL
#include <pcl/point_cloud.h>

// Eigen
#include <Eigen/Core>

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class CorrespondenceFilterTransform
         * @brief Correspondence filter class for benchmarking correspondences under a transformation
         * 
         * This class takes as input a query and a target point cloud, a Euclidean inlier threshold and a
         * transformation. Then, for each of the query-target correspondences, the transformation is applied, and the
         * class checks whether the point pair in the correspondence are spatially close. This is a useful functionality
         * for validating how many correspondences generated by some matching process are correct. It does require the
         * availability of ground truth pose information.
         * 
         * If the Euclidean inlier threshold is set to <= 0, the threshold is automatically set to twice the resolution
         * of the point cloud, estimated using detect::computeResolution().
         * 
         * \note If you input one-to-many correspondences, only the first match is considered!
         * 
         * @tparam PointT point type, must contain XYZ data
         * @author Anders Glent Buch
         */
        template<typename PointT>
        class CorrespondenceFilterTransform : public CorrespondenceFilterBase {
            public:
                /**
                 * Default constructor: sets transformation to zero and inlier threshold to automatic
                 */
                CorrespondenceFilterTransform() :
                    _transformation(Eigen::Matrix4f::Zero()),
                    _thres(-1) {}
                
                /// Empty destructor
                virtual ~CorrespondenceFilterTransform() {}
                
                /// @copydoc CorrespondenceFilterBase::filter()
                core::Correspondence::VecPtr filter(const core::Correspondence::Vec& corr);
                
                /**
                 * Set query point set
                 * @param query query point set
                 */
                inline void setQuery(typename pcl::PointCloud<PointT>::ConstPtr query) {
                    _query = query;
                }
                
                /**
                 * Set target point set
                 * @param target target point set
                 */
                inline void setTarget(typename pcl::PointCloud<PointT>::ConstPtr target) {
                    _target = target;
                }
                
                /**
                 * Set transformation
                 * @param transformation transformation
                 */
                inline void setTransformation(const Eigen::Matrix4f& transformation) {
                    _transformation = transformation;
                }
                
                /**
                 * Get transformation
                 * @return transformation
                 */
                inline const Eigen::Matrix4f& getTransformation() const {
                    return _transformation;
                }
                
                /**
                 * Set Euclidean inlier threshold (set to <= 0 for automatic)
                 * @param thres Euclidean inlier threshold
                 */
                inline void setThreshold(float thres) {
                    _thres = thres;
                }
                
                /**
                 * Get Euclidean inlier threshold
                 * @return Euclidean inlier threshold
                 */
                inline float getThreshold() const {
                    return _thres;
                }
                
                /**
                 * Get mask, only valid after a call to @ref filter()!
                 * @return mask
                 */
                inline const std::vector<bool>& getMask() const {
                    return _mask;
                }
                
            private:
                /// Source point cloud
                typename pcl::PointCloud<PointT>::ConstPtr _query;
                
                /// Target point cloud
                typename pcl::PointCloud<PointT>::ConstPtr _target;
                
                /// Transformation
                Eigen::Matrix4f _transformation;
                
                /// Euclidean inlier threshold
                float _thres;
                
                /// Mask: this will contain a true for each accepted correspondence in the input
                std::vector<bool> _mask;
        };
        
        /**
         * @ingroup filter
         * @brief Filter correspondences using a ground truth alignment pose
         * @param corr input correspondences
         * @param query source point cloud
         * @param target target point cloud
         * @param transformation transformation
         * @param thres Euclidean inlier threshold
         * @tparam PointT point type, must contain XYZ data
         * @return filtered correspondences
         */
        template<typename PointT>
        inline core::Correspondence::VecPtr filterCorrespondencesTransform(const core::Correspondence::Vec& corr,
                typename pcl::PointCloud<PointT>::ConstPtr query,
                typename pcl::PointCloud<PointT>::ConstPtr target,
                const Eigen::Matrix4f& transformation,
                float thres) {
            CorrespondenceFilterTransform<PointT> cft;
            cft.setQuery(query);
            cft.setTarget(target);
            cft.setTransformation(transformation);
            cft.setThreshold(thres);
            
            return cft.filter(corr);
        }
    }
}

#include "correspondence_filter_transform_impl.hpp"

#endif
