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

#ifndef COVIS_DETECT_POSE_SAMPLER_H
#define COVIS_DETECT_POSE_SAMPLER_H

// Own
#include "detect_base.h"
#include "../core/converters.h"
#include "../core/correspondence.h"
#include "../core/macros.h"
#include "../core/random.h"

// PCL
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class PoseSampler
         * @brief Helper class for generating pose hypotheses in @f$SE(3)@f$ using one or more point pairs,
         * given by correspondences.
         * 
         * This class provides an easy interface for computing relative poses between point sets, with or without a
         * local reference frame (RF). For point sets without RFs, >= 3 corresponding point pairs are needed to be able
         * to estimate a relative pose, while for points with an associated RF (which in itself provides an
         * object-relative element of @f$SE(3)@f$) only one point pair can be used.
         *
         * @tparam PointT point type
         * @tparam RFT reference frame type (can be unused)
         * @author Anders Glent Buch
         */
        template<typename PointT, typename RFT = pcl::ReferenceFrame>
        class PoseSampler : public DetectBase {
            public:
                /// Empty constructor
                PoseSampler() {}
                
                /**
                 * Sample n >= 3 unique correspondences
                 * @param corrs input point correspondences, size must be >= 3
                 * @param n number of point pairs to use for transformation estimation, must be >= 3
                 * @return sampled correspondences
                 */
                inline core::Correspondence::Vec sampleCorrespondences(
                        const core::Correspondence::Vec& corrs, size_t n = 3) const {
                    COVIS_ASSERT(n >= 3 && n <= corrs.size());
                    return core::extract(corrs, core::randidx(corrs.size(), n));
                }
                
                /**
                 * Sample a pose by sampling n >= 3 unique correspondences followed by a call to @ref transformation()
                 * @param corrs input point correspondences, size must be >= 3
                 * @param n number of point pairs to use for transformation estimation, must be >= 3
                 * @return sampled pose
                 */
                inline Eigen::Matrix4f sample(const core::Correspondence::Vec& corrs, size_t n = 3) const {
                    return transformation(sampleCorrespondences(corrs, n));
                }
                
                /**
                 * Generate a pose using input correspondences
                 * @param corrs input point correspondences, size must be >= 3, all assumed to be one-to-one.
                 * Thus, this function only uses the first target index inside each entry of @b corrs.
                 * @return sampled pose
                 * @exception an exception is thrown if corrs has less than 3 entries
                 */
                inline Eigen::Matrix4f transformation(const core::Correspondence::Vec& corrs) const {
                    // Extract source/target point indices
                    std::vector<int> sources(corrs.size());
                    std::vector<int> targets(corrs.size());
                    for(size_t i = 0; i < corrs.size(); ++i) {
                        sources[i] = corrs[i].query;
                        COVIS_ASSERT_DBG(!corrs[i].empty());
                        targets[i] = corrs[i].match[0];
                    }
                    
                    return transformation(sources, targets);
                }
                
                /**
                 * Generate a pose using input correspondences
                 * @param sources source or query points
                 * @param targets target points
                 * @return sampled pose
                 * @exception an exception is thrown if sources and targets are smaller than 3 or have inequal sizes
                 */
                inline Eigen::Matrix4f transformation(const std::vector<int>& sources,
                        const std::vector<int>& targets) const {
                    // Sanity checks
                    COVIS_ASSERT(_source && _target);
                    COVIS_ASSERT(sources.size() == targets.size());
                    COVIS_ASSERT_MSG(sources.size() >= 3, "Too few correspondences for transformation estimation!");
                    
                    // Run estimation
                    Eigen::Matrix4f result;
                    pcl::registration::TransformationEstimationSVD<PointT, PointT> est;
                    est.estimateRigidTransformation(*_source, sources, *_target, targets, result);
                    
                    // Estimated transformation
                    return result;
                }
                
                /// @copydoc transformation(const core::Correspondence::Vec& corrs) const
                inline Eigen::Matrix4f transformation(const core::Correspondence::VecPtr& corrs) const {
                    // Sanity checks
                    COVIS_ASSERT(corrs);
                    
                    return transformation(*corrs);
                }
                
                /**
                 * Sample a pose by sampling one correspondence followed by a call to @ref transformationRF()
                 * @param corrs input point correspondences
                 * @return sampled pose
                 */
                inline Eigen::Matrix4f sampleRF(const core::Correspondence::Vec& corrs) const {
                    COVIS_ASSERT(!corrs.empty());
                    
                    return transformationRF(corrs[core::randidx(corrs.size())]);
                }
                
                /**
                 * Generate a pose using an input correspondence defining a single point pair
                 *
                 * @note This function requires RFs to be provided
                 *
                 * @param corr input point correspondence, assumed to be one-to-one.
                 * Thus, this function only uses the first target index inside @b corr.
                 * @return sampled pose
                 * @exception an exception is thrown if @b corr has no target indices
                 */
                inline Eigen::Matrix4f transformationRF(const core::Correspondence& corr) const {
                    // Sanity checks
                    COVIS_ASSERT(!corr.empty());
                    COVIS_ASSERT(_source && _target);
                    COVIS_ASSERT(_sourcerf && _targetrf);
                    COVIS_ASSERT_DBG(_source->size() == _sourcerf->size());
                    COVIS_ASSERT_DBG(_target->size() == _targetrf->size());
                    
                    // Inverse source transformation
                    const Eigen::Matrix4f TsrcI = core::maprfi<PointT, RFT>(
                            (*_source)[corr.query], (*_sourcerf)[corr.query]);
                    
                    // Target transformation
                    const Eigen::Matrix4f Ttgt = core::maprf<PointT, RFT>(
                            (*_target)[corr.match[0]], (*_targetrf)[corr.match[0]]);
                    
                    // Result: T_target_source = T_target_feature * T_feature_source
                    return (Ttgt * TsrcI);
                }
                
                /**
                 * Set source point set
                 * @param source source point set
                 */
                inline void setSource(typename pcl::PointCloud<PointT>::ConstPtr source) {
                    _source = source;
                }
                
                /**
                 * Set source RFs
                 * @param sourcerf source RFs
                 */
                inline void setSourceRF(typename pcl::PointCloud<RFT>::ConstPtr sourcerf) {
                    _sourcerf = sourcerf;
                }
                
                /**
                 * Set target point set
                 * @param target target point set
                 */
                inline void setTarget(typename pcl::PointCloud<PointT>::ConstPtr target) {
                    _target = target;
                }
                
                /**
                 * Set target RFs
                 * @param targetrf target RFs
                 */
                inline void setTargetRF(typename pcl::PointCloud<RFT>::ConstPtr targetrf) {
                    _targetrf = targetrf;
                }
                
            private:
                /// Source descriptor set
                typename pcl::PointCloud<PointT>::ConstPtr _source;
                
                /// Source RFs
                typename pcl::PointCloud<RFT>::ConstPtr _sourcerf;
                
                /// Target descriptor set
                typename pcl::PointCloud<PointT>::ConstPtr _target;
                
                /// Target RFs
                typename pcl::PointCloud<RFT>::ConstPtr _targetrf;
        };
    }
}

#endif
