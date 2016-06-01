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

#ifndef COVIS_DETECT_CORRESPONDENCE_VOTING_H
#define COVIS_DETECT_CORRESPONDENCE_VOTING_H

// Own
#include "correspondence_filter_base.h"
#include "../core/dist.h"
#include "../core/macros.h"
#include "../core/transform.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class CorrespondenceVoting
         * @brief Correspondence filtering based on a voting procedure
         * 
         * For more information, see the following publication:
         * @cite buch2014search
         * 
         * @todo Rename this class to CorrespondenceFilterVoting
         * 
         * @tparam PointT point type, must contain XYZ data
         * @tparam RFT reference frame type
         * @author Anders Glent Buch
         * 
         * @example example/correspondence_voting/correspondence_voting.cpp
         */
        template<typename PointT, typename RFT = pcl::ReferenceFrame>
        class CorrespondenceVoting : public CorrespondenceFilterBase {
            public:
                /**
                 * Default constructor: sets up default parameters
                 */
                CorrespondenceVoting() :
                    _samples(250),
                    _similarity(0.9),
                    _penaltyThreshold(-1),
                    _confidences(),
                    _rfradius(0.025),
                    _maxdistsq(0.0f) {}
                
                /// Empty destructor
                virtual ~CorrespondenceVoting() {}
                
                /// @copydoc CorrespondenceFilterBase::filter()
                core::Correspondence::VecPtr filter(const core::Correspondence::Vec& corr);
                
                /**
                 * Get confidences of all the input correspondences
                 * @return correspondence confidences
                 */
                inline const std::vector<float>& getConfidences() const {
                    return _confidences;
                }
                
                /**
                 * Set number of samples
                 * @param samples number of samples
                 */
                inline void setSamples(size_t samples) {
                    _samples = samples;
                }
                
                /**
                 * Set edge length similarity
                 * @param similarity edge length similarity
                 */
                inline void setSimilarity(float similarity) {
                    _similarity = similarity;
                }
                
                /**
                 * Set penalty threshold for casting votes using the input correspondences
                 * @param penaltyThreshold penalty threshold, must be positive
                 */
                inline void setPenaltyThreshold(float penaltyThreshold) {
                    COVIS_ASSERT(penaltyThreshold > 0.0f);
                    _penaltyThreshold = penaltyThreshold;
                }
                
                /**
                 * Set query point set
                 * @param query query point set
                 */
                inline void setQuery(typename pcl::PointCloud<PointT>::ConstPtr query) {
                    _query = query;
                    PointT min, max;
                    pcl::getMinMax3D<PointT>(*_query, min, max);
                    _maxdistsq = core::distsq<PointT, float>(min, max);
                }
                
                /**
                 * Set query RFs
                 * @param queryrf query RFs
                 */
                inline void setQueryRF(typename pcl::PointCloud<RFT>::ConstPtr queryrf) {
                    _queryrf = queryrf;
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
                
                /**
                 * Set RF radius
                 * @param rfradius RF radius
                 */
                inline void setRFRadius(float rfradius) {
                    _rfradius = rfradius;
                }
                
                /**
                 * Set precomputed local neighbors, one for each query feature point
                 * @param queryNeighbors precomputed local neighbors
                 */
                inline void setQueryNeighbors(const core::Correspondence::VecPtr queryNeighbors) {
                    _queryNeighbors = queryNeighbors;
                }
                
                /**
                 * Get output transformations, one per filtered correspondence - only valid after a call to filter()
                 * @return output transformations
                 */
                inline const std::vector<Eigen::Matrix4f>& getTransformations() const {
                    return _transformations;
                }
                
            private:
                /// Number of votes
                size_t _samples;
                
                /// Relative distance similarity in [0,1]
                float _similarity;
                
                /**
                 * Upper threshold for casting a vote using the input correspondences
                 * If unset, and the input correspondences have 2 neighbors we use 0.8 for the ratio ranking,
                 * otherwise we use the median of the distances 
                 */
                float _penaltyThreshold;
                
                /// Calculated confidences, one to one with the input correspondences
                std::vector<float> _confidences;
                
                /// Query point cloud
                typename pcl::PointCloud<PointT>::ConstPtr _query;
                
                /// Query RFs
                typename pcl::PointCloud<RFT>::ConstPtr _queryrf;
                
                /// Target point cloud
                typename pcl::PointCloud<PointT>::ConstPtr _target;
                
                /// TargetRFs
                typename pcl::PointCloud<RFT>::ConstPtr _targetrf;
                
                /// RF radius: in case RFs are not input, they are estimated using this radius
                float _rfradius;
                
                /// Query neighbors
                core::Correspondence::VecPtr _queryNeighbors;
                
                /// Computed maximum distance between any two points on the source model
                float _maxdistsq;
                
                /// Output transformations of the filtered correspondences
                std::vector<Eigen::Matrix4f> _transformations;
                
                /**
                 * Perform simple local distance check using a correspondence pair
                 * @param corr1 first correspondence
                 * @param corr2 second correspondence
                 * @param simsq lower distance similarity threshold, squared
                 * @return true if source/target point distances are similar
                 */
                inline bool local(const core::Correspondence& corr1,
                        const core::Correspondence& corr2,
                        float simsq) const {
                    // Target point pair edge length
                    const float disttgt = core::distsq<PointT, float>((*_target)[corr1.match[0]], (*_target)[corr2.match[0]]);

                    if (disttgt > _maxdistsq / simsq)
                       return false;

                    // Source point pair edge length
                    const float distsrc = core::distsq<PointT, float>((*_query)[corr1.query], (*_query)[corr2.query]);

                    // Edge length similarity [0,1] where 1 is a perfect match
                    const float esim = ( distsrc < disttgt ? distsrc / disttgt : disttgt / distsrc );

                    return (esim >= simsq);
                }
                
                /**
                 * Perform global compatibility check of a correspondence using a transformation computed using another
                 * correspondence
                 * @param T transformation computed using first correspondence
                 * @param corr second correspondence
                 * @param thressq inlier threshold, squared
                 * @return true if transformed correspondence is spatially close
                 */
                inline bool global(const Eigen::Matrix4f& T, const core::Correspondence& corr, float thressq) const {
                    PointT p = (*_query)[corr.query];
                    core::transform<PointT>(p, T);
                    
                    return (core::distsq<PointT, float>(p, (*_target)[corr.match[0]]) < thressq);
                }
                
                /**
                 * Find optimal cut for the input vector using Otsu's method
                 * @param v data vector
                 * @param lower lower limit
                 * @param upper upper limit
                 * @return optimal cut in [lower, upper]
                 * @todo Find a good way to automatically compute the number of bins
                 */
                float thresholdOtsu(const std::vector<float>& v, float lower, float upper);
        };
    }
}

#include "correspondence_voting_impl.hpp"

#endif
