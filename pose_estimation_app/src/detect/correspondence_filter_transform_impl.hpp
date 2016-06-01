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

#ifndef COVIS_DETECT_CORRESPONDENCE_FILTER_TRANSFORM_IMPL_HPP
#define COVIS_DETECT_CORRESPONDENCE_FILTER_TRANSFORM_IMPL_HPP

// Own
#include "../core/dist.h"
#include "../core/transform.h"
#include "../detect/point_search.h"

namespace covis {
    namespace detect {
        template<typename PointT>
        core::Correspondence::VecPtr CorrespondenceFilterTransform<PointT>::filter(
                const core::Correspondence::Vec& corr) {
            // Sanity checks
            COVIS_ASSERT_MSG(_query, "Query point cloud not set!");
            COVIS_ASSERT_MSG(_target, "Target point cloud not set!");
            
            // Compute a good threshold, if required
            if(_thres <= 0.0)
                _thres = 2.0f * detect::computeResolution<PointT>(_query);
            
            COVIS_ASSERT_MSG(_thres > 0.0f, "Euclidean inlier threshold must be strictly positive!");
            
            // Check transformation
            Eigen::Matrix4f tcopy = _transformation;
            if(tcopy.isZero()) {
                COVIS_MSG_WARN("Transformation not set - assuming identity transformation!");
                tcopy = Eigen::Matrix4f::Identity();
            }
            
            // Set to true if identity transformation is in use
            const bool iseye = tcopy.isIdentity();
            
            // Allocate result
            core::Correspondence::VecPtr result(new core::Correspondence::Vec);
            result->reserve(corr.size());
            
            // Use squared distance for efficiency
            const float thressq = _thres * _thres;
            
            // Start
            _mask.resize(corr.size());
            for(size_t i = 0; i < corr.size(); ++i) {
                if(corr[i].size() > 0) {
                    COVIS_ASSERT_DBG_MSG(size_t(corr[i].query) < _query->size() &&
                            size_t(corr[i].match[0]) < _target->size(),
                            "Correspondence with index " << i << " invalid! Query/match index: " <<
                            corr[i].query << "/" << corr[i].match[0]);
                    // Get a copy of the query point and apply transformation
                    PointT pquery = (*_query)[corr[i].query];
                    if(!iseye)
                        core::transform(pquery, tcopy);
                    
                    // Evaluate Euclidean distance, and add to result if passed
                    _mask[i] = (core::distsq<PointT,float>(pquery, (*_target)[corr[i].match[0]]) <= thressq);
                    if(_mask[i])
                        result->push_back(core::Correspondence(corr[i].query, corr[i].match[0], corr[i].distance[0]));
                }
            }
            
            return result;
        }
    }
}

#endif
