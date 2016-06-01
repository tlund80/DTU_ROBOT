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

#ifndef COVIS_DETECT_FIT_EVALUATION_IMPL_HPP
#define COVIS_DETECT_FIT_EVALUATION_IMPL_HPP

// Own
#include "../core/traits.h"
#include "../core/transform.h"

// STL
#include <cfloat>

namespace covis {
    namespace detect {
        template<typename PointT>
        core::Correspondence::VecPtr FitEvaluation<PointT>::update(typename pcl::PointCloud<PointT>::ConstPtr source,
                const MatrixT& pose) {
            // Sanity checks
            COVIS_ASSERT(source && !source->empty());
            
            // Apply transformation
            typename pcl::PointCloud<PointT>::Ptr sourcet(new pcl::PointCloud<PointT>);
            core::transform<PointT>(*source, *sourcet, pose);
            
            return update(sourcet);
        }
        
        template<typename PointT>
        core::Correspondence::VecPtr FitEvaluation<PointT>::update(typename pcl::PointCloud<PointT>::ConstPtr source) {
            // Sanity checks
            COVIS_ASSERT(source && !source->empty());
            COVIS_ASSERT(_search && _search->getTarget() == _target);
            
            // Find point matches
            core::Correspondence::VecPtr correspondences = _search->knn(source, 1);
            
            update(source, correspondences);
            
            return correspondences;
        }
        
        template<typename PointT>
        void FitEvaluation<PointT>::update(typename pcl::PointCloud<PointT>::ConstPtr source,
                const MatrixT& pose,
                core::Correspondence::VecConstPtr correspondences) {
            // Sanity checks
            COVIS_ASSERT(source && !source->empty());
            
            // Apply transformation to the points indexed in the correspondence set and store point distances
            core::Correspondence::VecPtr corrWithDistances(new core::Correspondence::Vec(correspondences->size()));
            typename pcl::PointCloud<PointT>::Ptr sourcet(new pcl::PointCloud<PointT>(source->size(), 1));
            for(size_t i = 0; i < correspondences->size(); ++i) {
                COVIS_ASSERT_DBG(!(*correspondences)[i].empty());
                // Copy
                sourcet->points[(*correspondences)[i].query] = source->points[(*correspondences)[i].query];
                // Transform
                core::transform<PointT>(sourcet->points[(*correspondences)[i].query], pose);
                // Store distance, squared
                (*corrWithDistances)[i] = (*correspondences)[i];
                (*corrWithDistances)[i].distance[0] = (sourcet->points[(*correspondences)[i].query].getVector3fMap() - 
                        _target->points[(*correspondences)[i].match[0]].getVector3fMap()).squaredNorm();
            }
            
            update(sourcet, corrWithDistances);
        }
            
        template<typename PointT>
        void FitEvaluation<PointT>::update(typename pcl::PointCloud<PointT>::ConstPtr source,
                core::Correspondence::VecConstPtr correspondences) {
            // Sanity checks
            COVIS_ASSERT(source && !source->empty());
            COVIS_ASSERT_MSG(_inlierThresholdSquared > 0.0f, "Inlier threshold not set!");
            
            // Accumulate number of inliers and MSE
            _state.size = correspondences->size();
            _state.inliers.clear();
            _state.inliers.reserve(correspondences->size());
            _state.mse = 0.0;
            _state.occluded.clear();
            _state.occluded.reserve(correspondences->size());
            for(size_t i = 0; i < correspondences->size(); ++i) {
                COVIS_ASSERT_DBG(!(*correspondences)[i].empty());
                // Get source point
                const int isrc = (*correspondences)[i].query;
                if(_occlusionRemoval && core::HasNormal<PointT>::value) { // Occlusion removal enabled, normals present
                    const float nz = core::GetNormal<PointT>::nz(source->points[isrc], -source->points[isrc].z);
                    if(source->points[isrc].z * nz >= 0.0f) {
                        _state.occluded.push_back(isrc);
                        continue;
                    }
                }
                
                const float distsq = (*correspondences)[i].distance[0];
                if(distsq < _inlierThresholdSquared) {
                    _state.inliers.push_back((*correspondences)[i]);
                    _state.mse += distsq;
                }
            }
            
            // Set to infinite if there were no inliers
            if(_state.inliers.empty())
                _state.mse = DBL_MAX;
            else
                _state.mse /= double(_state.inliers.size());
        }
    }
}
#endif
