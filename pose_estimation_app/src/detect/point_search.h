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

#ifndef COVIS_DETECT_POINT_SEARCH_H
#define COVIS_DETECT_POINT_SEARCH_H

// Own
#include "search_base.h"
#include "../core/correspondence.h"
#include "../core/stat.h"

// PCL
#include <pcl/search/kdtree.h>

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class PointSearch
         * @brief Convenience class for kd-tree-based point search
         * 
         * This class is a simple wrapper around PCL's kd-tree search class for unordered data. For more information,
         * see @cite rusu20113d.
         * 
         * @note This class returns squared distances
         * 
         * @todo Make this class auto-select between kd-tree and organized search
         * 
         * @tparam PointT point type, must contain XYZ data
         * @author Anders Glent Buch
         */
        template<typename PointT>
        class PointSearch : public SearchBase<PointT> {
            using SearchBase<PointT>::_target;
            
            public:
                /// Pointer type
                typedef boost::shared_ptr<PointSearch<PointT> > Ptr;
                
                /// Pointer to const type
                typedef boost::shared_ptr<const PointSearch<PointT> > ConstPtr;
                
                /// K-d tree type
                typedef pcl::search::KdTree<PointT> TreeT;
                
                /// K-d tree instance
                TreeT tree;
                
                /**
                 * Constructor: setup k-d tree to produce sorted results
                 */
                PointSearch() : tree(true) {}
                
                /// Empty destructor
                virtual ~PointSearch() {}
                
                /**
                 * Compute the resolution of the indexed target point set as the mean nearest neighbor distance
                 * @param robust if set to true, use the median instead of the mean
                 * @return estimated resolution
                 */
                inline float resolution(bool robust = false) const {
                    COVIS_ASSERT_MSG(_target, "Target not set!");
                    
                    // All valid distances
                    std::vector<float> dists;
                    dists.reserve(_target->size());
                    
                    // Search results
                    std::vector<int> idx(2);
                    std::vector<float> distsq(2); // Must be sorted!
                    
                    // Start
                    for(size_t i = 0; i < _target->size(); ++i) {
                        if(pcl_isfinite((*_target)[i].x) &&
                                pcl_isfinite((*_target)[i].y) &&
                                pcl_isfinite((*_target)[i].z)) {
                            tree.nearestKSearch(int(i), 2, idx, distsq);
                            dists.push_back(std::sqrt(distsq[1]));
                        }
                    }
                    
                    return (robust ? core::median(dists) : core::mean(dists));
                }
                
            private:
                /// @copydoc SearchBase::index()
                void index();
                
                /// @copydoc SearchBase::doKnn()
                void doKnn(const PointT& query, size_t k, core::Correspondence& result) const;
                
                /// @copydoc SearchBase::doRadius()
                void doRadius(const PointT& query, float r, core::Correspondence& result) const;
        };
        
        /**
         * @ingroup detect
         * @brief Perform a single-shot bulk k-NN search in Euclidean space
         * @note This function returns squared distances
         * @param query query points
         * @param target target points to search into
         * @param k number of neighbors to search for
         * @return matches as correspondences
         */
        template<typename PointT>
        inline core::Correspondence::VecPtr knnSearch(typename pcl::PointCloud<PointT>::ConstPtr query,
                typename pcl::PointCloud<PointT>::ConstPtr target,
                size_t k) {
            PointSearch<PointT> pm;
            pm.setTarget(target);
            
            return pm.knn(query, k);
        }
        
        /**
         * @ingroup detect
         * @brief Perform a single-shot bulk radius search in Euclidean space
         * @note This function returns squared distances
         * @param query query points
         * @param target target points to search into
         * @param r Euclidean search radius
         * @return matches as correspondences
         */
        template<typename PointT>
        inline core::Correspondence::VecPtr radiusSearch(typename pcl::PointCloud<PointT>::ConstPtr query,
                typename pcl::PointCloud<PointT>::ConstPtr target,
                float r) {
            PointSearch<PointT> pm;
            pm.setTarget(target);
            
            return pm.radius(query, r);
        }
        
        /**
         * @ingroup detect
         * @brief Estimate the resolution of a point cloud
         * @param cloud point cloud for which to compute resolution
         * @param robust if true, use a robust measure
         * @return resolution
         */
        template<typename PointT>
        inline float computeResolution(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                bool robust = false) {
            PointSearch<PointT> pm;
            pm.setTarget(cloud);
            
            return pm.resolution(robust);
        }
    }
}

#include "point_search_impl.hpp"
    
#endif
