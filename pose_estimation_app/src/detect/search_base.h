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

#ifndef COVIS_DETECT_SEARCH_BASE_H
#define COVIS_DETECT_SEARCH_BASE_H

// Own
#include "detect_base.h"
#include "../core/correspondence.h"

// Boost
#include <boost/shared_ptr.hpp>

// PCL
#include <pcl/point_cloud.h>

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class SearchBase
         * @brief Base class for point/feature cloud searches using e.g. k-d trees
         * 
         * When using the bulk search methods with complete point clouds as inputs, this class speeds up search using
         * OpenMP. Remember to call @ref setTarget() before performing any search.
         * 
         * When subclassing this class, you need to implement the protected functions @ref index(), @ref doKnn() and
         * @ref doRadius().
         * 
         * @tparam PointT point type for searches
         * @author Anders Glent Buch
         */
        template<typename PointT>
        class SearchBase : public DetectBase {
            public:
                /// Pointer type
                typedef boost::shared_ptr<SearchBase<PointT> > Ptr;
                
                /// Empty constructor
                SearchBase() {}
                
                /// Empty destructor
                virtual ~SearchBase() {}
                
                /**
                 * Set target point set and call @ref index()
                 * @param target query point set
                 */
                inline void setTarget(typename pcl::PointCloud<PointT>::ConstPtr target) {
                    _target = target;
                    COVIS_ASSERT(target);
                    index();
                }
                
                /**
                 * Get target point set
                 * @return target point set
                 */
                inline typename pcl::PointCloud<PointT>::ConstPtr getTarget() const {
                    return _target;
                }

                /**
                 * Compute k-NN feature matches
                 * @param query query point
                 * @param k number of matches
                 * @return matches as correspondences
                 */
                inline core::Correspondence knn(const PointT& query, size_t k) const {
                    // Sanity check
                    COVIS_ASSERT(k > 0);
                    
                    core::Correspondence result;
                    doKnn(query, k, result);
                    result.query = 0;
                    
                    return result;
                }
                
                /**
                 * Compute k-NN feature matches
                 * @param query query point set
                 * @param k number of matches
                 * @return matches as correspondences
                 */
                core::Correspondence::VecPtr knn(const typename pcl::PointCloud<PointT>& query, size_t k) const;
                
                /**
                 * Compute k-NN feature matches
                 * @param query query point set
                 * @param k number of matches
                 * @return matches as correspondences
                 */
                inline core::Correspondence::VecPtr knn(
                        typename pcl::PointCloud<PointT>::ConstPtr query, size_t k) const {
                    COVIS_ASSERT(query);
                    return knn(*query, k);
                }

                /**
                 * Compute radius feature matches
                 * @param query query point
                 * @param r search radius
                 * @return matches as correspondences
                 */
                inline core::Correspondence radius(const PointT& query, float r) const {
                    // Sanity check
                    COVIS_ASSERT(r > 0.0f);

                    core::Correspondence result;
                    doRadius(query, r, result);
                    result.query = 0;
                    
                    return result;
                }

                /**
                 * Compute radius feature matches
                 * @param query query point set
                 * @param r search radius
                 * @return matches as correspondences
                 */
                core::Correspondence::VecPtr radius(const typename pcl::PointCloud<PointT>& query, float r) const;

                /**
                 * Compute radius feature matches
                 * @param query query point set
                 * @param r search radius
                 * @return matches as correspondences
                 */
                inline core::Correspondence::VecPtr radius(
                        typename pcl::PointCloud<PointT>::ConstPtr query, float r) const {
                    COVIS_ASSERT(query);
                    return radius(*query, r);
                }
                
            protected:
                /// Target point set
                typename pcl::PointCloud<PointT>::ConstPtr _target;
                
                /**
                 * Build the search index using @ref _target
                 */
                virtual void index() = 0;
                
                /**
                 * Perform a k-NN search
                 * @param query query point
                 * @param k number of neighbors
                 * @param result output correspondence (query index unset)
                 */
                virtual void doKnn(const PointT& query, size_t k, core::Correspondence& result) const = 0;
                
                /**
                 * Perform a radius search
                 * @param query query point
                 * @param r search radius
                 * @param result output correspondences (with squared distances)
                 */
                virtual void doRadius(const PointT& query, float r, core::Correspondence& result) const = 0;
        };
    }
}

#include "search_base_impl.hpp"

#endif
