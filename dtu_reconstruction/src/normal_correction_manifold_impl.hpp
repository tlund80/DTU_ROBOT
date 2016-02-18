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

#ifndef COVIS_FEATURE_NORMAL_CORRECTION_MANIFOLD_IMPL_HPP
#define COVIS_FEATURE_NORMAL_CORRECTION_MANIFOLD_IMPL_HPP

// Own
#include "dist.h"

// Boost
#include <boost/version.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>

// STL
//#include <stack>
#include <queue>

// PCL
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>

namespace covis {
    namespace feature {
        template<typename PointNT>
        size_t NormalCorrectionManifold<PointNT>::compute(pcl::PointCloud<PointNT>& cloud) {
            // Sanity checks
            COVIS_ASSERT(!cloud.empty());
            if(_useKNN) {
                COVIS_ASSERT_MSG(_k > 0, "The value of k must be positive for k-NN search!");
            } else {
                COVIS_ASSERT_MSG(_radius > 0.0f, "Radius must be positive for radius search!");
            }
            
            // Setup a search index
            pcl::search::KdTree<PointNT> search(true);
            search.setInputCloud(cloud.makeShared());
            
            // Create Riemannian graph
            std::vector<std::vector<int> > knn(cloud.size());
            std::vector<std::vector<float> > distsq(cloud.size());
            for(size_t i = 0; i < cloud.size(); ++i) {
                if(_useKNN)
                    search.nearestKSearch(i, _k, knn[i], distsq[i]);
                else
                    search.radiusSearch(i, _radius, knn[i], distsq[i]);
            }
            
            // Bookkeeping variables
            size_t visits = 0; // Number of vertices visited
            size_t flips = 0; // Number of normals flipped
            
            // Find seed point as the point with largest positive z-component
            size_t iseed = 0;
            float zmax = cloud[0].z;
            for(size_t i = 1; i < cloud.size(); ++i) {
                const float nzi = cloud[i].z;
                if (nzi > zmax) {
                    iseed = i;
                    zmax = nzi;
                }
            }
            
            std::vector<bool> visited(cloud.size(), false);
            
            // Visit the seed
            ++visits;
            visited[iseed] = true;
            if(cloud[iseed].normal_z < 0.0f) {
                flip(cloud[iseed]);
                ++flips;
            }
            
            
            // Initialize objects for search
//            std::stack<std::pair<size_t, size_t> > q;
            std::queue<std::pair<size_t, size_t> > q; // Each element and index of <parent, child>
            
            // Insert seed as parent of itself
            q.push(std::make_pair(iseed, iseed));
            
            // While container is non-empty
            while(!q.empty()) {
                // Get a const ref to next element
                //                const std::pair<size_t, size_t>& t = q.top();
                const std::pair<size_t, size_t>& t = q.front();
                
                // Unvisited vertex and its parent
                const size_t v = t.second; // Child index
                PointNT& child = cloud[v]; // Child point
                const PointNT& parent = cloud[t.first]; // Parent point
                
                // Visitor function: flip normal if inconsistent with parent
                if(core::dot<PointNT>(parent, child) < 0.0f) {
                    flip(child);
                    ++flips;
                }
                
                // Remove top
                q.pop();
                
                // Find children
                const std::vector<int>& idx = knn[v];
                
                // Loop over edges
                for(size_t i = 0; i < idx.size(); ++i) {
                    // Get neighbor vertex (child of v)
                    const size_t u(idx[i]);
                    // If neighbor not previously visited
                    if (!visited[u]) {
                        // Mark neighbor as visited and add to queue
                        visited[u] = true;
                        q.push(std::make_pair(v, u));
                        ++visits;
                    }
                }
            } // End while container is non-empty
            
            // Issue a warning if not all points could be visited
            if(visits < cloud.size()) {
                COVIS_MSG_WARN("Not all points could be reached (" << visits << "/" << cloud.size() << ")! "
                        "Try using a larger k-value or radius.");
            }
            
            return flips;
        }
    }
}

#endif
