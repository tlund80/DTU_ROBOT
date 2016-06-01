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

#ifndef COVIS_DETECT_POINT_SEARCH_IMPL_HPP
#define COVIS_DETECT_POINT_SEARCH_IMPL_HPP

namespace covis {
    namespace detect {
        template<typename PointT>
        void PointSearch<PointT>::index() {
            tree.setInputCloud(_target);
        }
        
        template<typename PointT>
        void PointSearch<PointT>::doKnn(const PointT& query, size_t k, core::Correspondence& result) const {
            std::vector<int> idx(k);
            std::vector<float> distsq(k);
            tree.nearestKSearch(query, int(k), idx, distsq);
            
            // Convert back
            result.match.resize(k);
            result.distance.resize(k);
            for(size_t i = 0; i < k; ++i) {
                result.match[i] = idx[i];
                result.distance[i] = distsq[i];
            }
        }
        
        template<typename PointT>
        void PointSearch<PointT>::doRadius(const PointT& query, float r, core::Correspondence& result) const {
            std::vector<int> idx;
            std::vector<float> distsq;
            tree.radiusSearch(query, double(r), idx, distsq);
            
            // Convert back
            const size_t k = idx.size();
            result.match.resize(k);
            result.distance.resize(k);
            for(size_t i = 0; i < k; ++i) {
                result.match[i] = idx[i];
                result.distance[i] = distsq[i];
            }
        }
    }
}

#endif
