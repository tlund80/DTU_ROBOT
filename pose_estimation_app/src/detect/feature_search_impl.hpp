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

#ifndef COVIS_DETECT_FEATURE_SEARCH_IMPL_HPP
#define COVIS_DETECT_FEATURE_SEARCH_IMPL_HPP

namespace covis {
    namespace detect {
        template<typename FeatureT>
        void FeatureSearch<FeatureT>::index() {
            // Set target data
            convert(*_target, _mtarget);
            
            // Build
#ifdef FLANN_1_8
            _tree.buildIndex(_mtarget);
#else
            _tree = new flann::Index<flann::L2<float> >(_mtarget, flann::KDTreeIndexParams(_trees));
            _tree->buildIndex();
#endif
        }
        
        template<typename FeatureT>
        void FeatureSearch<FeatureT>::doKnn(const FeatureT& query, size_t k, core::Correspondence& result) const {
            // Convert 
            flann::Matrix<float> mquery;
            convert(query, mquery);
            
            // Output
            std::vector<std::vector<idx_t> > idx;
            std::vector<std::vector<float> > distsq;
            
            // Search
#ifdef FLANN_1_8
            _tree.knnSearch(mquery, idx, distsq, k, _sparam);
#else
            _tree->knnSearch(mquery, idx, distsq, int(k), _sparam);
#endif
            
            // Convert back
            result.match.resize(k);
            result.distance.resize(k);
            for(size_t i = 0; i < k; ++i) {
                result.match[i] = idx[0][i];
                result.distance[i] = distsq[0][i];
            }
            
            // Clean up
            delete[] mquery.ptr();
        }
        
        template<typename FeatureT>
        void FeatureSearch<FeatureT>::doRadius(const FeatureT& query, float r, core::Correspondence& result) const {
            // Convert 
            flann::Matrix<float> mquery;
            convert(query, mquery);
            
            // Output
            std::vector<std::vector<idx_t> > idx;
            std::vector<std::vector<float> > distsq;
            
            // Search
#ifdef FLANN_1_8
            _tree.radiusSearch(mquery, idx, distsq, r * r, _sparam);
#else
            _tree->radiusSearch(mquery, idx, distsq, r * r, _sparam);
#endif
            
            // Convert back
            const size_t k = idx.size();
            result.match.resize(k);
            result.distance.resize(k);
            for(size_t i = 0; i < k; ++i) {
                result.match[i] = idx[0][i];
                result.distance[i] = distsq[0][i];
            }
            
            // Clean up
            delete[] mquery.ptr();
        }
    }
}

#endif
