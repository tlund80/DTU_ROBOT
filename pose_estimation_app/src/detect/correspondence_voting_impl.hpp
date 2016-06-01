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

#ifndef COVIS_DETECT_CORRESPONDENCE_VOTING_IMPL_HPP
#define COVIS_DETECT_CORRESPONDENCE_VOTING_IMPL_HPP

// Own
#include "pose_sampler.h"
#include "../core/random.h"

// Boost
#include <boost/smart_ptr/make_shared.hpp>

// PCL
#include <pcl/features/shot_lrf_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>

namespace covis {
    namespace detect {
        template<typename PointT, typename RFT>
        core::Correspondence::VecPtr CorrespondenceVoting<PointT, RFT>::filter(const core::Correspondence::Vec& corr) {
            // Clean up
            _confidences.clear();
            
            // Sanity checks
            COVIS_ASSERT(!corr.empty() && corr.size() <= _query->size());
            COVIS_ASSERT(_query && _target);
            COVIS_ASSERT(_maxdistsq > 0.0); // Relates to setQuery()
            COVIS_ASSERT(_samples > 0 && _samples < _query->size());
            COVIS_ASSERT(_similarity >= 0.0f && _similarity <= 1.0f);
            
            // Special case
            if (corr.size() <= 2)
                return boost::make_shared<core::Correspondence::Vec>(corr);

            // Mapping source feature --> correspondence
            std::vector<int> mapFeatCorr(_query->size(), -1);
            for(size_t i = 0; i < corr.size(); ++i) {
                COVIS_ASSERT_MSG(mapFeatCorr[corr[i].query] == -1,
                        "Duplicate correspondence for query point with index " << corr[i].query << "!");
                mapFeatCorr[corr[i].query] = i;
            }
            
            // Check correspondences and warn if 2-NNs for Lowe thresholding are unavailable
            bool ratio = true;
            for(size_t i = 0; i < corr.size(); ++i) {
//               COVIS_ASSERT_MSG(corr[i].size() >= 1, "Correspondences must have size >= 1 for voting-based rejection!");
               if (ratio && corr[i].size() < 2) {
                  ratio = false;
                  COVIS_MSG_WARN("2-NN correspondences not available for initial ratio ranking!");
                  break;
               }
            }

            // Create source search - it is needed either if source RFs or query neighbors are missing
            typename pcl::search::Search<PointT>::Ptr searchsrc;
            if(_query->isOrganized())
                searchsrc.reset(new pcl::search::OrganizedNeighbor<PointT>);
            else
                searchsrc.reset(new pcl::search::KdTree<PointT>);
            searchsrc->setInputCloud(_query);
            
            // If RFs are missing, estimate them
            if(!_queryrf) {
                COVIS_ASSERT(_rfradius > 0.0f);
                COVIS_MSG_WARN("Source RFs missing! Estimating...");
                pcl::SHOTLocalReferenceFrameEstimationOMP<PointT, RFT> lrf;
                lrf.setInputCloud(_query);
                lrf.setRadiusSearch(double(_rfradius));
                lrf.setSearchMethod(searchsrc);
                typename pcl::PointCloud<RFT>::Ptr sourcerf(new pcl::PointCloud<RFT>);
                lrf.compute(*sourcerf);
                _queryrf = sourcerf;
            }
            
            if(!_targetrf) {
                COVIS_ASSERT(_rfradius > 0.0f);
                COVIS_MSG_WARN("Target RFs missing! Estimating...");
                pcl::SHOTLocalReferenceFrameEstimationOMP<PointT, RFT> lrf;
                lrf.setInputCloud(_target);
                lrf.setRadiusSearch(double(_rfradius));
                typename pcl::PointCloud<RFT>::Ptr targetrf(new pcl::PointCloud<RFT>);
                lrf.compute(*targetrf);
                _targetrf = targetrf;
            }
            
            // If query neighbors are missing, estimate them
            if(!_queryNeighbors || _queryNeighbors->size() != corr.size()) {
                COVIS_MSG_WARN("Query neighbors not set, or invalid! Estimating...");
                if(!_queryNeighbors)
                    _queryNeighbors.reset(new core::Correspondence::Vec(corr.size()));
                else
                    _queryNeighbors->resize(corr.size());
#ifdef _OPENMP
#pragma omp parallel for
#endif
                for(size_t i = 0; i < corr.size(); ++i)
                    searchsrc->nearestKSearch(_query->points[corr[i].query], int(_samples + 1),
                            (*_queryNeighbors)[i].match, (*_queryNeighbors)[i].distance);
                
            }
            
            // Check RFs
            COVIS_ASSERT(_queryrf->size() == _query->size());
            COVIS_ASSERT(_targetrf->size() == _target->size());
            
            // Compute squared inlier threshold for global voting (5 x resolution)
            float thressq = 5.0f * detect::computeResolution<PointT>(_target);
            thressq *= thressq; // Squared
            
            // Instantiate pose sampler
            PoseSampler<PointT, RFT> ps;
            ps.setSource(_query);
            ps.setSourceRF(_queryrf);
            ps.setTarget(_target);
            ps.setTargetRF(_targetrf);
            _transformations.resize(corr.size());
            
            // Compute Lowe's ratio score, squared, if possible, otherwise feature distance
            std::vector<float> penalties(corr.size());
            if(ratio) {
                for(size_t i = 0; i < corr.size(); ++i)
                    penalties[corr[i].query] = corr[i].distance[0] / corr[i].distance[1];
            } else {
                for(size_t i = 0; i < corr.size(); ++i)
                    penalties[corr[i].query] = corr[i].distance[0];
            }
            
            // Prepare variables
            const float penaltyThres = ( _penaltyThreshold > 0.0 ?
                    _penaltyThreshold :
                    (ratio ? 0.8f * 0.8f : core::median(penalties)) );
            const float simsq = _similarity * _similarity; // For efficiency in dcheck()
            std::vector<size_t> numSamples(corr.size(), 0);
            std::vector<size_t> numAccepted(corr.size(), 0);
            _confidences.resize(corr.size());
            
            /*
             * First voting loop
             */
            // Loop over input correspondences
//#ifdef _OPENMP
//#pragma omp parallel for
//#endif
            for(size_t i = 0; i < corr.size(); ++i) {
                // First correspondence
                const core::Correspondence& c1 = corr[i];
                // First loop, get neighbors
                const core::Correspondence& cn = (*_queryNeighbors)[i];
                // Loop over neighbors
                for(size_t j = 0; j < cn.size(); ++j) {
                    // Index of neighbor feature point
                    const int& nidx = cn.match[j];
                    // Index of correspondence
                    const int& cidx = mapFeatCorr[nidx];
                    // If a correspondence exist for this feature point
                    if(cidx != -1) {
                        // Neighbor correspondence
                        const core::Correspondence& c2 = corr[cidx];
                        // Avoid self
                        if(&c2 != &c1) {
                            // If neighbor correspondence has a low penalty
//                            if(penalties[c2.query] < penaltyThres) {
                            if(penalties[cidx] < penaltyThres) {
                                ++numSamples[i];
                                if(local(c1, c2, simsq))
                                    ++numAccepted[i];
                            }
                        }
                    }
                }
                
                // Compute local confidence
                _confidences[i] = (numSamples[i] == 0 ? 0.0f : float(numAccepted[i]) / float(numSamples[i]));
            }
            
            /*
             * Second voting loop
             */
            // Sort the confidences descending, reorder the input correspondences accordingly
            std::vector<float> confsort = _confidences;
            core::Correspondence::VecPtr corrsort(new core::Correspondence::Vec);
            *corrsort = reorder(corr, core::sort(confsort, false));
            
            // Loop over input correspondences again
//#ifdef _OPENMP
//#pragma omp parallel for
//#endif
            for(size_t i = 0; i < corr.size(); ++i) {
                // Sample relative pose
                _transformations[i] = ps.transformationRF(corr[i]);
                // First correspondence
                const core::Correspondence& c1 = corr[i];
                // Loop over the _samples best correspondences
                for(size_t j = 0; j < _samples; ++j) {
                    // Second correspondence
                    const core::Correspondence& c2 = (*corrsort)[j];
                    // Avoid self
                    if(c2.query != c1.query) {
                        ++numSamples[i];
                        if(local(c1, c2, simsq) && global(_transformations[i], c2, thressq))
                            ++numAccepted[i];
                    }
                }
                
                // Update confidence
                _confidences[i] = ( numSamples[i] == 0 ? 0.0f : float(numAccepted[i]) / float(numSamples[i]) );
            }
            
            /*
             * Finalization
             */
            // Sort confidences, reorder correspondences transformations accordingly
            confsort = _confidences;
            const std::vector<size_t> order = core::sort(confsort, false);
            *corrsort = core::reorder(corr, order);
            _transformations = core::reorder(_transformations, order);
            
            // Find confidence cut point using Otsu's method
            const float cut = thresholdOtsu(confsort, 0.0f, 1.0f);
            
            // Find number n of confidences making the cut
            size_t n = 0;
            while(n < confsort.size() && confsort[n] > cut)
                ++n;
            
            // Take out the n associated correspondences and transformations
            corrsort->resize(n);
            _transformations.resize(n);
            
            return corrsort;
        }

        template<typename PointT, typename RFT>
        float CorrespondenceVoting<PointT, RFT>::thresholdOtsu(const std::vector<float>& v, float lower, float upper) {
           // TODO: Set histogram size
           const size_t bins(std::sqrt(double(v.size())) + 0.5);

           // Compute an absolute histogram for the data
           size_t h[bins];
           core::hist<float>(v, bins, h, lower, upper);

           // Find the cut point between outliers and inliers using Otsu's thresholding method
           const size_t idxthres = core::otsu(h, bins);

           return (lower + (upper - lower) * float(idxthres) / float(bins - 1));

        }
    }
}

#endif
