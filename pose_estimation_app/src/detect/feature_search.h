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

#ifndef COVIS_DETECT_FEATURE_SEARCH_H
#define COVIS_DETECT_FEATURE_SEARCH_H

// Own
#include "search_base.h"
#include "../covis_config.h"
#include "../core/correspondence.h"

// PCL
#include <pcl/point_representation.h>

// FLANN
#include <flann/flann.hpp>

// OpenMP
#ifdef _OPENMP
#include <omp.h>
#endif

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class FeatureSearch
         * @brief Convenience class for FLANN-based feature matching
         * 
         * This class uses FLANN's randomized kd-trees for approximate nearest neighbor search in high-dimensional
         * (> 10) data. The approximation level is implicitly set by the number of checks the algorithm performs in the
         * trees (@ref setChecks()). For more information, see @cite muja2009fast.   
         * 
         * @note This class returns squared L2 distances
         * 
         * @tparam FeatureT feature type, e.g. <b>pcl::SHOT352</b>
         * @author Anders Glent Buch
         */
        template<typename FeatureT>
        class FeatureSearch : public SearchBase<FeatureT> {
            using SearchBase<FeatureT>::_target;
            
            public:
                /**
                 * Constructor: set number of randomized trees and a default number of checks
                 * @param trees number of randomized trees
                 */
                FeatureSearch(size_t trees = 4) :
                        _trees(trees),
                        _sparam(256),
#ifdef FLANN_1_8
                        _tree(flann::KDTreeIndexParams(trees)) {
#else
                        _tree(NULL) {
#endif
                    // Sanity check
                    COVIS_ASSERT(_trees > 0);
                    
                    // Set number of cores to use during search
#ifdef _OPENMP
                    _sparam.cores = omp_get_max_threads();
#endif
                }
                
                /**
                 * Destructor: clean up FLANN tree
                 */
                virtual ~FeatureSearch() {
                    if(_mtarget.ptr() != NULL)
                        delete[] _mtarget.ptr();
#ifndef FLANN_1_8
                    if(_tree != NULL)
                        delete _tree;
#endif
                }
                
                /**
                 * Set number of randomized trees
                 * @param trees number of randomized trees
                 */
                inline void setTrees(size_t trees) {
                    _trees = trees;
#ifdef FLANN_1_8
                    _tree = flann::Index<flann::L2<float> >(flann::KDTreeIndexParams(_trees));
#endif
                    if(_mtarget.ptr() != NULL) {
                        COVIS_MSG_WARN("FLANN tree has already been built once! " <<
                                "You should call setTrees() before setTarget(). Rebuilding...");
                        delete[] _mtarget.ptr();
#ifndef FLANN_1_8
                        if(_tree != NULL)
                            delete _tree;
#endif
                        index();
                    }
                }
                
                /**
                 * Set number of checks to perform during search
                 * @param checks number of checks
                 */
                inline void setChecks(size_t checks) {
                    _sparam.checks = int(checks);
                }
                
            private:
                /// Number of randomized trees
                size_t _trees;
                
                /// FLANN search parameters
                flann::SearchParams _sparam;
                
                /// FLANN matrix of target features
                flann::Matrix<float> _mtarget;
                
#ifdef FLANN_1_8
                /// Index type returned by FLANN
                typedef int idx_t;
                
                /// FLANN search tree
                flann::Index<flann::L2<float> > _tree;
#else
                /// Index type returned by FLANN
                typedef int idx_t;
                
                /// FLANN search tree
                flann::Index<flann::L2<float> >* _tree;
#endif
                
                /// PCL point representation, used for conversion to FLANN matrices
                pcl::DefaultFeatureRepresentation<FeatureT> _rep;
                
                /**
                 * Convert a single feature to a FLANN matrix, row-major
                 * @param query query feature
                 * @param mquery output FLANN matrix
                 */
                inline void convert(const FeatureT& query, flann::Matrix<float>& mquery) const {
//                    COVIS_ASSERT(_rep.isTrivial());
                    float* data = new float[_rep.getNumberOfDimensions()];
                    _rep.copyToFloatArray(query, data);
                    mquery = flann::Matrix<float>(data, 1, _rep.getNumberOfDimensions());
                }
                
                /**
                 * Convert a feature cloud to a FLANN matrix, row-major
                 * @param query query feature cloud
                 * @param mquery output FLANN matrix
                 */
                inline void convert(const typename pcl::PointCloud<FeatureT>& query,
                        flann::Matrix<float>& mquery) const {
//                    COVIS_ASSERT(_rep.isTrivial());
                    float* data = new float[_rep.getNumberOfDimensions() * query.size()];
                    for(size_t i = 0; i < query.size(); ++i)
                        _rep.copyToFloatArray(query[i], &data[i * _rep.getNumberOfDimensions()]);
                    mquery = flann::Matrix<float>(data, query.size(), _rep.getNumberOfDimensions());
                }
                
                /// @copydoc SearchBase::index()
                void index();
                
                /// @copydoc SearchBase::doKnn()
                void doKnn(const FeatureT& query, size_t k, core::Correspondence& result) const;
                
                /// @copydoc SearchBase::doRadius()
                void doRadius(const FeatureT& query, float r, core::Correspondence& result) const;
        };
        
        /**
         * @ingroup detect
         * @brief Perform a k-NN search in feature space
         * @param query query features
         * @param target target features to search into
         * @param k number of neighbors to search for
         * @param trees number of randomized trees to use
         * @param checks number of checks to perform during search
         * @return matches as correspondences
         */
        template<typename FeatureT>
        inline core::Correspondence::VecPtr computeKnnMatches(typename pcl::PointCloud<FeatureT>::ConstPtr query,
                typename pcl::PointCloud<FeatureT>::ConstPtr target,
                size_t k,
                size_t trees = 4,
                size_t checks = 256) {
            FeatureSearch<FeatureT> fm(trees);
            fm.setTarget(target);
            fm.setChecks(checks);
            
            return fm.knn(query, k);
        }
        
        /**
         * @ingroup detect
         * @brief Perform a 2-NN search in feature space and return correspondences with the distance set to the ratio
         * of the distance to the first match to the distance to the seconds, all squared for efficiency
         * @param query query features
         * @param target target features to search into
         * @param trees number of randomized trees to use
         * @param checks number of checks to perform during search
         * @return matches as correspondences
         */
        template<typename FeatureT>
        inline core::Correspondence::VecPtr computeRatioMatches(typename pcl::PointCloud<FeatureT>::ConstPtr query,
                typename pcl::PointCloud<FeatureT>::ConstPtr target,
                size_t trees = 4,
                size_t checks = 256) {
            FeatureSearch<FeatureT> fm(trees);
            fm.setTarget(target);
            fm.setChecks(checks);
            
            core::Correspondence::VecPtr c = fm.knn(query, 2);
            for(size_t i = 0; i < c->size(); ++i) {
                (*c)[i].distance[0] = (*c)[i].distance[0] / (*c)[i].distance[1];
                (*c)[i].match.resize(1);
                (*c)[i].distance.resize(1);
            }
            
            return c;
        }
    }
}

#include "feature_search_impl.hpp"
    
#endif
