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

#ifndef COVIS_DETECT_FIT_EVALUATION_H
#define COVIS_DETECT_FIT_EVALUATION_H

// Own
#include "detect_base.h"
#include "point_search.h"
#include "../core/correspondence.h"
#include "../core/detection.h"

// Boost
#include <boost/shared_ptr.hpp>

// STL
#include <cmath>

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class FitEvaluation
         * @brief Evaluate the relative pose between two point clouds based on different criteria
         * 
         * Use this class to repeatedly evaluate a number of pose hypothesis between one or more object/query models and
         * a fixed scene/target model. The target model is assumed static, and must be input to this class, and then it
         * is a matter of using the @ref update() function with the current pose hypothesis in each iteration of your
         * loop, followed by one or more evaluation functions, e.g. @ref inliers().
         * 
         * Alternatively, 
         * 
         * Do not forget to specify the main parameter, the inlier threshold, using @ref setInlierThreshold().
         * 
         * @tparam PointT point type for source and target
         * @author Anders Glent Buch
         */
        template<typename PointT>
        class FitEvaluation : public DetectBase {
            public:
                /// Pointer type
                typedef boost::shared_ptr<FitEvaluation<PointT> > Ptr;
                
                /// Matrix type, just a forward of @ref core::Detection::MatrixT
                typedef core::Detection::MatrixT MatrixT;
                
                /// Empty constructor
                FitEvaluation() :
                    _inlierThresholdSquared(-1.0f),
                    _occlusionRemoval(false) {}
                
                /**
                 * Constructor: set input target and create a search index
                 * @sa @ref setTarget()
                 * @param target point cloud
                 */
                FitEvaluation(typename pcl::PointCloud<PointT>::ConstPtr target) :
                    _inlierThresholdSquared(-1.0f),
                    _occlusionRemoval(false) {
                    setTarget(target);
                }
                
                /**
                 * Constructor: directly set the search index
                 * @sa @ref setSearch()
                 * @param search search index
                 */
                FitEvaluation(const PointSearch<PointT>& search) : _inlierThresholdSquared(-1.0f) {
                    setSearch(search);
                }
                
                /// Destructor
                virtual ~FitEvaluation() {}
                
                /**
                 * Update function: apply an input pose hypothesis to a source point cloud and call
                 * @ref update(typename pcl::PointCloud<PointT>::ConstPtr source) "update()"
                 * @param source source point cloud
                 * @param pose hypothesis pose
                 * @return all point correspondences between source and target
                 */
                core::Correspondence::VecPtr update(typename pcl::PointCloud<PointT>::ConstPtr source,
                        const MatrixT& pose);
                
                /**
                 * Update function: assume the source is already aligned to the scene, use the internal search tree to
                 * find point correspondences, and perform the following evaluations:
                 *   - Inlier count: the absolute (@ref inliers()) and relative
                 *    (@ref inlierFraction()) number of point matches up to the inlier threshold
                 *   - (R)MSE: the (root) mean squared error (@ref rmse() and @ref mse())
                 * @param source source point cloud, assumed aligned to the scene according to a pose hypothesis
                 * @return all point correspondences between source and target
                 */
                core::Correspondence::VecPtr update(typename pcl::PointCloud<PointT>::ConstPtr source);
                
                /**
                 * Apply a pose hypothesis to the source and evaluate the fit, in this case using precomputed point
                 * correspondences, e.g. from feature matching. This function applies the pose to the source, and then
                 * calls @ref update(typename pcl::PointCloud<PointT>::ConstPtr, core::Correspondence::VecConstPtr)
                 * @param source source point cloud
                 * @param pose hypothesis pose
                 * @param correspondences point correspondences source --> target
                 */
                void update(typename pcl::PointCloud<PointT>::ConstPtr source,
                        const MatrixT& pose,
                        core::Correspondence::VecConstPtr correspondences);
                
                /**
                 * Update function: assume the source is already aligned to the scene, use the input point
                 * correspondences, and perform the following evaluations:
                 *   - Inlier count: the absolute (@ref inliers()) and relative
                 *    (@ref inlierFraction()) number of point matches up to the inlier threshold
                 *   - (R)MSE: the (root) mean squared error (@ref rmse() and @ref mse())
                 * @param source source point cloud
                 * @param correspondences point correspondences source --> target
                 */
                void update(typename pcl::PointCloud<PointT>::ConstPtr source,
                        core::Correspondence::VecConstPtr correspondences);
                
                /**
                 * Set the target point cloud and create a search index for it
                 * @param target target point cloud
                 */
                inline void setTarget(typename pcl::PointCloud<PointT>::ConstPtr target) {
                    _target = target;
                    if(!_search)
                        _search.reset(new PointSearch<PointT>);
                    _search->setTarget(_target);
                }
                
                /**
                 * Directly set the search object, the search must be initialized with _target
                 * @param search search index
                 */
                inline void setSearch(typename PointSearch<PointT>::Ptr search) {
                    _search = search;
                }
                
                /**
                 * Get the search object, mutable for outside use
                 * @return search index
                 */
                inline typename PointSearch<PointT>::Ptr getSearch() {
                    return _search;
                }
                
                /**
                 * Set the Euclidean distance determining the inlier threshold
                 * @param inlierThreshold Euclidean (non-squared) inlier threshold
                 */
                inline void setInlierThreshold(float inlierThreshold) {
                    COVIS_ASSERT(inlierThreshold > 0.0f);
                    _inlierThresholdSquared = inlierThreshold * inlierThreshold;
                }
                
                /**
                 * Enable removal of self-occluded points
                 * This flag is only considered if the input point cloud contains normals, in which case the
                 * self-occluded points will have a normal vector pointing away from the camera
                 * @param occlusionRemoval occlusion removal flag
                 */
                inline void setOcclusionRemoval(bool occlusionRemoval) {
                    _occlusionRemoval = occlusionRemoval;
                }
                
                /**
                 * Get the inlier point correspondences
                 * @return inliers
                 */
                inline const core::Correspondence::Vec& getInliers() const {
                    return _state.inliers;
                }
                
                /**
                 * Get the inlier count
                 * @return inlier count
                 */
                inline size_t inliers() const {
                    return _state.inliers.size();
                }
                
                /**
                 * Get relative number of inliers to the total number of source points
                 * @return inlier fraction
                 */
                inline float inlierFraction() const {
                    return float(inliers()) / float(_state.size);
                }
                
                /**
                 * Get the squared fitness of the inliers
                 * @return mean squared Euclidean distance
                 */
                inline float mse() const {
                    return _state.mse;
                }
                
                /**
                 * Get the fitness of the inliers
                 * @return root mean squared Euclidean distance
                 */
                float rmse() const {
                    return std::sqrt(mse());
                }
                
                /**
                 * Get the number of occluded points, only non-zero if occlusion removal is enabled
                 * @return number of occluded points
                 */
                size_t occluded() const {
                    return _state.occluded.size();
                }
                
            private:
                /// Target point cloud representing the scene
                typename pcl::PointCloud<PointT>::ConstPtr _target;
                
                /// Search index into @ref _target
                typename PointSearch<PointT>::Ptr _search;
                
                /// Euclidean inlier threshold, squared
                float _inlierThresholdSquared;
                
                /// Occlusion removal flag, only used if the input point type contains normal information
                bool _occlusionRemoval;
                
                struct {
                    size_t size; ///< Number of source points, necessary for computing relative inlier fraction
                    core::Correspondence::Vec inliers; ///< Inlier correspondences
                    double mse; ///< MSE for the inliers
                    std::vector<size_t> occluded; ///< Indices of occluded points, if occlusion removal is enabled
                } _state;
        };
    }
}

#include "fit_evaluation_impl.hpp"

#endif
