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

#ifndef COVIS_DETECT_RANSAC_H
#define COVIS_DETECT_RANSAC_H

// Own
#include "detect_base.h"
#include "fit_evaluation.h"
#include "point_search.h"
#include "../core/correspondence.h"

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class Ransac
         * @brief Pose estimation using RANSAC
         * 
         * This class implements the classical RANSAC @cite fischler1981random algorithm for pose estimation, with a few
         * modifications possible:
         *   - You can disable the re-estimation of the model using the consensus set using @ref setReestimatePose().
         *   - The "data points" of this RANSAC are a set of precomputed feature correspondences
         *   (@ref setCorrespondences()), from which pose hypotheses are repeatedly sampled. By default, this class
         *   evaluates inliers using this set of correspondences, i.e. how many of the input correspondences are
         *   consistent with a sampled pose. In some cases, it can be more robust to apply the pose to the full model,
         *   and perform inlier evaluation between the point clouds. This behavior can be enabled using
         *   @ref setFullEvaluation(), but be aware that this requires expensive searches for point correspondences.
         *   - This class uses @ref FitEvaluation for verifying a pose model. By default, the object is set to use the
         *   number of inliers as evaluation criterion, as in classical RANSAC. However, you can change this by
         *   modifying the @ref FitEvaluation object using @ref getFitEvaluation(). The @ref FitEvaluation object can
         *   also be set directly from the outside to save initialization time using @ref setFitEvaluation(), otherwise
         *   it is automatically initialized by this class.
         *   
         * Additionally, this class offers a few extensions to classical RANSAC:
         *   - The sampling process can be biased by the quality of the correspondences, in contrast with classical
         *   RANSAC which treats all data points uniformly. This is achieved by weighting each correspondence by the 
         *   negative of the distance. This behavior is enabled using @ref setWeightedSampling(). Note that this is
         *   supported only for Boost version 1.47 and up.
         *   - The early prerejection of bad samples of @cite buch2013pose can be enabled using @ref setPrerejection()
         *   and @ref setPrerejectionSimilarity(). Remember to increase the number of iterations when using this!
         * 
         * @todo Allow for a number of flags that can specify one or more criteria (maybe even by AND/OR) to use for
         * accepting a pose hypothesis. For now we use the inlier count, as in classical RANSAC.
         * 
         * @tparam point type
         * @author Anders Glent Buch
         * 
         * @example example/ransac/ransac.cpp
         */
        template<typename PointT>
        class Ransac : public DetectBase {
            public:
                /**
                 * Constructor: set default parameters:
                 *   - Iterations (@ref setIterations()): 1000
                 *   - Inlier threshold (@ref setInlierThreshold()): 0.01
                 *   - Inlier fraction (@ref setInlierFraction()): 0.1
                 *   - Sample size (@ref setSampleSize()): 3
                 *   - Re-estimation of pose using consensus set (@ref setReestimatePose()): on
                 *   - Full evaluation of inliers using the input models (@ref setFullEvaluation()): off
                 *   - Prerejection (@ref setPrerejection() and @ref setPrerejectionSimilarity()): off
                 */
                Ransac() :
                    _iterations(1000),
                    _inlierThreshold(0.01),
                    _inlierFraction(0.1),
                    _sampleSize(3),
                    _reestimatePose(true),
                    _fullEvaluation(false),
                    _weightedSampling(false),
                    _prerejection(false),
                    _prerejectionSimilarity(0.9),
                    _verbose(false) {}
                
                /**
                 * Run RANSAC
                 * @return best detection, if any was found - this can be verified directly in a boolean expresstion:
                 * @code
                 * covis::core::Detection d = estimate();
                 * if(d)
                 *   // Do something...
                 * @endcode
                 */
                core::Detection estimate();
                
                /**
                 * Get all hypothesis detections with enough inliers generated by the RANSAC loop, only valid after a
                 * call to @ref estimate()
                 * @return all detections with enough inliers
                 */
                inline const core::Detection::Vec& getAllDetections() const {
                    return _allDetections;
                }
                
                /**
                 * Set the source point cloud
                 * @param source source point cloud
                 */
                inline void setSource(typename pcl::PointCloud<PointT>::ConstPtr source) {
                    _source = source;
                }
                
                /**
                 * Set the target point cloud
                 * @param target target point cloud
                 */
                inline void setTarget(typename pcl::PointCloud<PointT>::ConstPtr target) {
                    _target = target;
                }
                
                /**
                 * Set point correspondences to sample from
                 * @param correspondences correspondences source --> target
                 */
                inline void setCorrespondences(core::Correspondence::VecConstPtr correspondences) {
                    _correspondences = correspondences;
                }

                /**
                 * Set the target search
                 * @param search mutable target search
                 */
                inline void setSearch(typename PointSearch<PointT>::Ptr search) {
                    _search = search;
                }
                
                /**
                 * Set the fit evaluator
                 * @param fitEvaluation mutable fit evaluator
                 */
                inline void setFitEvaluation(typename FitEvaluation<PointT>::Ptr fitEvaluation) {
                    _fitEvaluation = fitEvaluation;
                }
                
                /**
                 * Get the fit evaluator for the target
                 * @note This object remains uninitialized until the first call to @ref estimate() or
                 * @ref setFitEvaluation()
                 * @return mutable fit evaluator
                 */
                inline typename FitEvaluation<PointT>::Ptr getFitEvaluation() {
                    return _fitEvaluation;
                }
                
                /**
                 * Set RANSAC iterations
                 * @param iterations iterations
                 */
                inline void setIterations(size_t iterations) {
                    _iterations = iterations;
                }
                
                /**
                 * Set Euclidean inlier threshold
                 * @param inlierThreshold Euclidean inlier threshold
                 */
                inline void setInlierThreshold(float inlierThreshold) {
                    _inlierThreshold = inlierThreshold;
                }
                
                /**
                 * Set required inlier fraction (must be in [0,1])
                 * @param inlierFraction required inlier fraction
                 */
                inline void setInlierFraction(float inlierFraction) {
                    _inlierFraction = inlierFraction;
                }
                
                /**
                 * Set sample size for generating a pose (must be >= 3)
                 * @param sampleSize sample size
                 */
                inline void setSampleSize(size_t sampleSize) {
                    _sampleSize = sampleSize;
                }
                
                /**
                 * Set the re-estimation flag for the pose
                 * In the standard RANSAC formulation, the model is re-estimated using the consensus set, however this
                 * will decrease the speed of the algorithm. If you set this to false, you can expect higher speed at
                 * the expense of potential loss of accuracy in the poses. 
                 * @param reestimatePose pose re-estimation flag
                 */
                inline void setReestimatePose(bool reestimatePose) {
                    _reestimatePose = reestimatePose;
                }
                
                /**
                 * Enable full evaluation, i.e. evaluation of inliers by point correspondence search between the source
                 * and the target model.
                 * @param fullEvaluation full evaluation flag
                 */
                inline void setFullEvaluation(bool fullEvaluation) {
                    _fullEvaluation = fullEvaluation;
                }
                
                /**
                 * Set the weighted sampling flag 
                 * @note This functionality is only supported for Boost >= 1.47
                 * @param weightedSampling weighted sampling flag
                 */
                inline void setWeightedSampling(bool weightedSampling) {
                    _weightedSampling = weightedSampling;
                }
                
                /**
                 * Set the prerejection flag 
                 * @param prerejection prerejection flag
                 */
                inline void setPrerejection(bool prerejection) {
                    _prerejection = prerejection;
                }
                
                /**
                 * Set the prerejection similarity threshold
                 * @param prerejectionSimilarity prerejection similarity threshold in [0,1]
                 */
                inline void setPrerejectionSimilarity(float prerejectionSimilarity) {
                    _prerejectionSimilarity = prerejectionSimilarity;
                }
                
                /**
                 * Set verbose flag for printing
                 * @param verbose verbose flag
                 */
                inline void setVerbose(bool verbose) {
                    _verbose = verbose;
                }
                
            private:
                /// All detections generated by the internal RANSAC loop
                core::Detection::Vec _allDetections;
                
                /// Source point cloud to be placed into @ref _target
                typename pcl::PointCloud<PointT>::ConstPtr _source;
                
                /// Target point cloud
                typename pcl::PointCloud<PointT>::ConstPtr _target;
                
                /// Correspondences source --> target
                core::Correspondence::VecConstPtr _correspondences;

                /// Target search, used during fit evaluation
                typename PointSearch<PointT>::Ptr _search;
                
                /// Evaluation object for generated pose hypotheses
                typename FitEvaluation<PointT>::Ptr _fitEvaluation;
                
                /// Number of RANSAC iterations
                size_t _iterations;
                
                /// Euclidean inlier threshold
                float _inlierThreshold;
                
                /// Visibility or required fraction of inliers of the source points [0,1]
                float _inlierFraction;
                
                /// Number of point matches to sample when generating a pose
                size_t _sampleSize;
                
                /// If set to true (default), a pose is re-estimated if it has enough inliers
                bool _reestimatePose;
                
                /// Full evaluation flag
                bool _fullEvaluation;
                
                /// Weighted sampling flag
                bool _weightedSampling;
                
                /// If set to true, apply early prerejection
                bool _prerejection;
                
                /// If prerejection is set to true, use this polygonal similarity threshold
                float _prerejectionSimilarity;
                
                /// Verbose flag
                bool _verbose;
        };
        
        /**
         * @ingroup detect
         * @brief Perform RANSAC pose estimation between an object (source) and a scene (target)
         * @param source source point cloud, to be aligned with target
         * @param target target point cloud
         * @param correspondences point correspondences source --> target
         * @param iterations number of RANSAC iterations
         * @param inlierThreshold Euclidean inlier threshold, defaults to 0.01
         * @param inlierFraction required inlier fraction to the total number of source points, defaults to 10 %
         * @param sampleSize number of point pairs to sample for pose hypothesis generation, defaults to 3
         * @param reestimatePose if true (default), reestimate the pose using the consensus set
         * @param fullEvaluation if true, perform full evaluation of pose hypotheses using source/target models
         * @param occlusionRemoval occlusion removal flag, only considered if PointT contains normals
         * @param verbose if true, print status messages
         * @return best detection, if any was found - this can be verified by checking that the inlier fraction
         * (@ref core::Detection::inlierfrac) is > 0
         */
        template<typename PointT>
        inline core::Detection ransac(typename pcl::PointCloud<PointT>::ConstPtr source,
                typename pcl::PointCloud<PointT>::ConstPtr target,
                core::Correspondence::VecConstPtr correspondences,
                size_t iterations = 1000,
                float inlierThreshold = 0.01,
                float inlierFraction = 0.1,
                size_t sampleSize = 3,
                bool reestimatePose = true,
                bool fullEvaluation = false,
                bool occlusionRemoval = false,
                bool verbose = false) {
            typename FitEvaluation<PointT>::Ptr eval(new detect::FitEvaluation<PointT>(target));
            eval->setOcclusionRemoval(occlusionRemoval);
            
            detect::Ransac<PointT> ransac;
            
            ransac.setSource(source);
            ransac.setTarget(target);
            ransac.setCorrespondences(correspondences);
            ransac.setFitEvaluation(eval);
            
            ransac.setIterations(iterations);
            ransac.setInlierThreshold(inlierThreshold);
            ransac.setInlierFraction(inlierFraction);
            ransac.setSampleSize(sampleSize);
            ransac.setReestimatePose(reestimatePose);
            ransac.setFullEvaluation(fullEvaluation);
            ransac.setVerbose(verbose);
            
            return ransac.estimate();
        }
    }
}

#include "ransac_impl.hpp"

#endif
