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

#ifndef COVIS_FEATURE_NORMAL_CORRECTION_MANIFOLD_H
#define COVIS_FEATURE_NORMAL_CORRECTION_MANIFOLD_H

#include "macros.h"

namespace covis {
    namespace feature {
        /**
         * @class NormalCorrectionManifold
         * @ingroup feature
         * @brief Normal correction class for complete models
         * 
         * This class implements a method for getting consistent orientations for a set of precomputed normals.
         * For complete smooth models, this means that all normals are pointing "outwards" relative to the centroid.
         * Thus, any two close points must have almost parallel normals.
         * The method is suitable for these kinds of full models; view-based models only require you to the flip the
         * normal against the viewpoint to get consistent orientations.
         * 
         * This method is loosely related to the following work, but avoids the computation of a spanning tree:
         * 
         * H. Hoppe, T. DeRose, T. Duchamp.@n
         * "Surface Reconstruction from Unorganized Points".@n
         * <i>ACM 26(2):71--78, 1992.</i>
         * 
         * By default, this class uses a 10-NN search for speed, although this can be customized, also to a radius
         * search.
         *
         * @tparam PointNT point type, containing both XYZ and normal data
         * @author Anders Glent Buch
         * @example example/normal_correction_manifold/normal_correction_manifold.cpp
         */
        template<typename PointNT>
        class NormalCorrectionManifold {
            public:
                /// Default constructor, sets up a 10-NN search
                NormalCorrectionManifold() : _useKNN(true), _k(10), _radius(0.01f) {}

                /// Empty destructor
                virtual ~NormalCorrectionManifold() {}
                
                /**
                 * Correct normals
                 * @param cloud input/output point cloud, only normal fields are modified
                 * @return number of normals corrected
                 */
                size_t compute(pcl::PointCloud<PointNT>& cloud);
                
                /**
                 * Set to true to use k-NN search, false to use radius search
                 * @param useKNN k-NN or radius search flag
                 */
                inline void setUseKNN(bool useKNN) {
                    _useKNN = useKNN;
                }
                
                /**
                 * Set number of k-NN for neighbor search
                 * @param k number of k-NNs
                 */
                inline void setK(size_t k) {
                    _k = k;
                }
                
                /**
                 * Set radius for neighbor search
                 * @param radius radius
                 */
                inline void setRadius(float radius) {
                    _radius = radius;
                }
                
            private:
                /// Set to true to use k-NN search, false to use radius search
                bool _useKNN;
                
                /// Number of neighbors for k-NN search
                size_t _k;
                
                /// Search radius for radius search
                float _radius;
                
                /**
                 * Invert the orientation of a normal vector
                 * @param p input/output point with normal
                 */
                inline void flip(PointNT& p) {
                    p.normal_x = -p.normal_x;
                    p.normal_y = -p.normal_y;
                    p.normal_z = -p.normal_z;
                }
        };
        
        /**
         * @ingroup feature
         * Perform normal correction on a manifold using @ref NormalCorrectionManifold by a either a k-NN search or a
         * radius search
         * @param cloud input/output point cloud, must contain XYZ and normal data
         * @param useKNN set to true to use k-NN search, false to use radius search
         * @param k number of k-NNs
         * @param radius search radius
         * @return number of corrected normals
         */
        template<typename PointNT>
        inline size_t computeCorrectedNormals(pcl::PointCloud<PointNT>& cloud,
                bool useKNN = true,
                size_t k = 10,
                float radius = 0.01f) {
            NormalCorrectionManifold<PointNT> ncm;
            ncm.setUseKNN(useKNN);
            ncm.setK(k);
            ncm.setRadius(radius);
            
            return ncm.compute(cloud);
        }
    }
}

#include "normal_correction_manifold_impl.hpp"

#endif
