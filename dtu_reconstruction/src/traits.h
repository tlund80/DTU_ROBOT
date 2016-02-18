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

#ifndef COVIS_CORE_TRAITS_H
#define COVIS_CORE_TRAITS_H

// Own
#include "ids.h"

// Boost
#include <boost/mpl/assert.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/fold.hpp>

// PCL
#include <pcl/point_types.h>

namespace covis {
    namespace core {
        /*
         * Code below stolen from here:
         * https://github.com/PointCloudLibrary/pcl/issues/440
         */
        
        /**
         * @ingroup core
         * @struct TypeCheck
         * @brief Static check if a Boost MPL sequence contains a type
         * 
         * Example:
         * @code
         * FieldCheck<boost::mpl::vector<bool, char>, bool>::value; // True
         * FieldCheck<boost::mpl::vector<bool, char>, int>::value; // False
         * @endcode
         * 
         * @tparam TypeVector Boost MPL vector of types
         * @tparam T type to check for
         */
        template<typename TypeVector, typename T>
        struct TypeCheck : boost::mpl::contains<TypeVector, T>::type {};
        
        /**
         * @ingroup core
         * @struct PCLFieldCheck
         * @brief Static check if a PCL point type contains a field
         * 
         * Example:
         * @code
         * PCLFieldCheck<pcl::PointXYZ, pcl::fields::x>::value;
         * @endcode
         * 
         * @tparam PointT PCL point type
         * @tparam Field PCL field, fully qualified
         * @sa @ref TypeCheck, @ref COVIS_STATIC_ASSERT_PCL_FIELD
         */
        template<typename PointT, typename Field>
        struct PCLFieldCheck : TypeCheck<typename pcl::traits::fieldList<PointT>::type, Field> {};
        
        /**
         * @ingroup core
         * @struct PCLFieldVectorCheck
         * @brief Static check if a PCL point type contains a vector of fields
         * 
         * Example:
         * @code
         * PCLFieldCheck<pcl::PointXYZ, boost::mpl::vector<pcl::fields::x, pcl::fields::y, pcl::fields::z> >::value;
         * @endcode
         * 
         * @tparam PointT PCL point type
         * @tparam Field Boost MPL vector of PCL fields
         * @sa @ref PCLFieldCheck, @ref TypeCheck
         */
        template<typename PointT, typename FieldVector>
        struct PCLFieldVectorCheck :
                boost::mpl::fold<FieldVector,
                boost::mpl::bool_<true>,
                boost::mpl::and_<boost::mpl::_1, PCLFieldCheck<PointT, boost::mpl::_2> > >::type {};
                
        /**
         * @ingroup core
         * 
         * @struct HasXYZ
         * 
         * @brief Internal traits struct used for determining whether a point type contains XYZ data
         * @tparam PointT point type
         * @sa @ref PCLFieldCheck, @ref PCLFieldVectorCheck
         */
        template<typename PointT>
        struct HasXYZ {
            /// Boost MPL vector of XYZ fields
            typedef boost::mpl::vector<pcl::fields::x, pcl::fields::y, pcl::fields::z> FieldVector;
            
            /// True if point type has all fields
            static const bool value = PCLFieldVectorCheck<PointT, FieldVector>::value;
        };
        
        /**
         * @ingroup core
         * 
         * @struct HasNormal
         * 
         * @brief Internal traits struct used for determining whether a point type contains a normal vector
         * @tparam PointT point type
         * @sa @ref PCLFieldCheck, @ref PCLFieldVectorCheck
         */
        template<typename PointT>
        struct HasNormal {
            /// Boost MPL vector of normal fields
            typedef boost::mpl::vector<pcl::fields::normal_x, pcl::fields::normal_y, pcl::fields::normal_z> FieldVector;
            
            /// True if point type has normal data
            static const bool value = PCLFieldVectorCheck<PointT, FieldVector>::value;
        };
        
// These pragmas temporarily disable unused parameter warnings - this because of the peculiar GetNormal behavior
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4100)
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#elif defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
        /**
         * @ingroup core
         * 
         * @struct GetNormal
         * 
         * @brief Internal traits struct used for getting the components of a normal vector, if they exist
         * @tparam PointT point type
         * @tparam HasN normal existence flag
         * @sa @ref HasNormal
         */
        template<typename PointT, bool HasN = HasNormal<PointT>::value>
        struct GetNormal {
            /**
             * Get the third component of a normal vector, if it exists
             * @param p input point
             * @param defaultValue return this value if normal information does not exist
             * @return third normal component or default value if no normal exists
             */
            static inline float nz(const PointT& p, float defaultValue) {
                return defaultValue;
            }
        };
        
        /// @cond
        template<typename PointT>
        struct GetNormal<PointT,true> {
            static inline float nz(const PointT& p, float defaultValue = 0.0) {
                return p.normal_z;
            }
        };
        /// @endcond
#if defined(_MSC_VER)
#pragma warning(pop)
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#elif defined(__clang__)
#pragma clang diagnostic pop
#endif
        
        /**
         * @ingroup core
         * 
         * @struct HasRF
         * 
         * @brief Internal traits struct used for determining whether a point type contains a reference frame
         * @note This checks that either pcl::fields::rf OR pcl::fields::x|y|z_axis exist
         * @tparam PointT point type
         * @sa @ref PCLFieldCheck, @ref PCLFieldVectorCheck
         */
        template<typename PointT>
        struct HasRF {
            /// Boost MPL vector of RF axis fields
            typedef boost::mpl::vector<pcl::fields::x_axis, pcl::fields::y_axis, pcl::fields::z_axis> FieldVector;
            
            /// True if point type has RF data
            static const bool value =
                    (PCLFieldCheck<PointT, pcl::fields::rf>::value || PCLFieldVectorCheck<PointT, FieldVector>::value);
        };
        
        /**
         * @ingroup core
         * 
         * @struct HasRGB
         * 
         * @brief Internal traits struct used for determining whether a point type contains RGB data
         * @note This checks that either pcl::fields::rgb OR pcl::fields::rgba exist
         * @tparam PointT point type
         * @sa @ref PCLFieldCheck, @ref PCLFieldVectorCheck
         */
        template<typename PointT>
        struct HasRGB {
            /// True if point type has RGB data
            static const bool value =
                    (PCLFieldCheck<PointT, pcl::fields::rgb>::value || PCLFieldCheck<PointT, pcl::fields::rgba>::value);
        };
        
        /**
         * @ingroup core
         * 
         * @struct HasID
         * 
         * @brief Internal traits struct used for determining whether a point type contains intrinsic dimensionality
         * @tparam PointT point type
         * @sa @ref PCLFieldCheck, @ref PCLFieldVectorCheck
         */
        template<typename PointT>
        struct HasID {
            /// Boost MPL vector of ID fields
            typedef boost::mpl::vector<pcl::fields::id0, pcl::fields::id1, pcl::fields::id2> FieldVector;
            
            /// True if point type has ID data
            static const bool value = PCLFieldVectorCheck<PointT, FieldVector>::value;
        };
        
        /**
         * @ingroup core
         * 
         * @struct HasScale
         * 
         * @brief Internal traits struct used for determining whether a point type contains scale
         * @tparam PointT point type
         * @sa @ref PCLFieldCheck, @ref PCLFieldVectorCheck
         */
        template<typename PointT>
        struct HasScale {
            /// True if point type has scale data
            static const bool value = PCLFieldCheck<PointT, pcl::fields::scale>::value;
        };
    }
}

#endif
