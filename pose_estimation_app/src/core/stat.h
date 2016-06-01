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

#ifndef COVIS_CORE_STAT_H
#define COVIS_CORE_STAT_H

// Own
#include "macros.h"

// STL
#include <algorithm>
#include <cmath>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @brief Compute the arithmetic sample mean of a vector of numbers
         * @param v vector of numbers
         * @return mean sample mean
         * @exception if v is empty
         */
        template<typename RealT>
        RealT mean(const std::vector<RealT>& v);
        
        /**
         * @ingroup core
         * @brief Compute the sample variance of a vector of n numbers with precomputed mean
         * @param v vector of numbers
         * @param mean precomputed mean
         * @param unbiased if set to true, the division factor is set to n-1 where n is the size of sample
         * @return sample variance
         * @exception if size of v is < 2
         */
        template<typename RealT>
        RealT var(const std::vector<RealT>& v, RealT mean, bool unbiased = true);
        
        /**
         * @ingroup core
         * @brief Compute the sample variance of a vector of n numbers
         * @param v vector of numbers
         * @param unbiased if set to false, the division factor is set to n instead of n-1 where n is the size of sample
         * @return sample variance
         * @exception if size of v is < 2
         */
        template<typename RealT>
        inline RealT var(const std::vector<RealT>& v, bool unbiased = true) {
            return var<RealT>(v, mean<RealT>(v), unbiased);
        }
        
        /**
         * @ingroup core
         * @brief Compute the sample standard deviation of a vector of numbers
         * @param v vector of numbers
         * @param unbiased if set to false, the division factor is set to n instead of n-1 where n is the size of sample
         * @return sample standard deviation
         * @exception if size of v is < 2
         */
        template<typename RealT>
        inline RealT stddev(const std::vector<RealT>& v, bool unbiased = true) {
            return sqrtf(var<RealT>(v, unbiased));
        }
        
        /**
         * @ingroup core
         * @brief Compute the sample standard deviation of a vector of numbers with precomputed mean
         * @param v vector of numbers
         * @param mean precomputed mean
         * @param unbiased if set to false, the division factor is set to n instead of n-1 where n is the size of sample
         * @return sample standard deviation
         * @exception if size of v is < 2
         */
        template<typename RealT>
        inline RealT stddev(const std::vector<RealT>& v, RealT mean, bool unbiased = true) {
            return sqrtf(var<RealT>(v, mean, unbiased));
        }
        
        /**
         * @ingroup core
         * @brief Compute median of a vector of numbers
         * @param v vector
         * @return median
         * @exception if size of v is empty
         */
        template<typename RealT>
        RealT median(const std::vector<RealT>& v);
        
        /**
         * 
         * @ingroup core
         * @brief Compute an absolute histogram of floating point data values
         * @note If any data value is outside [lower, upper], it is shifted to the nearest limit
         * @param data input data values
         * @param bins output histogram dimension
         * @param result absolute histogram, must be preallocated!
         * @param lower lower bound
         * @param upper upper bound
         * @sa @ref rhist()
         */
        template<typename RealT>
        inline void hist(const typename std::vector<RealT>& data,
                size_t bins,
                size_t* result,
                RealT lower = RealT(0),
                RealT upper = RealT(1));
        
        /**
         * @ingroup core
         * @brief Compute a relative histogram of floating point data values
         * @note If any data value is outside [lower, upper], it is shifted to the nearest limit
         * @param data input data values
         * @param bins output histogram dimension
         * @param result relative histogram, must be preallocated!
         * @param lower lower bound
         * @param upper upper bound
         * @sa @ref hist()
         */
        template<typename RealT>
        inline void rhist(const typename std::vector<RealT>& data,
                size_t bins,
                RealT* result,
                RealT lower = RealT(0),
                RealT upper = RealT(1));
        
        /**
         * 
         * @ingroup core
         * @brief Compute an absolute 2D histogram of corresponding floating point data values
         * @note If any data value is outside [lower, upper], it is shifted to the nearest limit
         * @param data1 input data values along the first dimension
         * @param data2 input data values along the second dimension, must be one to one with data1!
         * @param bins1 output histogram bins along the first dimension
         * @param bins2 output histogram bins along the first dimension
         * @param result absolute histogram, must be preallocated to bins1 * bins2!
         * @param lower1 lower bound of the first dimension
         * @param upper1 upper bound of the first dimension
         * @param lower2 lower bound of the second dimension
         * @param upper2 upper bound of the second dimension
         * @sa @ref rhist2()
         */
        template<typename RealT>
        inline void hist2(const typename std::vector<RealT>& data1,
                const typename std::vector<RealT>& data2,
                size_t bins1,
                size_t bins2,
                size_t* result,
                RealT lower1,
                RealT upper1,
                RealT lower2,
                RealT upper2);


        /**
         * @ingroup core
         * @brief Find the optimal threshold value in a bimodal histogram using Otsu's method
         *
         * Algorithm taken from:
         * http://en.wikipedia.org/wiki/Otsu's_method
         *
         * @param hist absolute histogram data, must be of size bins!
         * @param bins histogram dimension
         * @return optimal threshold in [0, size(histogram)-1]
         * @exception if bins == 0
         */
        size_t otsu(const size_t* hist, size_t bins);
    }
}

#include "stat_impl.hpp"

#endif
