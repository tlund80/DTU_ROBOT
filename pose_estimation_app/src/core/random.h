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

#ifndef COVIS_CORE_RANDOM_H
#define COVIS_CORE_RANDOM_H

// Own
#include "macros.h"
#include "range.h"

// Boost
#include <boost/random.hpp>
#include <boost/version.hpp>

// STL
#include <algorithm>
#include <cmath>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @brief Random number generator
         */
#if BOOST_VERSION >= 104700
        extern boost::random::mt19937 randgen;
#else
        extern boost::mt19937 randgen;
#endif
        
        /**
         * @ingroup core
         * @brief Random uniform number in range
         * @param lower lower bound (inclusive)
         * @param upper upper bound (inclusive)
         * @return random uniform number in [upper,lower]
         * @tparam RealT floating point type
         */
        template<typename RealT>
        inline RealT rand(RealT lower = RealT(0), RealT upper = RealT(1)) {
#if BOOST_VERSION >= 104700
            boost::random::uniform_real_distribution<RealT> dist(lower, upper);
            return dist(randgen);
#else
            boost::uniform_real<RealT> dist(lower, upper);
            boost::variate_generator<boost::mt19937&, boost::uniform_real<RealT> > var(randgen, dist);
            return var();
#endif
        }
        
        /**
         * @ingroup core
         * @brief Generate n random uniform numbers in range
         * @param n number of random numbers to generate
         * @param lower lower bound (inclusive)
         * @param upper upper bound (inclusive)
         * @return n random uniform numbers in [upper, lower]
         * @tparam RealT floating point type
         */
        template<typename RealT>
        inline std::vector<RealT> rand(size_t n, RealT lower = RealT(0), RealT upper = RealT(1)) {
            std::vector<RealT> result(n);
            for(size_t i = 0; i < n; ++i)
                result[i] = rand<RealT>(lower, upper);
            
            return result;
        }
        
        /**
         * @ingroup core
         * @brief Random number from the normal distribution
         * @param mean distribution mean
         * @param stddev distribution standard deviation
         * @return a sample from the normal distribution with mean m and standard deviation std
         * @tparam RealT floating point type
         */
        template<typename RealT>
        inline double randn(RealT mean = RealT(0), RealT stddev = RealT(1)) {
#if BOOST_VERSION >= 104700
            boost::random::normal_distribution<RealT> dist(mean, stddev);
            return dist(randgen);
#else
            boost::normal_distribution<RealT> dist(mean, stddev);
            boost::variate_generator<boost::mt19937&, boost::normal_distribution<RealT> > var(randgen, dist);
            return var();
#endif
        }
        
        /**
         * @ingroup core
         * @brief Generate n random numbers from the normal distribution
         * @param n number of random numbers to generate
         * @param mean distribution mean
         * @param stddev distribution standard deviation
         * @return n samples from the normal distribution with mean m and standard deviation std
         * @tparam RealT floating point type
         */
        template<typename RealT>
        inline std::vector<RealT> randn(size_t n, RealT mean = RealT(0), RealT stddev = RealT(1)) {
            std::vector<RealT> result(n);
            for(size_t i = 0; i < n; ++i)
                result[i] = randn<RealT>(mean, stddev);
            
            return result;
        }
        
        /**
         * @ingroup core
         * @brief Random integer in [0, size-1]
         * @param size range limit
         * @return random uniform number in [0, size-1]
         * @tparam IntT integer type
         */
        template<typename IntT>
        inline IntT randidx(IntT size) {
#if BOOST_VERSION >= 104700
            boost::random::uniform_int_distribution<IntT> dist(0, size - 1);
            return dist(randgen);
#else
            boost::uniform_int<IntT> dist(0, size - 1);
            boost::variate_generator<boost::mt19937&, boost::uniform_int<IntT> > vargen(randgen, dist);
            return vargen();
#endif
        }
        
        /**
         * @ingroup core
         * @brief Generate n unique random numbers in the range [0,size-1]
         * Thanks to Troels Bo Jørgensen for a faster algorithm for doing this
         * @param size range limit
         * @param n number of numbers
         * @return n unique random uniform numbers in the range [0, size-1]
         * @tparam IntT integer type
         * @exception if n > size
         */
        template<typename IntT>
        std::vector<IntT> randidx(IntT size, IntT n);
    }
}

#include "random_impl.hpp"

#endif
