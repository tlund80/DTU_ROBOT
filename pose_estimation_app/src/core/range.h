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

#ifndef COVIS_CORE_RANGE_H
#define COVIS_CORE_RANGE_H

// Own
#include "macros.h"

namespace covis {
    namespace core {
        /**
         * Return maximum of three numbers
         * @param a first number
         * @param b second number
         * @param c third number
         * @return max value
         */
        template<typename T>
        inline T max(T a, T b, T c) {
            return (a > b ? (a > c ? a : c) : (b > c ? b : c));
        }
        
        /**
         * Return minimum of three numbers
         * @param a first number
         * @param b second number
         * @param c third number
         * @return min value
         */
        template<typename T>
        inline T min(T a, T b, T c) {
            return (a < b ? (a < c ? a : c) : (b < c ? b : c));
        }
        
        /// \cond
        // Sort comparator based on templated pair - uses the second member
        template<typename PairT>
        struct CmpAscend {
                inline bool operator()(const PairT& p1, const PairT& p2) {
                    return (p1.second < p2.second);
                }
        };
        template<typename PairT>
        struct CmpDescend {
                inline bool operator()(const PairT& p1, const PairT& p2) {
                    return (p1.second > p2.second);
                }
        };
        /// \endcond
        
        /**
         * @ingroup core
         * 
         * @brief Sort a vector of comparable elements and return updated indices
         * @sa reorder()
         * @param v vector of comparable values
         * @param ascend set to true for ascending order (default)
         * @return indices into v such that v_sorted[i] = v[indices[i]]
         * @tparam T type comparable using '<' or '>' operator
         */
        template<typename T>
        inline std::vector<size_t> sort(std::vector<T>& v, bool ascend = true) {
            // Vector of <index,value>
            typedef std::pair<size_t, T> PairT;
            std::vector<PairT> id(v.size());
            for(size_t i = 0; i < v.size(); ++i)
                id[i] = std::pair<size_t, T>(i, v[i]);
            
            // Sort the pairs
            if(ascend)
                std::sort(id.begin(), id.end(), CmpAscend<PairT>());
            else
                std::sort(id.begin(), id.end(), CmpDescend<PairT>());
            
            // Copy back and reorder indices
            std::vector<size_t> idx(v.size());
            for(size_t i = 0; i < v.size(); ++i) {
                idx[i] = id[i].first;
                v[i] = id[i].second;
            }
            
            return idx;
        }
        
        /**
         * @ingroup core
         * @brief Reorder a vector as follows: result[i] = v[order[i]]
         * @sa sort()
         * @param v vector
         * @param order order
         * @return ordered vector
         */
        template<typename T>
        inline typename std::vector<T> reorder(const typename std::vector<T>& v, const std::vector<size_t>& order) {
            COVIS_ASSERT(v.size() == order.size());
            typename std::vector<T> result(v.size());
            for(size_t i = 0; i < v.size(); ++i) {
                const size_t& idxi = order[i];
                COVIS_ASSERT(idxi < v.size());
                result[i] = v[idxi];
            }
            
            return result;
        }
        
        /**
         * @ingroup core
         * @brief Perform natural sorting, alias humanly intuitive sorting of strings containing numbers
         * @param v vector of strings to sort
         * @param ascend set to true for ascending order (default)
         */
        void natsort(std::vector<std::string>& v, bool ascend = true);
        
        /**
         * @ingroup core
         * @brief Generate a monotonically increasing sequence of integers in the range {a, a+1, ..., b}
         * @param a lower limit (inclusive)
         * @param b upper limit (inclusive)
         * @return a vector with integers {a, a+1, ..., b}
         * @tparam IntT integer type
         * @exception if a > b
         */
        template<typename IntT>
        inline std::vector<IntT> range(IntT a, IntT b) {
            COVIS_ASSERT(b >= a);
            const IntT size = 1 + b - a;
            std::vector<IntT> result(size);
            for(IntT i = 0; i < size; ++i)
                result[i] = a + i;
            return result;
        }
        
        /**
         * @ingroup core
         * @brief Generate a monotonically increasing sequence of integers in the range {0, 1, ..., size-1}
         * \note Contrary to @ref range(IntT a, IntT b), this function is exclusive in the upper bound
         * @param size upper limit (exclusive)
         * @return a vector with numbers {0, 1, ..., size-1}
         * @tparam IntT integer type
         * @exception if size < 0
         */
        template<typename IntT>
        inline std::vector<IntT> range(IntT size) {
            return range<IntT>(0, size - 1);
        }
        
        /**
         * @ingroup core
         * Return true if all elements are zero
         * @param v vector
         * @return true if all elements of v are zero
         */
        template<typename T>
        inline bool zero(const std::vector<T>& v) {
            const T val(0);
            for(typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it)
                if(*it != val)
                    return false;
            
            return true;
        }
        
        /**
         * @ingroup core
         * Mask a vector, i.e. extract only elements where input mask is true (or false if inverted)
         * @param v input vector
         * @param m mask, must be same size as vector
         * @param invert set to true to extract false elements
         * @return elements of vector where mask is true (or false if invert is true)
         * @tparam VectorT container type, implementing [] operator, e.g. std::vector<T> or pcl::PointCloud<T>
         */
        template<typename VectorT>
        inline VectorT mask(const VectorT& v, const std::vector<bool>& m, bool invert = false) {
            COVIS_ASSERT(v.size() == m.size());
            VectorT result;
            result.reserve(v.size());
            for(size_t i = 0; i < v.size(); ++i)
                if((m[i] && !invert) || (!m[i] && invert))
                    result.push_back(v[i]);
                
            return result;
        }
        
        /**
         * @ingroup core
         * Extract indexed elements from a vector
         * @param v input vector
         * @param idx indices of elements from v to extract
         * @param invert set to true to extract all other elements than the indexed
         * @return elements of vector where mask is true (or false if invert is true)
         * @tparam VectorT container type, implementing [] operator, e.g. std::vector<T> or pcl::PointCloud<T>
         * @tparam IntT index type for indexing into v
         */
        template<typename VectorT, typename IntT>
        inline VectorT extract(const VectorT& v, const typename std::vector<IntT>& idx, bool invert = false) {
            std::vector<bool> maskk(v.size(), false);
            for(typename std::vector<IntT>::const_iterator it = idx.begin(); it != idx.end(); ++it)
                maskk[*it] = true;
            
            return mask<VectorT>(v, maskk, invert);
        }
        
        /**
         * @ingroup core
         * Print a vector of streamable elements
         * @param v vector
         * @param os output stream
         * @param prefix string to insert before printing the vector
         * @param suffix string to insert right after printing the vector
         * @param delim delimiter string to insert between elements
         * @param term termination string to insert after all elements and the suffix have been printed
         * @return modified stream
         */
        template<typename T>
        inline std::ostream& print(const std::vector<T>& v, std::ostream& os = std::cout,
                const std::string& prefix = "[", const std::string& suffix = "]",
                const std::string& delim = " ", const std::string& term = "\n") {
            if(!v.empty()) {
                os << prefix;
                for(typename std::vector<T>::const_iterator it = v.begin(); it != v.end()-1; ++it)
                    os << *it << delim;
                os << v.back() << suffix;
            }
            
            os << term;
            
            return os;
        }
    }
}
#endif
