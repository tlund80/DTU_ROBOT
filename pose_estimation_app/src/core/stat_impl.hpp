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

#ifndef COVIS_CORE_STAT_IMPL_HPP
#define COVIS_CORE_STAT_IMPL_HPP

namespace covis {
    namespace core {
        template<typename RealT>
        RealT mean(const std::vector<RealT>& v) {
            COVIS_ASSERT(!v.empty());

            // TODO: Do this with double precision, then downcast in the end
            double result = 0.0;
            for (typename std::vector<RealT>::const_iterator it = v.begin(); it != v.end(); ++it)
                result += *it;
            result /= double(v.size());
            
            return RealT(result);
        }
        
        template<typename RealT>
        RealT var(const std::vector<RealT>& v, RealT mean, bool unbiased) {
            COVIS_ASSERT(v.size() >= 2);
            
            // TODO: Do this with double precision, then downcast in the end
            double result = 0.0;
            for(typename std::vector<RealT>::const_iterator it = v.begin(); it != v.end(); ++it) {
                const double diff = *it - mean;
                result += diff * diff;
            }
            result /= (unbiased ? double(v.size() - 1) : double(v.size()));
            
            return RealT(result);
        }
        
        template<typename RealT>
        RealT median(const std::vector<RealT>& v) {
            COVIS_ASSERT(!v.empty());
            switch(v.size()) {
                case 1:
                    return v[0];
                case 2:
                    return RealT(0.5*(v[0] + v[1]));
                default: {
                    std::vector<RealT> vv = v; // Modified sequence
                    typename std::vector<RealT>::iterator middle = vv.begin() + ptrdiff_t(v.size() / 2);
                    std::nth_element(vv.begin(), middle, vv.end());
                    if(v.size() & 1) // Odd
                        return *middle;
                    else
                        return RealT( 0.5 * (*middle + *std::max_element(vv.begin(), middle - 1)) );
                }
            }
        }
        
        template<typename RealT>
        void hist(const typename std::vector<RealT>& data,
                size_t bins,
                size_t* result,
                RealT lower,
                RealT upper) {
            // Sanity check
            COVIS_ASSERT(bins > 0);
            
            // Initialize result to zero
            std::memset(result, 0, bins * sizeof(size_t));
            
            // Increment
            const RealT inc = RealT(bins) / (upper - lower);
            
            // Compute absolute histogram
            for(typename std::vector<RealT>::const_iterator it = data.begin(); it != data.end(); ++it) {
                // Handle limits
                if(*it < lower)
                    ++result[0];
                else if(*it > upper)
                    ++result[bins-1];
                else
                    ++result[size_t((*it - lower) * inc)];
            }
        }
        
        template<typename RealT>
        void rhist(const typename std::vector<RealT>& data,
                size_t bins,
                RealT* result,
                RealT lower,
                RealT upper) {
            // Sanity check
            COVIS_ASSERT(bins > 0);

            // Special case
            if(data.empty()) {
                for(size_t i = 0; i < bins; ++i)
                    result[i] = RealT(0);
                
                return;
            }
            
            // Compute absolute histogram
            size_t histabs[bins];
            hist<RealT>(data, bins, histabs, lower, upper);
            
            // Compute relative histogram
            const RealT sizef(data.size());
            for(size_t i = 0; i < bins; ++i)
                result[i] = RealT(histabs[i]) / sizef;
        }
        
        template<typename RealT>
        void hist2(const typename std::vector<RealT>& data1,
                const typename std::vector<RealT>& data2,
                size_t bins1,
                size_t bins2,
                size_t* result,
                RealT lower1,
                RealT upper1,
                RealT lower2,
                RealT upper2) {
            // Sanity check
            COVIS_ASSERT(data1.size() == data2.size());
            COVIS_ASSERT(bins1 > 0 && bins2 > 0);
            const size_t bins = bins1 * bins2;
            
            // Initialize result to zero
            std::memset(result, 0, bins * sizeof(size_t));
            
            // Increments
            const RealT inc1 = RealT(bins1) / (upper1 - lower1);
            const RealT inc2 = RealT(bins2) / (upper2 - lower2);
            
            // Compute absolute 2D histogram
            for(size_t i = 0; i < data1.size(); ++i) {
                size_t idx1, idx2;
                
                const float v1 = data1[i];
                if(v1 < lower1)
                    idx1 = 0;
                else if(v1 > upper1)
                    idx1 = bins1 - 1;
                else
                    idx1 = size_t((v1 - lower1) * inc1);
                
                const float v2 = data2[i];
                if(v2 < lower2)
                    idx2 = 0;
                else if(v2 > upper2)
                    idx2 = bins2 - 2;
                else
                    idx2 = size_t((v2 - lower2) * inc2);
                
                // Row-major storage: stride equal to second dimension
                ++result[idx1 * bins2 + idx2];
            }
        }
        
        template<typename RealT>
        void rhist2(const typename std::vector<RealT>& data1,
                const typename std::vector<RealT>& data2,
                size_t bins1,
                size_t bins2,
                RealT* result,
                RealT lower1,
                RealT upper1,
                RealT lower2,
                RealT upper2) {
            // Sanity check
            COVIS_ASSERT(data1.size() == data2.size());
            COVIS_ASSERT(bins1 > 0 && bins2 > 0);
                
            // Special case
            if(data1.empty()) {
                for(size_t i = 0; i < bins1 * bins2; ++i)
                    result[i] = RealT(0);
                
                return;
            }
            
            // Compute absolute 2D histogram
            size_t histabs[bins1 * bins2];
            hist2<RealT>(data1, data2, bins1, bins2, histabs, lower1, upper1, lower2, upper2);
            
            // Compute relative 2D histogram
            const RealT sizef(data1.size());
            for(size_t i = 0; i < bins1 * bins2; ++i)
                result[i] = RealT(histabs[i]) / sizef;
        }
    }
}
#endif
