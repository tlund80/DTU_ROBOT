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

#ifndef COVIS_CORE_RANDOM_IMPL_HPP
#define COVIS_CORE_RANDOM_IMPL_HPP

namespace covis {
    namespace core {
        template<typename IntT>
        std::vector<IntT> randidx(IntT size, IntT n) {
            if(n > size)
                COVIS_THROW("Cannot sample " << n << " samples out of a population of " << size << "!");
            else if(n == size)
                return range(size);
            
            std::vector<IntT> sampleIndices(n);
            IntT tempSample;

            // Draw random samples until n samples is reached
            for(IntT i = 0; i < n; i++) {
                // Select a random number
                sampleIndices[i] = randidx<IntT>(size - i);
                
                // Run trough list of numbers, starting at the lowest, to avoid duplicates
                for(IntT j = 0; j < i; j++) {
                    // Move value up if it is higher than previous selections to ensure true randomness
                    if(sampleIndices[i] >= sampleIndices[j]) {
                        sampleIndices[i]++;
                    } else {
                        // The new number is lower, place it at the correct point and break for a sorted list
                        tempSample = sampleIndices[i];
                        for(IntT k = i; k > j; k--)
                            sampleIndices[k] = sampleIndices[k-1];
                        
                        sampleIndices[j] = tempSample;
                        
                        break;
                    }
                }
            }
            
            return sampleIndices;
        }
    }
}
#endif
