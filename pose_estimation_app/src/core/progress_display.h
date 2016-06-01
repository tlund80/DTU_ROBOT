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


#ifndef COVIS_CORE_PROGRESS_DISPLAY_H
#define COVIS_CORE_PROGRESS_DISPLAY_H

// Boost
#include <boost/shared_ptr.hpp>

// STL
#include <cstdio>

namespace covis {
    namespace core {
        /**
         * @class ProgressDisplay
         * @ingroup core
         * @brief Show the progress of a time consuming loop in the terminal
         * 
         * This class works much like the progress display of Boost. To show the process of a loop:
         * @code
         * size_t iterations = 12345;
         * core::ProgressDisplay pd(12345);
         * for(size_t i = 0; i < iterations; ++i, ++pd)
         *     doSomething();
         * @endcode
         * 
         * @author Anders Glent Buch
         */
        class ProgressDisplay {
            public:
                /// Pointer type
                typedef boost::shared_ptr<ProgressDisplay> Ptr;
                
                /**
                 * Constructor: set number of iterations
                 * @param iter number of iterations
                 * @param abs if set to true, absolute iteration is counted instead of percent
                 */
                ProgressDisplay(size_t iter, bool abs = false) : _iter(iter), _abs(abs), _i(0), _cnt(0), _done(false) {
                    if (_abs)
                        operator++();
                    else
                        operator+=(0);
                }
                
                /**
                 * Destructor: print a newline if 100 % has not been reached
                 */
                virtual ~ProgressDisplay() {
                    if (!_done) {
                        printf("\n");
                        fflush(stdout);
                    }
                }
                
                /**
                 * Increment operator
                 * @param inc increment
                 * @return reference to \b this
                 */
                inline ProgressDisplay& operator+=(unsigned int inc) {
                    if (!_done) {
                        // Delete current line
                        for (int i = 0; i < _cnt; ++i)
                            printf("\b");
                        
                        // Increment
                        _i += inc;
                        
                        if (_abs) {
                            // Check if we are done
                            if (_i >= _iter) {
                                _done = true;
                                printf("> %lu/%lu\n", _iter, _iter);
                            } else {
                                // Write progress
                                _cnt = printf("> %lu/%lu", _i, _iter);
                            }
                        } else {
                            // Get progress in %
                            const size_t pct( 100.0 * double(_i) / double(_iter - 1) + 0.5 );
                            
                            // Check if we are done
                            if (pct >= 100) {
                                _done = true;
                                printf("> 100 %%\n");
                            } else {
                                // Write progress
                                _cnt = printf("> %lu %%", pct);
                            }
                        }
                        
                        // Flush
                        fflush(stdout);
                    }
                    
                    return *this;
                }
                
                /**
                 * Prefix increment
                 * @return reference to \b this
                 */
                inline ProgressDisplay& operator++() {
                    return operator+=(1);
                }
                
                /**
                 * Postfix increment
                 * @return the state of \b this before increment
                 */
                inline ProgressDisplay operator++(int) {
                    // Get current state
                    const ProgressDisplay pre = *this;
                    
                    // Increment
                    operator++();
                    
                    return pre;
                }
                
            private:
                /// Empty constructor
                ProgressDisplay();
                
                /// Number of iterations
                size_t _iter;
                
                /// True for absolute iteration count printing
                bool _abs;
                
                /// Current iteration
                size_t _i;
                
                /// Number of printed characters
                int _cnt;
                
                /// True when final iteration has been reached
                bool _done;
        };
    }
}

#endif