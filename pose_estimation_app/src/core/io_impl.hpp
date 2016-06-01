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

#ifndef COVIS_CORE_IO_IMPL_HPP
#define COVIS_CORE_IO_IMPL_HPP

// Own
#include "macros.h"

// STL
#include <fstream>
#include <sstream>

// Eigen
#include <Eigen/Core>

namespace covis {
    namespace core {
        template<typename T>
        std::string stringify(const T& value) {
            std::ostringstream oss;
            oss << value;
            return oss.str();
        }

        template<typename RealT>
        std::string stringify(const RealT& value, size_t precision) {
            std::ostringstream oss;
            oss.precision(precision);
            oss << value;
            return oss.str();
        }
        
        template<typename Derived>
        void read(const std::string& filename, Eigen::MatrixBase<Derived>& m, bool rowMajor, bool binary) {
            // Sanity checks
            if(m.rows() == 0 || m.cols() == 0) {
                COVIS_MSG_WARN("Output matrix dimensions not initialized - returning zero matrix!");
                return;
            }
            
            // Open file
            std::ifstream ifs(filename.c_str(), (binary ? std::ios::binary : std::ios::in));
            if(!ifs)
                COVIS_THROW("Cannot open file \"" << filename << "\" for reading!");
            
            // Read
            typedef typename Eigen::MatrixBase<Derived>::Scalar Scalar;
            for(typename Eigen::MatrixBase<Derived>::Index r = 0; r < m.rows(); ++r) {
                for(typename Eigen::MatrixBase<Derived>::Index c = 0; c < m.cols(); ++c) {
                    if(rowMajor) {
                        if(binary)
                            ifs.read(reinterpret_cast<char*>(&m.derived()(r, c)), sizeof(Scalar));
                        else
                            ifs >> m.derived()(r,c);
                    } else {
                        if(binary)
                            ifs.read(reinterpret_cast<char*>(&m.derived()(c, r)), sizeof(Scalar));
                        else
                            ifs >> m.derived()(c,r);
                    }
                    
                    if(!ifs)
                        COVIS_THROW("Failed to read matrix from file \"" << filename << "\"! " <<
                                (r + 1) * (c + 1) - 1 << " element(s) have been found, but matrix size is " <<
                                m.rows() * m.cols() << "!");
                }
            }
        }

        template<typename Derived>
        void write(const std::string& filename,
                const Eigen::MatrixBase<Derived>& m,
                bool rowMajor,
                bool binary,
                bool append) {
            // Sanity checks
            if(m.rows() == 0 || m.cols() == 0) {
                COVIS_MSG_WARN("Input matrix dimensions not initialized!");
                return;
            }

            // Open file
            std::ofstream ofs;
            ofs.open(filename.c_str(),
                    (binary ? std::ios::binary : std::ios::out) | (append ? std::ios::app : std::ios::out));
            if(!ofs)
                COVIS_THROW("Cannot open file \"" << filename << "\" for writing!");

            // Write
            typedef typename Eigen::MatrixBase<Derived>::Scalar Scalar;
            for(typename Eigen::MatrixBase<Derived>::Index r = 0; r < m.rows(); ++r) {
                for(typename Eigen::MatrixBase<Derived>::Index c = 0; c < m.cols(); ++c) {
                    if(rowMajor) {
                        if(binary)
                            ofs.write(reinterpret_cast<const char*>(&m.derived()(r,c)), sizeof(Scalar));
                        else
                            ofs << m.derived()(r,c) << " ";
                    } else {
                        if(binary)
                            ofs.write(reinterpret_cast<const char*>(&m.derived()(c, r)), sizeof(Scalar));
                        else
                            ofs << m.derived()(c,r) << " ";
                    }

                    if(!ofs)
                        COVIS_THROW("Failed to write matrix to file \"" << filename << "\"!");
                }
                if(!binary)
                    ofs << std::endl;
            }
        }
    }
}

#endif
