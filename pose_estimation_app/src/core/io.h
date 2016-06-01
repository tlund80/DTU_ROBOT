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

#ifndef COVIS_CORE_IO_H
#define COVIS_CORE_IO_H

// Eigen
#include <eigen3/Eigen/Core>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @brief Stringify a value
         * @param value input value
         * @return string representation of value
         */
        template<typename T>
        std::string stringify(const T& value);
        
        /**
         * @ingroup core
         * @brief Stringify a floating point value with specified precision
         * @param value input value
         * @param precision number of digits
         * @return string representation of value
         */
        template<typename RealT>
        std::string stringify(const RealT& value, size_t precision);
        
        /**
         * @ingroup core
         * @brief Read an Eigen matrix of any type from a raw data file
         * @note This function reads the number of elements in the matrix object, so this has to be initialized,
         * otherwise a warning message will be printed beforehand. Using e.g. @b Eigen::Matrix4f already provides an
         * initialization of 16 elements.
         * @note The data file cannot contain anything else than numeric information
         * @param filename input file name
         * @param m output matrix
         * @param rowMajor if true (default), assume file data is stored in row-major format
         * @param binary if true, read from a binary file
         * @exception an exception is thrown if the file fails to read
         */
        template<typename Derived>
        void read(const std::string& filename,
                Eigen::MatrixBase<Derived>& m,
                bool rowMajor = true,
                bool binary = false);

        /**
         * @ingroup core
         * @brief Write an Eigen matrix of any type to a raw data file
         * @param filename output file name
         * @param m input matrix
         * @param rowMajor if true (default), write data in row-major format
         * @param binary if true, write to a binary file
         * @param append if true, append data to an existing file
         * @exception an exception is thrown if the file fails to write
         */
        template<typename Derived>
        void write(const std::string& filename,
                const Eigen::MatrixBase<Derived>& m,
                bool rowMajor = true,
                bool binary = false,
                bool append = false);
    }
}

#include "io_impl.hpp"

#endif
