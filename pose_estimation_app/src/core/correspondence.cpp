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

#include "correspondence.h"

// STL
#include <fstream>

std::ostream& covis::core::operator<<(std::ostream& os, const Correspondence& corr) {
    const size_t size = corr.size();
    if (size == 0) {
        os << "(Empty correspondence)";
    } else {
        os << "Correspondence of size " << size << ":" << std::endl;
        
        os << "\tQuery: " << corr.query << std::endl;
        
        if (size == 1) {
            os << "\tMatch (index, distance): (" << corr.match[0] << ", " << corr.distance[0] << ")";
        } else {
            os << "\tMatches (index, distance):" << std::endl;
            for (size_t i = 0; i < size - 1; ++i)
                os << "\t\t(" << corr.match[i] << ", " << corr.distance[i] << ")" << std::endl;
            os << "\t\t(" << corr.match[size-1] << ", " << corr.distance[size-1] << ")";
        }
    }
    
    return os;
}

void covis::core::save(const std::string& filename, const Correspondence::Vec& corr) {
    std::ofstream ofs(filename.c_str());
    for(Correspondence::Vec::const_iterator it = corr.begin(); it != corr.end(); ++it) {
        COVIS_ASSERT(ofs);
        ofs << it->size() << " " << it->query << " ";
        for(size_t i = 0; i < it->size(); ++i)
            ofs << it->match[i] << " ";
        for(size_t i = 0; i < it->size(); ++i)
            ofs << it->distance[i] << " ";
        ofs << std::endl;
    }
    COVIS_ASSERT(ofs);
}

void covis::core::load(const std::string& filename, Correspondence::Vec& corr) {
    corr.clear();
    std::ifstream ifs(filename.c_str());
    while(true) {
        COVIS_ASSERT(ifs);
        size_t size;
        ifs >> size;
        Correspondence c(size);
        ifs >> c.query;
        for(size_t i = 0; i < size; ++i)
            ifs >> c.match[i];
        for(size_t i = 0; i < size; ++i)
            ifs >> c.distance[i];
        if(ifs.eof())
            break;
        else
            corr.push_back(c);
    }
}
