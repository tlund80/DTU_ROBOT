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

#include "detection.h"

std::ostream& covis::core::operator<<(std::ostream& os, const Detection& d) {
    if (d.label.empty())
        os << "Object " << d.idx << " detected" << std::endl;
    else
        os << "Object \"" << d.label << "\" detected" << std::endl;
    
    if (d.rmse < std::numeric_limits<float>::max())
        os << "\tRMSE: " << d.rmse << std::endl;
    
    if (d.penalty < std::numeric_limits<float>::max())
        os << "\tPenalty: " << d.penalty << std::endl;
    
    if (d.inlierfrac > 0.0f)
        os << "\tInlier fraction: " << d.inlierfrac << std::endl;
    
    if ( !d.params.empty() ) {
        os << "\tNamed parameters:" << std::endl;
        for (std::map<std::string, float>::const_iterator it = d.params.begin(); it != d.params.end(); ++it)
            os << "\t\t" << it->first << ": " << it->second << std::endl;
    }
    
    COVIS_ASSERT(d.pose.rows() == 4 && d.pose.cols() == 4);
    os << "\tPose:" << std::endl;
    os << "\t\t          |" << d.pose(0, 0) << " " << d.pose(0, 1) << " " << d.pose(0, 2) << "|\n";
    os << "\t\tRotation: |" << d.pose(1, 0) << " " << d.pose(1, 1) << " " << d.pose(1, 2) << "|\n";
    os << "\t\t          |" << d.pose(2, 0) << " " << d.pose(2, 1) << " " << d.pose(2, 2) << "|\n";
    
    os << "\t\tTranslation: [" << d.pose(0, 3) << " " << d.pose(1, 3) << " " << d.pose(2, 3) << "]\n";
    
    return os;
}