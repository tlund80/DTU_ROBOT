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

// Own
#include "range.h"

// Boost
#include <boost/regex.hpp>
#include <boost/type_traits/is_same.hpp>

// STL
#include <set>

// Algorithm copied verbatim from here:
// http://web.archive.org/web/20071217040157/http://www.boostcookbook.com/Recipe:/1235053
namespace {
    template< typename S >
    struct natural_sort_regex;
    
    template<>
    struct natural_sort_regex< std::string > {
        static const boost::regex regex() {
            return boost::regex( "(\\d+)|([^\\d]+)" );
        }
    };
    template<>
    struct natural_sort_regex< std::wstring > {
        static const boost::wregex regex() {
            return boost::wregex( L"(\\d+)|([^\\d]+)" );
        }
    };
    
    template< typename C = std::less< std::string >, unsigned int padding_length = 10 >
    struct natural_sort : public std::binary_function < typename C::first_argument_type, typename C::first_argument_type, bool > {
    
        BOOST_STATIC_ASSERT( ( boost::is_same< typename C::first_argument_type, typename C::second_argument_type >::value ) );
        BOOST_STATIC_ASSERT( ( boost::is_same< typename C::result_type, bool >::value ) );
    
        typedef C comparator_type;
        typedef typename comparator_type::first_argument_type string_type;
        typedef boost::match_results< typename string_type::const_iterator > match_results;
        typedef boost::regex_iterator< typename string_type::const_iterator > regex_iterator;
        typedef natural_sort_regex< string_type > regex_type;
    
        bool operator()( const string_type &l, const string_type &r ) {
            for( regex_iterator lit( l.begin(), l.end(), regex_type::regex() ), rit( r.begin(), r.end(), regex_type::regex() ), end;
                    lit != end && rit != end; ++lit, ++rit )
                if ( compare( *lit, *rit ) )
                    return true;
                else if ( compare( *rit, *lit ) )
                    return false;
            return false;
        }
    
    private:
        comparator_type comp;
        bool compare( const match_results &l, const match_results &r ) {
            if ( !l[ 1 ].str().empty() && !r[ 1 ].str().empty() )
                return comp( pad( l.str() ), pad( r.str() ) );
            else
                return comp( l.str(), r.str() );
        }
        string_type pad( const string_type &s ) {
            return string_type( padding_length - s.length(), '0' ) + s;
        }
    };
}

void covis::core::natsort(std::vector<std::string>& v, bool ascend) {
    if(ascend) {
        std::set<std::string, natural_sort<std::less<std::string> > > ascending(v.begin(), v.end());
        copy(ascending.begin(), ascending.end(), v.begin());
    } else {
        std::set<std::string, natural_sort<std::greater<std::string> > > descending(v.begin(), v.end());
        copy(descending.begin(), descending.end(), v.begin());
    }
}
