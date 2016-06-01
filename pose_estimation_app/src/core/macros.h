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

#ifndef COVIS_CORE_MACROS_H
#define COVIS_CORE_MACROS_H

// Own
#include "traits.h"

// STL
#include <cstdlib>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>

/*
 * Messages
 */

/**
 * @ingroup core
 * @brief Print a message to standard output
 */
#define COVIS_MSG(ostreamexp) std::cout << ostreamexp << '\n'

/**
 * @ingroup core
 * @brief Print an important message to standard output
 */
#define COVIS_MSG_INFO(ostreamexp) COVIS_MSG("\033[0;32m" << ostreamexp << "\033[0m")

/**
 * @ingroup core
 * @brief Print a warning message to standard error output
 */
#define COVIS_MSG_WARN(ostreamexp) COVIS_MSG("\033[0;33m" << ostreamexp << "\033[0m")

/**
 * @ingroup core
 * @brief Print an error message to standard error output
 */
#define COVIS_MSG_ERROR(ostreamexp) COVIS_MSG("\033[1;31m" << ostreamexp << "\033[0m")

/*
 * Logs
 */

/**
 * @ingroup core
 * @brief CoViS log levels
 */
enum COVIS_LOG_LEVEL_TYPE {
    COVIS_LOG_LEVEL_ERROR = 0, /**< Default log level: show only errors */
    COVIS_LOG_LEVEL_WARN, /**< Show warnings and errors */
    COVIS_LOG_LEVEL_INFO, /**< Show important messages, warnings and errors */
    COVIS_LOG_LEVEL_ALL /**< Maximum log level: show all log messages */
};

/**
 * @ingroup core
 * @brief CoViS log level, defaults to \ref COVIS_LOG_LEVEL_ERROR
 */
extern COVIS_LOG_LEVEL_TYPE CovisLogLevel;

/// @cond
namespace covis {
    namespace core {
        void setLogLevel(COVIS_LOG_LEVEL_TYPE level);
        int getLogLevel();
        inline void terminate() {
            abort();
        }
    }
}
/// @endcond

/**
 * @ingroup core
 * @brief Set CoViS log Level
 */
#define COVIS_SET_LOG_LEVEL(level) covis::core::setLogLevel(level);

/**
 * @ingroup core
 * @brief Get the CoViS log level as an integer from 0 (error) to 3 (all)
 */
#define COVIS_GET_LOG_LEVEL() covis::core::getLogLevel()

/**
 * @ingroup core
 * @brief Log an error message
 */
#define COVIS_LOG_ERROR(ostreamexp)\
if(COVIS_GET_LOG_LEVEL() >= COVIS_LOG_LEVEL_ERROR) {\
    COVIS_MSG_ERROR(ostreamexp);\
}

/**
 * @ingroup core
 * @brief Log a warning message
 */
#define COVIS_LOG_WARN(ostreamexp)\
if(COVIS_GET_LOG_LEVEL() >= COVIS_LOG_LEVEL_WARN) {\
    COVIS_MSG_WARN(ostreamexp);\
}

/**
 * @ingroup core
 * @brief Log an important info message
 */
#define COVIS_LOG_INFO(ostreamexp)\
if(COVIS_GET_LOG_LEVEL() >= COVIS_LOG_LEVEL_INFO) {\
    COVIS_MSG_INFO(ostreamexp);\
}

/**
 * @ingroup core
 * @brief Log a message, no matter what the log level is
 */
#define COVIS_LOG_ALL(ostreamexp)\
if(COVIS_GET_LOG_LEVEL() >= COVIS_LOG_LEVEL_ALL) {\
    COVIS_MSG(ostreamexp);\
}

/**
 * @ingroup core
 * @copydoc COVIS_LOG_ALL
 */
#define COVIS_LOG(ostreamexp) COVIS_LOG_ALL(ostremexp)

/*
 * Exceptions
 */

/**
 * @ingroup core
 * @brief Throw an exception with a message to standard output
 */
#define COVIS_THROW(ostreamexp)\
do {\
    std::stringstream tmp;\
    tmp << __FILE__ << ":" << __LINE__ << ": " << ostreamexp;\
    COVIS_MSG_ERROR(tmp.str());\
    std::set_terminate(covis::core::terminate);\
    throw std::runtime_error(tmp.str());\
} while(false)

/*
 * Assertions
 */

/**
 * @ingroup core
 * @brief Assert a condition is met, otherwise throw an exception with a specified message
 */
#define COVIS_ASSERT_MSG(condition, ostreamexp) if(!(condition)) COVIS_THROW(ostreamexp)

/**
 * @ingroup core
 * @brief Assert a condition is met, otherwise throw an exception with a standard message
 */
#define COVIS_ASSERT(condition) COVIS_ASSERT_MSG(condition, "assertion '" << #condition << "' failed!")

#ifdef NDEBUG

/**
 * @ingroup core
 * @brief (Empty during release builds)
 */
#define COVIS_ASSERT_DBG(condition)

/**
 * @ingroup core
 * @brief (Empty during release builds)
 */
#define COVIS_ASSERT_DBG_MSG(condition, ostreamexp)

#else // NDEBUG

/**
 * @ingroup core
 * @brief Assert a condition is met, otherwise throw an exception with a standard message
 */
#define COVIS_ASSERT_DBG(condition) COVIS_ASSERT(condition)

/**
 * @ingroup core
 * @brief Assert a condition is met, otherwise throw an exception with a specified message
 */
#define COVIS_ASSERT_DBG_MSG(condition, ostreamexp) COVIS_ASSERT_MSG(condition, ostreamexp)

#endif // NDEBUG

/*
 * Traits
 */

/**
 * @ingroup core
 * @brief This macro causes a compiler macro if a PCL point type does not contain a specified field
 * 
 * Example:
 * @code
 * COVIS_STATIC_ASSERT_PCL_FIELD(pcl::PointXYZ, rgba, RGB); // Causes an error message about missing RGB information
 * @endcode
 * 
 * @param PointT PCL point type
 * @param Field PCL field without the namespace qualifiers, e.g. x
 * @param FieldName descriptive name of the field, e.g. X
 * @sa @ref covis::core::PCLFieldCheck "PCLFieldCheck"
 */
#define COVIS_STATIC_ASSERT_PCL_FIELD(PointT, Field, FieldName)\
    BOOST_MPL_ASSERT_MSG((covis::core::PCLFieldCheck<PointT, pcl::fields::Field>::value),\
            PCL_FIELD_##FieldName##_MISSING,\
            (PointT))

/**
 * @ingroup core
 * @brief This macro causes a compiler macro if a PCL point type does not contain XYZ information
 * @param PointT PCL point type
 * @sa @ref covis::core::HasXYZ "HasXYZ"
 */
#define COVIS_STATIC_ASSERT_XYZ(PointT) \
    BOOST_MPL_ASSERT_MSG(covis::core::HasXYZ<PointT>::value, PCL_FIELD_XYZ_MISSING, (PointT))

/**
 * @ingroup core
 * @brief This macro causes a compiler macro if a PCL point type does not contain normal information
 * @param PointT PCL point type
 * @sa @ref covis::core::HasNormal "HasNormal"
 */
#define COVIS_STATIC_ASSERT_NORMAL(PointT) \
        BOOST_MPL_ASSERT_MSG(covis::core::HasNormal<PointT>::value, PCL_FIELD_NORMAL_MISSING, (PointT))

/**
 * @ingroup core
 * @brief This macro causes a compiler macro if a PCL point type does not contain RF information
 * @param PointT PCL point type
 * @sa @ref covis::core::HasRF "HasRF"
 */
#define COVIS_STATIC_ASSERT_RF(PointT) \
        BOOST_MPL_ASSERT_MSG(covis::core::HasRF<PointT>::value, PCL_FIELD_RF_MISSING, (PointT))

/**
 * @ingroup core
 * @brief This macro causes a compiler macro if a PCL point type does not contain RGB(A) information
 * @param PointT PCL point type
 * @sa @ref covis::core::HasRGB "HasRGB"
 */
#define COVIS_STATIC_ASSERT_RGB(PointT) \
        BOOST_MPL_ASSERT_MSG(covis::core::HasRGB<PointT>::value, PCL_FIELD_RGB_MISSING, (PointT))

/**
 * @ingroup core
 * @brief This macro causes a compiler macro if a PCL point type does not contain intrinsic dimensionality
 * @param PointT PCL point type
 * @sa @ref covis::core::HasID "HasID"
 */
#define COVIS_STATIC_ASSERT_ID(PointT) \
        BOOST_MPL_ASSERT_MSG(covis::core::HasID<PointT>::value, PCL_FIELD_ID_MISSING, (PointT))

/**
 * @ingroup core
 * @brief This macro causes a compiler macro if a PCL point type does not contain scale
 * @param PointT PCL point type
 * @sa @ref covis::core::HasScale "HasScale"
 */
#define COVIS_STATIC_ASSERT_SCALE(PointT) \
        BOOST_MPL_ASSERT_MSG(covis::core::HasScale<PointT>::value, PCL_FIELD_SCALE_MISSING, (PointT))

#endif
