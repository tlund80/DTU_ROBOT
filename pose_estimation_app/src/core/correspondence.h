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

#ifndef COVIS_CORE_CORRESPONDENCE_H
#define COVIS_CORE_CORRESPONDENCE_H

// Own
#include "covis_base.h"
#include "macros.h"

// Boost
#include <boost/shared_ptr.hpp>

// STL
#include <limits>
#include <ostream>

// PCL
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @class Correspondence
         * @brief Structure for representing a one to many correspondence, e.g. between features
         * 
         * This class is an immediate extension of the PCL <b>Correspondence</b> class, which only allows
         * one to one matches.
         * 
         * @author Anders Glent Buch
         */
        struct Correspondence : public CovisBase {
            /// Vector type
            typedef std::vector<Correspondence> Vec;

            /// Pointer to vector
            typedef boost::shared_ptr<Vec> VecPtr;

            /// Pointer to const vector
            typedef boost::shared_ptr<const Vec> VecConstPtr;
            
            /// Index of the query (source) point
            int query;
            
            /// Indices of the matching (target) points
            std::vector<int> match;
            
            /// Distances between corresponding points
            std::vector<float> distance;
            
            /// Empty constructor
            Correspondence() : query(0) {}
            
            /**
             * Construct an empty correspondence
             * @param size number of entries
             */
            Correspondence(size_t size) : query(0), match(size), distance(size) {}
            
            /**
             * Parameter constructor: create a one-to-one correspondence
             * @param query query point
             * @param match matched point
             * @param distance matching distance
             */
            Correspondence(int query, int match, float distance) :
                query(query),
                match(1, match),
                distance(1, distance) {}
            
            /**
             * Parameter constructor: create a one-to-many correspondence
             * @param query query point
             * @param match matched points
             * @param distance matching distances
             */
            Correspondence(int query, const std::vector<int>& match, const std::vector<float>& distance) :
                query(query),
                match(match),
                distance(distance) {}
            
            /**
             * Initialize this from a PCL correspondence
             * @param cpcl PCL correspondence
             */
            Correspondence(const pcl::Correspondence& cpcl) :
                query(cpcl.index_query),
                match(1, cpcl.index_match),
                distance(1, cpcl.distance) {}
            
            /**
             * Cast operator to a PCL correspondence
             * 
             * @warning Since PCL correspondences only support one target match, information is lost if this has many
             * target indices/distances
             */
            inline operator pcl::Correspondence() const {
                if(*this)
                    return pcl::Correspondence(query, match[0], distance[0]);
                else
                    return pcl::Correspondence(query, -1, std::numeric_limits<float>::max());
            }
            
            /**
             * Get number of matches
             * @return size of this
             */
            inline size_t size() const {
                COVIS_ASSERT_DBG(match.size() == distance.size());
                return match.size();
            }
            
            /**
             * Return empty status
             * @return true if size is zero
             */
            inline bool empty() const {
                return size() == 0;
            }
            
            /// Return non-empty status
            inline operator bool() const {
                return size() > 0;
            }
        };

        /**
         * @ingroup core
         * Comparator function for sorting one-to-one correspondences in ascending order
         * @param c1 first correspondence
         * @param c2 second correspondence
         * @return comparison
         */
        inline bool cmpCorrDistAscend(const Correspondence& c1, const Correspondence& c2) {
            COVIS_ASSERT(c1.size() >= 1 && c2.size() >= 1);
            return c1.distance[0] < c2.distance[0];
        }

        /**
         * \ingroup core
         * Sort a set of correspondences in ascending order according to matching distances (front entry only)
         * @param corr correspondences to sort
         */
        inline void sort(Correspondence::Vec& corr) {
           std::sort(corr.begin(), corr.end(), cmpCorrDistAscend);
        }
        
        /**
         * @ingroup core
         * Convert a vector of correspondences to PCL correspondences
         * @warning Since PCL correspondences only support one target match, information is lost if corr has many target
         * indices/distances. If you want to maintain all the information, see @ref covis::core::flatten().
         * 
         * @param corr input correspondences
         * @return PCL one to one correspondences
         */
        inline pcl::CorrespondencesPtr convert(const Correspondence::Vec& corr) {
            pcl::CorrespondencesPtr result(new pcl::Correspondences(corr.size()));
            for(size_t i = 0; i < corr.size(); ++i)
                (*result)[i] = corr[i];
            
            return result;
        }
        
        /**
         * @ingroup core
         * Convert a vector of correspondences to PCL correspondences
         * @note Contrary to @ref covis::core::convert(), this function copies all correspondences per query.
         * Therefore, it is possible that the output vector has a size larger than the input vector.
         * 
         * @param corr input correspondences
         * @return PCL correspondences
         */
        inline pcl::CorrespondencesPtr flatten(const Correspondence::Vec& corr) {
            pcl::CorrespondencesPtr result(new pcl::Correspondences);
            result->reserve(corr.size());
            for(size_t i = 0; i < corr.size(); ++i)
                for(size_t j = 0; j < corr[i].size(); ++j)
                    result->push_back(pcl::Correspondence(corr[i].query, corr[i].match[j], corr[i].distance[j]));
            
            return result;
        }
        
        /**
         * @ingroup core
         * Extract the points indexed by a set of correspondences between two point clouds
         * @note A query/target point is only pushed back once to the output point clouds, even though it may appear
         * several times in the correspondence set
         * @param corr correspondences query --> target
         * @param query query point cloud
         * @param target target point cloud
         * @return a pair of point clouds <query_indexed,target_indexed> taken from the input correspondences' query
         * and target indices, respectively
         */
        template<typename PointT>
        inline std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> 
        extractCorrespondingPoints(const Correspondence::Vec& corr,
                typename pcl::PointCloud<PointT>::ConstPtr query,
                typename pcl::PointCloud<PointT>::ConstPtr target) {
            COVIS_ASSERT(query && target);
            
            // Avoid pushing back points more than once
            std::vector<bool> qmask(query->size(), false);
            std::vector<bool> tmask(target->size(), false);
            
            // Result
            std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> result(
                    new pcl::PointCloud<PointT>, new pcl::PointCloud<PointT>);
            for(size_t i = 0; i < corr.size(); ++i) {
                if(!qmask[corr[i].query]) {
                    result.first->push_back(query->points[corr[i].query]);
                    qmask[corr[i].query] = true;
                }
                
                for(size_t j = 0; j < corr[i].size(); ++j)
                    if(!tmask[corr[i].match[j]]) {
                        result.second->push_back(target->points[corr[i].match[j]]);
                        tmask[corr[i].match[j]] = true;
                    }
            }
            
            return result;
        }

        /**
         * @ingroup core
         * Print a correspondence to a stream
         * @param os stream to print to
         * @param corr correpondence
         * @return modified stream
         */
        std::ostream& operator<<(std::ostream& os, const Correspondence& corr);
        
        /**
         * @ingroup core
         * Save a vector of correspondences to a text file
         * Each line consists of the following entries: size query [matches] [distances]
         * @param filename output file name
         * @param corr correspondences to save
         * @exception an exception is thrown if an I/O operation fails
         */
        void save(const std::string& filename, const Correspondence::Vec& corr);
        
        /**
         * @ingroup core
         * Load a vector of correspondences from a text file
         * @param filename input file name
         * @param corr correspondences to load into
         * @exception an exception is thrown if an I/O operation fails
         */
        void load(const std::string& filename, Correspondence::Vec& corr);
    }
}

#endif
