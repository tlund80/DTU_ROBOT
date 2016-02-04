#ifndef FEATURE_MATCHING_METRICS_H
#define FEATURE_MATCHING_METRICS_H

// Own
#include "feature_matching_features.h"

// Boost
#include <boost/assign/list_of.hpp>
#include <boost/bimap.hpp>

// Covis
#include <covis/core/correspondence.h>

// FLANN
#include <flann/flann.hpp>

// Metric list shared among the compilation units
extern std::vector<std::string> mnames;
extern size_t mnum;

// All metrics
enum DISTANCE_METRIC {
    L1,
    L1_RATIO,
    L2,
    L2_RATIO,
    LINF,
    LINF_RATIO,
    HELLINGER,
    CHISQ,
    EMD
};

// All metric names
const std::string mnamesAll[] = {
    "L1",
    "L1_RATIO",
    "L2",
    "L2_RATIO",
    "LINF",
    "LINF_RATIO",
    "HELLINGER",
    "CHISQ",
    "EMD"
};

typedef boost::bimap<std::string,DISTANCE_METRIC> MetricMap;
const MetricMap mbimap = boost::assign::list_of<MetricMap::relation>
    ("L1", L1)
    ("L1_RATIO", L1_RATIO)
    ("L2", L2)
    ("L2_RATIO", L2_RATIO)
    ("LINF", LINF)
    ("LINF_RATIO", LINF_RATIO)
    ("HELLINGER", HELLINGER)
    ("CHISQ", CHISQ)
    ("EMD", EMD)
    ;

// Metric to string
std::string m2str(DISTANCE_METRIC distanceMetric);

// String to metric
DISTANCE_METRIC str2m(std::string str);

// Match two sets of descriptors featt --> featq
covis::core::Correspondence::VecPtr match(const MatrixT& featQuery,
        const MatrixT& featTarget,
        DISTANCE_METRIC distanceMetric);

template<int DistanceMetric, typename RealT> struct MetricTraits {};
template<typename RealT> struct MetricTraits<L1,RealT> { typedef typename flann::L1<RealT> FlannMetric; static const bool IsRatio = false; };
template<typename RealT> struct MetricTraits<L1_RATIO,RealT> { typedef typename flann::L1<RealT> FlannMetric; static const bool IsRatio = true; };
template<typename RealT> struct MetricTraits<L2,RealT> { typedef typename flann::L2<RealT> FlannMetric; static const bool IsRatio = false; };
template<typename RealT> struct MetricTraits<L2_RATIO,RealT> { typedef typename flann::L2<RealT> FlannMetric; static const bool IsRatio = true; };
template<typename RealT> struct MetricTraits<LINF,RealT> { typedef typename flann::MaxDistance<RealT> FlannMetric; static const bool IsRatio = false; };
template<typename RealT> struct MetricTraits<LINF_RATIO,RealT> { typedef typename flann::MaxDistance<RealT> FlannMetric; static const bool IsRatio = true; };
template<typename RealT> struct MetricTraits<HELLINGER,RealT> { typedef typename flann::L2<RealT> FlannMetric; static const bool IsRatio = false; }; // See match() in source file
template<typename RealT> struct MetricTraits<CHISQ,RealT> { typedef typename flann::ChiSquareDistance<RealT> FlannMetric; static const bool IsRatio = false; };
template<typename RealT> struct MetricTraits<EMD,RealT> { typedef typename flann::L1<RealT> FlannMetric; static const bool IsRatio = false; }; // See match() in source file

template<int DistanceMetric>
struct Matcher {
    // Promote the typedefs for the distance metric
    typedef typename MetricTraits<DistanceMetric, float>::FlannMetric Metric;
    static const bool IsRatio = MetricTraits<DistanceMetric, float>::IsRatio;
    
    // Multiple randomized trees
    typedef typename flann::KDTreeIndex<Metric> KDTree;
    typedef typename boost::shared_ptr<KDTree> KDTreePtr;
    
    // Create an index with specified number of randomized trees
    static KDTreePtr index(const MatrixT& feat, int trees = 4) {
        flann::Matrix<float> mfeat(const_cast<float*>(feat.data()), feat.rows(), feat.cols());
        KDTreePtr result(new KDTree(mfeat, flann::KDTreeIndexParams(trees)));
        result->buildIndex();
        
        return result;
    }
    
    // Find approximate NNs using an existing index
    static covis::core::Correspondence::VecPtr knn(const KDTree& index, const MatrixT& featQuery, int checks = 512) {
        // NNs
        const int k = (IsRatio ? 2 : 1);
        // Map to FLANN matrix
        flann::Matrix<float> mquery(const_cast<float*>(featQuery.data()), featQuery.rows(), featQuery.cols());
        // Prepare output
        std::vector<std::vector<size_t> > idx;
        std::vector<std::vector<float> > dist;
        // Search
        index.knnSearch(mquery, idx, dist, k, flann::SearchParams(checks));
        // Convert
        covis::core::Correspondence::VecPtr result(
                new covis::core::Correspondence::Vec(mquery.rows, covis::core::Correspondence(1)));
        for(size_t i = 0; i < mquery.rows; ++i) {
            (*result)[i].query = i;
            (*result)[i].match[0] = idx[i][0];
            (*result)[i].distance[0] = (IsRatio ?  dist[i][0] / dist[i][1] : dist[i][0]);
        }
        
        return result;
    }
    
    // Linear matching
    static covis::core::Correspondence::VecPtr match(const MatrixT& featQuery, const MatrixT& featTarget) {
        COVIS_ASSERT(featQuery.cols() == featTarget.cols());
        
        const int k = (IsRatio ? 2 : 1);
        
        flann::Matrix<float> mquery(const_cast<float*>(featQuery.data()), featQuery.rows(), featQuery.cols());
        flann::Matrix<float> mtarget(const_cast<float*>(featTarget.data()), featTarget.rows(), featTarget.cols());
        flann::LinearIndex<typename MetricTraits<DistanceMetric, float>::FlannMetric> index(mtarget);
        
        std::vector<std::vector<size_t> > idx;
        std::vector<std::vector<float> > dist;
        
        index.knnSearch(mquery, idx, dist, k, flann::SearchParams());
        
        covis::core::Correspondence::VecPtr result(
                new covis::core::Correspondence::Vec(mquery.rows, covis::core::Correspondence(1)));
        for(size_t i = 0; i < mquery.rows; ++i) {
            (*result)[i].query = i;
            (*result)[i].match[0] = idx[i][0];
            (*result)[i].distance[0] = (IsRatio ?  dist[i][0] / dist[i][1] : dist[i][0]);
        }
        
        return result;
    }
};

#endif
