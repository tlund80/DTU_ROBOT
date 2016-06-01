#include "feature_matching_metrics.h"

// Boost
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>

using namespace covis;

std::vector<std::string> mnames;
size_t mnum;

std::string m2str(DISTANCE_METRIC distanceMetric) {
    return mbimap.right.at(distanceMetric);
}

DISTANCE_METRIC str2m(std::string str) {
    boost::algorithm::to_upper(str);
    return mbimap.left.at(str);
}

core::Correspondence::VecPtr match(const MatrixT& featQuery,
        const MatrixT& featTarget,
        DISTANCE_METRIC distanceMetric) {
    core::Correspondence::VecPtr result;
    switch(distanceMetric) {
        case L1:
            result = Matcher<L1>::match(featQuery, featTarget);
            break;
        case L1_RATIO:
            result = Matcher<L1_RATIO>::match(featQuery, featTarget);
            break;
        case L2:
            result = Matcher<L2>::match(featQuery, featTarget);
            break;
        case L2_RATIO:
            result = Matcher<L2_RATIO>::match(featQuery, featTarget);
            break;
        case LINF:
            result = Matcher<LINF>::match(featQuery, featTarget);
            break;
        case LINF_RATIO:
            result = Matcher<LINF_RATIO>::match(featQuery, featTarget);
            break;
        case HELLINGER:
            // Hellinger is just the L2 between the sqrt elements
            result = Matcher<HELLINGER>::match(featQuery.cwiseSqrt(), featTarget.cwiseSqrt());
            break;
        case CHISQ:
            result = Matcher<CHISQ>::match(featQuery, featTarget);
            break;
        case EMD: {
            // Take local copies
            MatrixT featq = featQuery;
            MatrixT featt = featTarget;
            // Compute cumulative sum for each row
            for(int i = 0; i < featq.rows(); ++i)
                std::partial_sum(&featq(i,0), &featq(i,0) + featq.cols(), &featq(i,0));
            for(int i = 0; i < featt.rows(); ++i)
                std::partial_sum(&featt(i,0), &featt(i,0) + featt.cols(), &featt(i,0));
            result = Matcher<EMD>::match(featq, featt);
            break;
        } default:
            COVIS_THROW("Unknown metric");
    }
    
    return result;
}
