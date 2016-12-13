#ifndef FEATURE_MATCHING_FEATURES_H
#define FEATURE_MATCHING_FEATURES_H

// Covis
#include <covis/feature.h>

// Boost
#include <boost/range.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

// Useful types - row-major is extremely important during feature matching
typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixT;

// Feature list shared among the compilation units
extern std::vector<std::string> fnames;
extern size_t fnum;

/*
 * CURRENTLY THE FOLLOWING STRINGS ARE SUPPORTED:
 */
const std::string fnamesAll[] = {
        "ecsad",
         "fpfh",
         "ndhist",
         "rops",
         "shot",
         "si",
         "usc",
                         //  "pfh"
        "3dsc"
};

// Useful macros
#define HAS_FEATURE(feature) (std::find(boost::begin(fnames), boost::end(fnames), feature) != boost::end(fnames))
#define FEATURE_IDX(feature) (std::find(boost::begin(fnames), boost::end(fnames), feature) - boost::begin(fnames))

// Feature types
typedef covis::feature::ECSAD<PointT>::Histogram EcsadT; // Histogram
typedef pcl::FPFHSignature33 FpfhT; // Histogram
typedef covis::feature::DistanceNormalHistogram<PointT,8,16>::Histogram NDHist;// Histogram
typedef pcl::Histogram<135> RopsT; // Histogram
typedef pcl::SHOT352 ShotT; // Descriptor
typedef pcl::Histogram<153> SpinT; // Histogram
//typedef pcl::ShapeContext1980 UscT;// Descriptor
typedef pcl::UniqueShapeContext1960 UscT;// Descriptor
typedef pcl::ShapeContext1980 DSCT;// Descriptor
typedef pcl::PFHSignature125 PfhT;
typedef pcl::VFHSignature308 CvfhT;


template<typename FeatureT>
inline void feat2mat(const typename pcl::PointCloud<FeatureT>& feat, MatrixT& mfeat) {
    COVIS_ASSERT(!feat.empty());
    // Map the feature cloud 
    mfeat = Eigen::Map<const MatrixT, Eigen::Aligned, Eigen::OuterStride<> >(
            reinterpret_cast<float*>(const_cast<FeatureT*>(&feat[0])), // Data pointer
            feat.size(), // Rows
            FeatureT::descriptorSize(), // Columns
            Eigen::OuterStride<>(sizeof(FeatureT) / sizeof(float))); // Stride (e.g. 352 + 9 for SHOT)
}

void features(pcl::PolygonMesh::ConstPtr mesh,
        CloudT::ConstPtr surf,
        CloudT::ConstPtr query,
        const std::vector<double>& radius,
        double resolution,
        std::vector<MatrixT>& feat,
        Eigen::VectorXf& timings);

#endif
