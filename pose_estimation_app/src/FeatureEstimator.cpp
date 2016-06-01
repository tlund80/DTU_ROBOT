#include "FeatureEstimator.h"

//#include "feature_matching_tools.h"

// Own
//#include "rops_estimation_fix.h"
//#include "shot_lrf_mod.h"

// PCL
#include <pcl/features/3dsc.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/usc.h>
#include <pcl/features/rops_estimation.h>

using namespace covis;

template<typename FeatureT>
FeatureEstimator<FeatureT>::FeatureEstimator()
{

}

template<typename FeatureT>
void FeatureEstimator<FeatureT>::features(pcl::PolygonMesh::ConstPtr mesh,
        CloudT::ConstPtr surf,
        CloudT::ConstPtr query,
        const std::vector<double>& radius,
        double resolution,
        std::vector<MatrixT>& feat,
        Eigen::VectorXf& timings) {
    // Sanity check
    COVIS_ASSERT(radius.size() == fnum);

    // Resize output vector
    if(feat.size() != fnum)
        feat.resize(fnum);

    if(_feature = ECSAD) {
        const size_t fidx = size_t(ECSAD);// FEATURE_IDX("ecsad");

        pcl::PointCloud<EcsadT> ecsad;
        feature::ECSAD<PointT> est;
        est.setRadius(radius[fidx]);
        est.setSurface(surf);

        TIME(\
                ecsad = *est.compute(query);,
                timings[fidx]
        );
        COVIS_ASSERT(ecsad.size() == query->size());

        feat2mat(ecsad, feat[fidx]);
    }

    if(_feature = FPFH) {
        const size_t fidx = size_t(FPFH);//FEATURE_IDX("fpfh");

        pcl::PointCloud<FpfhT> fpfh;
        pcl::FPFHEstimation<PointT,PointT,FpfhT> est;
        est.setRadiusSearch(radius[fidx]);
        est.setSearchSurface(surf);
        est.setInputNormals(surf);
        est.setInputCloud(query);

        TIME(\
                est.compute(fpfh);,
                timings[fidx]
        );
        COVIS_ASSERT(fpfh.size() == query->size());

        feat2mat(fpfh, feat[fidx]);
    }

    if(_feature = NDHIST) {
        const size_t fidx = size_t(NDHIST);//FEATURE_IDX("ndhist");

        pcl::PointCloud<NDHist> ndhist;
        feature::DistanceNormalHistogram<PointT,8,16> est;
        est.setRadius(radius[fidx]);
        est.setSkipNegatives(true);
        est.setSurface(surf);

        TIME(\
                ndhist = *est.compute(query);,
                timings[fidx]
        );
        COVIS_ASSERT(ndhist.size() == query->size());

        feat2mat(ndhist, feat[fidx]);

    }

    if(_feature = ROPS) {
        const size_t fidx = size_t(ROPS);//FEATURE_IDX("rops");

        pcl::PointCloud<RopsT> rops;
        //pcl::ROPSEstimationFix<PointT,RopsT> est;
        pcl::ROPSEstimation<PointT,RopsT> est;
        est.setRadiusSearch(radius[fidx]);
        est.setSupportRadius(radius[fidx]);
        est.setSearchSurface(surf);
        est.setTriangles(mesh->polygons);
        est.setInputCloud(query);

        TIME(\
                est.compute(rops);,
                timings[fidx]
        );
        COVIS_ASSERT(rops.size() == query->size());

        feat2mat(rops, feat[fidx]);
    }

  /*  if(HAS_FEATURE("ropsmod")) {
        const size_t fidx = FEATURE_IDX("ropsmod");

        pcl::PointCloud<pcl::ReferenceFrame>::Ptr frames(new pcl::PointCloud<pcl::ReferenceFrame>);
        pcl::SHOTLocalReferenceFrameEstimationMod<PointT> lrfmod;
        lrfmod.setRadiusSearch(radius[fidx]);
        lrfmod.setSearchSurface(surf);
        lrfmod.setInputCloud(query);

        pcl::PointCloud<RopsT> rops;
        pcl::ROPSEstimationFix<PointT,RopsT> est;
        est.setRadiusSearch(radius[fidx]);
        est.setSupportRadius(radius[fidx]);
        est.setSearchSurface(surf);
        est.setTriangles(mesh->polygons);
        est.setInputCloud(query);

        TIME(\
                lrfmod.compute(*frames);

                est.setInputReferenceFrames(frames);
                est.compute(rops);,
                timings[fidx]
        );
        COVIS_ASSERT(rops.size() == query->size());

        feat2mat(rops, feat[fidx]);
    }
    */

    if(_feature = SHOT) {
        const size_t fidx = size_t(SHOT);//FEATURE_IDX("shot");

        pcl::PointCloud<ShotT> shot;
        pcl::SHOTEstimation<PointT,PointT,ShotT> est;
        est.setRadiusSearch(radius[fidx]);
        est.setSearchSurface(surf);
        est.setInputNormals(surf);
        est.setInputCloud(query);

        TIME(\
                est.compute(shot);,
                timings[fidx]
        );
        COVIS_ASSERT(shot.size() == query->size());

        feat2mat(shot, feat[fidx]);

        // SHOT produces NaNs during normalization if there are no entries inside descriptor
        for(int r = 0; r < feat[fidx].rows(); ++r)
            for(int c = 0; c < feat[fidx].cols(); ++c)
                if(std::isnan(feat[fidx](r,c)))
                    feat[fidx](r,c) = 0.0f;
    }


    if(_feature = SHOTCOLOR) {
        const size_t fidx = size_t(SHOTCOLOR);//FEATURE_IDX("shot");

        pcl::PointCloud<ShotColorT> shotcolor;
        pcl::SHOTColorEstimation<PointT,PointT,ShotColorT> est;
        est.setRadiusSearch(radius[fidx]);
        est.setSearchSurface(surf);
        est.setInputNormals(surf);
        est.setInputCloud(query);

        TIME(\
                est.compute(shotcolor);,
                timings[fidx]
        );
        COVIS_ASSERT(shotcolor.size() == query->size());

        feat2mat(shotcolor, feat[fidx]);

        // SHOT produces NaNs during normalization if there are no entries inside descriptor
        for(int r = 0; r < feat[fidx].rows(); ++r)
            for(int c = 0; c < feat[fidx].cols(); ++c)
                if(std::isnan(feat[fidx](r,c)))
                    feat[fidx](r,c) = 0.0f;
    }

    if(_feature = SPINIMAGES) {
        const size_t fidx = size_t(SPINIMAGES);//FEATURE_IDX("si");

        pcl::PointCloud<SpinT> si;
        pcl::SpinImageEstimation<PointT,PointT,SpinT> est;
        est.setRadiusSearch(radius[fidx]);
        est.setSearchSurface(surf);
        est.setInputNormals(query); // SI needs only normals at feature points
        est.setInputCloud(query);

        TIME(\
                est.compute(si);,
                timings[fidx]
        );
        COVIS_ASSERT(si.size() == query->size());

        feat2mat(si, feat[fidx]);
    }

    if(_feature = USC) {
        const size_t fidx = size_t(USC);//FEATURE_IDX("usc");

        pcl::PointCloud<UscT> usc;
        pcl::UniqueShapeContext<PointT,UscT> est;
        est.setRadiusSearch(radius[fidx]); // 20*mr in paper
        est.setMinimalRadius(0.1 * radius[fidx]); // 0.1*max_radius[fidx] in paper
        est.setLocalRadius(radius[fidx]); // RF radius[fidx], 20*mr in paper
        est.setPointDensityRadius(2.0 * resolution); // 2*mr in paper
        est.setSearchSurface(surf);
        est.setInputCloud(query);

        TIME(\
                est.compute(usc);,
                timings[fidx]
        );
        COVIS_ASSERT(usc.size() == query->size());

        feat2mat(usc, feat[fidx]);
    }

}
