#include "feature_matching_features.h"
#include "feature_matching_tools.h"

// Own
#include "rops_estimation_fix.h"
#include "shot_lrf_mod.h"

// PCL
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/spin_image.h>

//Global descriptors
//#include <pcl/features/cvfh.h> //Clustered Viewpoint Feature Histogram
//#include <pcl/features/vfh.h> //Viewpoint Feature Histogram
//#include <pcl/features/esf.h> //Ensemble of Shape Functions
//#include <pcl/features/grsd.h> //Global Radius-based Surface Descriptors
//#include <pcl/features/our_cvfh.h> //Oriented, Unique and Repeatable Clustered Viewpoint Feature Histogram
//#include <pcl/features/crh.h> //Camera Roll Histogram


using namespace covis;

std::vector<std::string> fnames;
size_t fnum;

void features(pcl::PolygonMesh::ConstPtr mesh,
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
    
    if(HAS_FEATURE("ecsad")) {
        const size_t fidx = FEATURE_IDX("ecsad");
        COVIS_MSG("\t" << "- ecsad");
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

    if(HAS_FEATURE("fpfh")) {
        const size_t fidx = FEATURE_IDX("fpfh");
        COVIS_MSG("\t" << "- fpfh");
        
        pcl::PointCloud<FpfhT> fpfh;
        pcl::FPFHEstimationOMP<PointT,PointT,FpfhT> est;
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

    if(HAS_FEATURE("ndhist")) {
        const size_t fidx = FEATURE_IDX("ndhist");
        COVIS_MSG("\t" << "- ndhist");
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

    if(HAS_FEATURE("rops")) {
        const size_t fidx = FEATURE_IDX("rops");
        COVIS_MSG("\t" << "- rops");
        pcl::PointCloud<RopsT> rops;
        pcl::ROPSEstimationFix<PointT,RopsT> est;
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

    if(HAS_FEATURE("ropsmod")) {
        const size_t fidx = FEATURE_IDX("ropsmod");
        COVIS_MSG("\t" << "- ropsmod");
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

    if(HAS_FEATURE("shot")) {
        const size_t fidx = FEATURE_IDX("shot");
        COVIS_MSG("\t" << "- shot");
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

    if(HAS_FEATURE("si")) {
        const size_t fidx = FEATURE_IDX("si");
        COVIS_MSG("\t" << "- si");
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
    if(HAS_FEATURE("usc")) {
        const size_t fidx = FEATURE_IDX("usc");
        COVIS_MSG("\t" << "- usc");
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
    
     if(HAS_FEATURE("pfh")) {
     const size_t fidx = FEATURE_IDX("pfh");
        COVIS_MSG("\t" << "- pfh");
        pcl::PointCloud<PfhT> pfh;
        pcl::PFHEstimation<PointT,PointT,PfhT> est;
        est.setRadiusSearch(radius[fidx]);
        est.setSearchSurface(surf);
        est.setInputNormals(surf);
        est.setInputCloud(query);

        TIME(\
                est.compute(pfh);,
                timings[fidx]
        );
        COVIS_ASSERT(pfh.size() == query->size());
        
        feat2mat(pfh, feat[fidx]);
      
    }
    
    //Global descriptor
  /*  if(HAS_FEATURE("cvfh")) {
     const size_t fidx = FEATURE_IDX("cvfh");
        
        pcl::PointCloud<CvfhT> cvfh;
        pcl::CVFHEstimation<PointT,PointT,CvfhT> est;
        est.setRadiusSearch(radius[fidx]);
	est.setKSearch(0); //Need to set K=0 to not crash
	est.setNormalizeBins(false); //false by default
	//est.setClusterTolerance(5*radius[fidx]);
	//est.setViewPoint(0,0,0);
        est.setSearchSurface(surf);
        est.setInputNormals(surf);
        est.setInputCloud(query);

        TIME(\
                est.compute(cvfh);,
                timings[fidx]
        );
	std::cout <<  "cvfh.size() = " << cvfh.size() <<  " query->size() =" << query->size() << std::endl;
        COVIS_ASSERT(cvfh.size() == query->size());
        
        feat2mat(cvfh, feat[fidx]);
      
    }
    */
    
      if(HAS_FEATURE("3dsc")) {
        const size_t fidx = FEATURE_IDX("3dsc");
        COVIS_MSG("\t" << "- 3dsc");
        pcl::PointCloud<DSCT> dsc;
        pcl::ShapeContext3DEstimation <PointT,PointT, DSCT> est;
        est.setRadiusSearch(radius[fidx]); // 20*mr in paper
        est.setMinimalRadius(0.5 * radius[fidx]); // 0.1*max_radius[fidx] in paper
        //est.setLocalRadius(radius[fidx]); // RF radius[fidx], 20*mr in paper
        est.setPointDensityRadius(4.0 * resolution); // 2*mr in paper
        est.setSearchSurface(surf);
        est.setInputCloud(query);
	est.setInputNormals(surf);
        
        TIME(\
                est.compute(dsc);,
                timings[fidx]
        );
        COVIS_ASSERT(dsc.size() == query->size());
        
        feat2mat(dsc, feat[fidx]);
       
    }
    
}
