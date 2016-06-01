// CoViS
#include <covis/covis.h>
#include <halcon/halconestimation.h>
using namespace covis;

// Boost
#include <boost/filesystem.hpp>
#include <boost/range.hpp>
namespace fs=boost::filesystem;

// PCL
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/registration/correspondence_rejection_distance.h"
#include "pcl/registration/icp.h"

// OpenMP
#include <omp.h>

// Own
#include "feature_matching_tools.h"
#include "feature_matching_features.h"
#include "feature_matching_metrics.h"

// Meshes, surfaces and feature points for the model database
std::vector<pcl::PolygonMesh::Ptr> meshq;
std::vector<CloudT::Ptr> surfq, query;
double avgModelMeshRes; // Average model mesh resolution
boost::random::mt19937 gen; // Random number generator

/*
 * Main
 */
int main(int argc, const char** argv) {
    /**
     * Get program inputs
     */
    
    // Setup program options
    core::ProgramOptions po;
    
    // Positionals
    po.addPositional("query", "query/object model files (PLY)");
    po.addPositional("target", "scene/target model files (PLY)");
    po.addPositional("pose-directory", "path to pose files, file names are resolved as [pose-directory]/[query-basename]-[target-basename].[pose-suffix]");
    po.addPositional("pose-suffix", "pose file suffix (e.g. 'xf' for the UWA dataset)");
    po.addPositional("output-directory", "path to output directory");
    
    // Features
    po.addOption("features", "all", "select a subset of features - in case of 'all', all features are enabled.");
    po.addOption("decimation", 'd', -1, "if in ]0,1[, perform quadric mesh decimation down to this factor");
    po.addOption("radius", 'r', 15, "feature radii, given as a multiple of the average query mr, one per feature");
    po.addOption("resolution", 'e', 5, "resolution of the feature seed points, given as a multiple of the average query mr");
    po.addOption("resolution-query", -1, "if non-zero, apply another resolution to the query models");
    po.addFlag('c', "pca", "use PCA version of a predefined subset of descriptors");
    po.addOption("pca-variation", 99, "if PCA is enabled, use this variation [%] for selecting the number of components");
    
    // Matching
    po.addOption("threshold", 't', 1, "inlier distance threshold during recognition, given as a multiple of the target mr");
    po.addFlag('f', "fusion", "test all ternary combinations instead of single features");
    
    // Recognition
    po.addOption("ransac-iterations", 1000, "number of RANSAC iterations");
    po.addOption("correspondence-fraction", 1.0, "use this best fraction ]0,1] of feature correspondences during RANSAC");
    po.addOption("inlier-fraction", 0.05, "require this fraction ]0,1] of inliers during RANSAC");
    po.addOption("icp-iterations", 50, "number of ICP iterations");
    po.addOption("translation-tolerance", "maximum allowable translation error in metric units > 0");
    po.addOption("rotation-tolerance", "maximum allowable geodesic rotation error in degrees ]0, 180]");
    
    // General
    po.addOption("pose-separator", 'p', "-", "pose file separator inserted between scene and model file (e.g. '-' for the UWA dataset)");
    po.addFlag('m', "pose-file-model-first", "pose file order, set this flag for query model name first");
    po.addFlag('v', "verbose", "print debug information");
    po.addFlag('i', "visualize", "show alignments");
    po.addFlag('p', "append", "append data to output files");
    po.addFlag('o', "omp", "use OpenMP, if available");
    po.addFlag('u', "dry-run", "don't save any outputs");
    
    // Parse
    if(!po.parse(argc, argv))
        return 1;

    const bool verbose = po.getFlag("verbose");
    
    if(verbose)
        po.print();
    
    //Check if halcon dongle is avalible
  /*  HTuple info;
    try{
       GetSystem("hostids", &info);
    }catch (HOperatorException &e)
    {
        pcl::console::print_error("%s\n", e.ErrorText().Text());
	return 0;
    }
*/
    // Get paths
    const std::vector<std::string> queries = po.getVector("query");
    const std::vector<std::string> targets = po.getVector("target");
    const std::string poseDir = po.getValue("pose-directory");
    const std::string poseSuffix = po.getValue("pose-suffix");
    std::string outputDir = po.getValue("output-directory");
    COVIS_ASSERT(!queries.empty() && !targets.empty());
    COVIS_ASSERT(!poseDir.empty() && !poseSuffix.empty() && !outputDir.empty());

    // Get feature name list
    fnames = po.getVector("features");
    if(fnames.empty() || (fnames.size() == 1 && boost::iequals(fnames[0], "all"))) { // No features provided, or "all"
        fnames = std::vector<std::string>(boost::begin(fnamesAll), boost::end(fnamesAll));
    } else { // Features provided
        for(size_t i = 0; i < fnames.size(); ++i) {
            boost::to_lower(fnames[i]);
            COVIS_ASSERT_MSG(std::find(boost::begin(fnamesAll), boost::end(fnamesAll), fnames[i]) != boost::end(fnamesAll),
                    "Unknown feature: " << fnames[i] << "!");
        }
    }
    fnum = fnames.size();
    
    // Get feature options
    const double decimation = po.getValue<double>("decimation");
    std::vector<double> fradMul = po.getVector<double>("radius");
    if(fradMul.size() == 1 && fnum > 1) {
        COVIS_MSG_WARN("Only one radius specified - applying to all features!");
        fradMul.resize(boost::size(fnamesAll), fradMul[0]);
    }
    std::vector<double> frad = fradMul;
    double fres = po.getValue<double>("resolution");
    COVIS_ASSERT(fres > 0.0);
    double fresQuery = po.getValue<double>("resolution-query");
    if(fresQuery <= 0)
        fresQuery = fres;
    
    // PCA case: remove unsuited features
    const bool pca = po.getFlag("pca");
    const float pcaVariation = po.getValue<float>("pca-variation"); // Variation in %

    core::print(fnames, std::cout, "Testing the following features: [", "]");
    if(pca && verbose)
        COVIS_MSG_WARN("Using the following PCA variation [%]: " << pcaVariation);
    
    double thres = (po.getValue<double>("threshold") <= 0.0 ? 1.0 : po.getValue<double>("threshold"));
    
    const bool fusion = po.getFlag("fusion");
    std::vector<boost::tuple<int,int,int> > fusionCombinations;
    std::vector<std::string> fnamesFusion;
    if(fusion && fnum > 2) {
        if(verbose)
            COVIS_MSG_WARN("Feature fusion enabled! Testing the following combinations:");
        for(int i = 0; i < int(fnum) - 2; ++i) {
            for(int j = i + 1; j < int(fnum) - 1; ++j) {
                for(int k = j + 1; k < int(fnum); ++k) {
                    fusionCombinations.push_back(boost::make_tuple(i,j,k));
                    fnamesFusion.push_back(fnames[i] + "-" + fnames[j] + "-" + fnames[k]);
                    if(verbose)
                        COVIS_MSG("\t" << fnamesFusion.back());
                }
            }
        }
    }
    const size_t fnumFusion = fusionCombinations.size();
    
    // Recognition
    const size_t ransacIterations = po.getValue<size_t>("ransac-iterations");
    const double corrFraction = po.getValue<double>("correspondence-fraction");
    COVIS_ASSERT(corrFraction > 0.0 && corrFraction <= 1.0);
    const double inlierFraction = po.getValue<double>("inlier-fraction");
    COVIS_ASSERT(inlierFraction > 0.0 && inlierFraction <= 1.0);
    const size_t icpIterations = po.getValue<size_t>("icp-iterations");
    COVIS_ASSERT(ransacIterations > 0 && icpIterations > 0);
    const double translationTol = po.getValue<double>("translation-tolerance");
    const double rotationTol = po.getValue<double>("rotation-tolerance");
    COVIS_ASSERT(translationTol > 0.0 && rotationTol > 0.0 && rotationTol <= 180.0);
    
    // General
    const std::string poseSeparator = po.getValue("pose-separator");
    const bool poseFileModelFirst = po.getFlag("pose-file-model-first");
    const bool visualize = po.getFlag("visualize");
    const bool append = po.getFlag("append");
    const bool useOmp = po.getFlag("omp");
    const bool dryrun = po.getFlag("dry-run");
    
    if(!useOmp)
        omp_set_num_threads(1);
    
    /*
     * END PROGRAM OPTIONS BLOCK
     */

    // Prepare output directories
    if(!boost::filesystem::is_directory(boost::filesystem::path(outputDir.c_str()))) {
        if(verbose)
            COVIS_MSG("Output directory \"" << outputDir << "\" does not exist, creating it...");
        COVIS_ASSERT(boost::filesystem::create_directory(boost::filesystem::path(outputDir.c_str())));
    }
    
    /*
     * Load query models
     */
    if(verbose)
        COVIS_MSG_INFO("Loading and preprocessing " << queries.size() << " query models...");
    surfq.resize(queries.size());
    meshq.resize(queries.size());
    std::pair<size_t,size_t> sizes(0, 0); // Stats
    for(size_t i = 0; i < queries.size(); ++i) {
        if(verbose)
            COVIS_MSG("\t" << boost::filesystem::path(queries[i]).stem().string());
        surfq[i].reset(new CloudT);
        meshq[i].reset(new pcl::PolygonMesh);
        pcl::io::loadPLYFile(queries[i], *meshq[i]);
        pcl::fromPCLPointCloud2(meshq[i]->cloud, *surfq[i]);
        COVIS_ASSERT_MSG(!meshq[i]->polygons.empty() && !surfq[i]->empty(), "Empty object polygon or vertex set!");
        sizes.first += surfq[i]->size();
        sizes.second += meshq[i]->polygons.size();
        
        // Normalize the normals
        size_t invalidNormals = 0;
        for(size_t j = 0; j < surfq[i]->size(); ++j) {
            Eigen::Vector3f n = surfq[i]->points[j].getNormalVector3fMap();
            if(!n.hasNaN() && n.norm() > 1e-5f) {
                n.normalize();
            } else {
                ++invalidNormals;
                n.setZero();
            }
            surfq[i]->points[j].getNormalVector3fMap() = n;
        }
        
        if(invalidNormals > 0)
            COVIS_MSG_WARN("Warning: mesh has " << invalidNormals << " invalid normals!");
        
        // Copy normalized normals points back to mesh
        pcl::toPCLPointCloud2(*surfq[i], meshq[i]->cloud);
        
//        show(meshq[i]);
    }
    
    COVIS_MSG_INFO("Got " << queries.size() << " surface models with an average of " <<
            sizes.first / queries.size() << " vertices and " <<
            sizes.second / queries.size() << " polygons");
    
    /*
     * Decimate queries
     */
    sizes = std::make_pair(0,0);
    if(decimation > 0.0 && decimation < 1.0) {
        if(verbose)
            COVIS_MSG_INFO("Performing mesh decimation with a factor of " << decimation << "...");
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for(size_t i = 0; i < queries.size(); ++i) {
            const boost::filesystem::path qpath(queries[i]);
#ifdef _OPENMP
#pragma omp critical
#endif
            if(verbose)
                COVIS_MSG("\t" << qpath.stem().string());
            
            decimate(*meshq[i], decimation);
            pcl::fromPCLPointCloud2(meshq[i]->cloud, *surfq[i]);

#ifdef _OPENMP
#pragma omp critical
#endif
            {
                sizes.first += surfq[i]->size();
                sizes.second += meshq[i]->polygons.size();
            }
        }
        
        COVIS_MSG_INFO("Got " << queries.size() << " decimated surface models with an average of " <<
                sizes.first / queries.size() << " vertices and " <<
                sizes.second / queries.size() << " polygons");
    }

    // Estimate average model resolution, used for setting inlier threshold, feature radius and feature resolution
    if(verbose)
        COVIS_MSG_INFO("Estimating average object mesh resolution...");
    Eigen::VectorXd resolutions(queries.size());
    for(size_t i = 0; i < queries.size(); ++i)
        resolutions[i] = resolution(meshq[i], surfq[i]);
    
    avgModelMeshRes = resolutions.mean();
    for(size_t i = 0; i < frad.size(); ++i)
        frad[i] *= avgModelMeshRes;
    fresQuery *= avgModelMeshRes;
    fres *= avgModelMeshRes;
    thres *= avgModelMeshRes;
    
    if(verbose) {
        COVIS_MSG("\t" << avgModelMeshRes);
        core::print(frad, std::cout, "Setting feature radii to: [", "]");
        COVIS_MSG_INFO("Setting query/target feature resolution to: " << fresQuery << "/" << fres);
        COVIS_MSG_INFO("Setting RANSAC+ICP inlier threshold to: " << thres);
    }
    
    /*
     * Seed the queries
     */
    if(verbose)
        COVIS_MSG_INFO("Selecting seed points for description using object feature resolution...");
    
    query.resize(queries.size());
    size_t size = 0;
    for(size_t i = 0; i < queries.size(); ++i) {
        seed(surfq[i], fresQuery, query[i]);
        
        size += query[i]->size();
//        show(meshq[i], query[i]);
    }
    
    COVIS_MSG_INFO("Got " << queries.size() << " object models with an average of " <<
            size / queries.size() << " points");
    
    
    /**
     * Compute features for model library
     */
    if(verbose)
        COVIS_MSG_INFO("Computing features for query model(s)...");
    
    // Each entry becomes a vector of length fnum
    std::vector<std::vector<MatrixT> > featq(queries.size());
    //std::vector<HTuple> halcon_queries_models; 
    
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(size_t i = 0; i < queries.size(); ++i) {    
#ifdef _OPENMP
#pragma omp critical
#endif
            if(verbose)
                COVIS_MSG("\t" << boost::filesystem::path(queries[i]).stem().string());
        featq[i].resize(fnum);
        Eigen::VectorXf dummy(fnum);
	// COVIS_MSG("\t" << "Compute features .......");
        features(meshq[i], surfq[i], query[i], frad, avgModelMeshRes,
                featq[i],
                dummy);
	// COVIS_MSG("\t" << "Done!!");
	
/*	if(verbose)
                COVIS_MSG_INFO("\t\t" << "--> halcon model ...");
	//Create Halcon Surface Model for each queries
	  //Create the object model from a point cloud
	  perception::Create3DObjectModel cadModelCreator;
	  HTuple halcon_model;
	  HTuple x,y,z;

	  CloudT::Ptr hc = query[i];
	  for (size_t i = 0; i < hc->points.size(); ++i){
	    x.Append(HTuple(hc->points[i].x));
	    y.Append(HTuple(hc->points[i].y));
	    z.Append(HTuple(hc->points[i].z));
	}

        cadModelCreator.createModelFromPoints(x,y,z);
        cadModelCreator.computeSurfaceNormals();       
        HTuple objectModel3D = cadModelCreator.get3DObjectModel();
	halcon_queries_models.push_back(objectModel3D);
	*/
    }
    
    /**
     * Compute PCA
     */
    std::vector<cv::PCA> pcas;
    std::vector<std::vector<size_t> > components;
    if(pca)
        pcaTrain(featq, fnames, std::vector<float>(1, pcaVariation), pcas, components, verbose);

    // Create a library of all object features
    std::vector<MatrixT> featqAll(fnum);
    for(size_t j = 0; j < queries.size(); ++j) {
        // Add features to library
        for(size_t k = 0; k < fnum; ++k) {
            const size_t rowsFirst = featqAll[k].rows();
            const size_t rowsSecond = featq[j][k].rows();
            const size_t cols = featq[j][k].cols();
            COVIS_ASSERT(rowsFirst + rowsSecond > 0 && cols > 0);
            featqAll[k].conservativeResize(rowsFirst + rowsSecond, cols);
            featqAll[k].block(rowsFirst, 0, rowsSecond, cols) = featq[j][k];
        }
    }
    
    // Create a map from global feature index to <object, feature>
    std::vector<std::pair<size_t,size_t> > mapFeatqAll(featqAll[0].rows());
    size_t cnt = 0; // Global counter
    for(size_t j = 0; j < featq.size(); ++j) // Number of objects
        for(int k = 0; k < featq[j][0].rows(); ++k) // Number of features
            mapFeatqAll[cnt++] = std::make_pair(j, k);
    COVIS_ASSERT(cnt == mapFeatqAll.size());
    
    // Project object features to PCA subspaces
    if(pca) {
        // Now update the features
        if(verbose)
            COVIS_MSG_WARN("Projecting all query features to PCA subspaces...");
        Eigen::VectorXf dummy(fnum);
        pcaProject(featqAll, fnames, pcas, components, dummy, verbose);
    }
    
    // TODO: Create a search index
    if(verbose)
        COVIS_MSG_INFO("Creating search structures for " << featqAll[0].rows() << " object library features...");
    
    typedef Matcher<L2_RATIO> MatcherT;
    const int numTrees = 4; // TODO: Put to cmdline
    MatcherT::KDTreePtr trees[fnum];
    for(size_t j = 0; j < fnum; ++j) {
        if(verbose)
            COVIS_MSG("\t" << fnames[j] << "...");
        trees[j] = MatcherT::index(featqAll[j], numTrees);
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Start loop over all scenes
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    const size_t& fnumDetection = (fusion ? fnumFusion : fnum);
    const std::vector<std::string>& fnamesDetection = (fusion ? fnamesFusion : fnames);
    
    // For each feature there is a matrix with detection/timing information, one row per scene
    MatrixT detectionOutputs[fnumDetection]; //+1 to add halcon
    MatrixT detectionTimings[fnumDetection]; //+1 to add halcon

    
    for(size_t i = 0; i < targets.size(); ++i) {
        const boost::filesystem::path tpath(targets[i]);
        const std::string pscn = tpath.stem().string();
        
        if(verbose)
            COVIS_MSG_INFO("---------- PROCESSING SCENE " << i+1 << "/" << targets.size() <<
                    " (" << pscn << ") ----------");
        
        CloudT::Ptr surft(new CloudT);
        pcl::PolygonMesh::Ptr mesht(new pcl::PolygonMesh);
	COVIS_MSG_INFO("LOADING SCENE " << targets[i]);
        pcl::io::loadPLYFile(targets[i], *mesht);
        pcl::fromPCLPointCloud2(mesht->cloud, *surft);
        COVIS_ASSERT_MSG(!mesht->polygons.empty() && !surft->empty(), "Empty scene polygon or vertex set!");

        
        // Normalize the normals
        size_t invalidNormals = 0;
        for(size_t j = 0; j < surft->size(); ++j) {
            Eigen::Vector3f n = surft->points[j].getNormalVector3fMap();
            if(!n.hasNaN() && n.norm() > 1e-5f) {
                n.normalize();
            } else {
                ++invalidNormals;
                n.setZero();
            }
            surft->points[j].getNormalVector3fMap() = n;
        }
        
        if(invalidNormals > 0)
            COVIS_MSG_WARN("Warning: mesh has " << invalidNormals << " invalid normals!");

        // Copy normalized normals back to mesh
        pcl::toPCLPointCloud2(*surft, mesht->cloud);

        // Decimate mesh, copy back to surface
        float decimationTiming = 0.0;
        if(decimation > 0.0 && decimation < 1.0) {
            if(verbose)
                COVIS_MSG_INFO("Decimating scene...");
            TIME(\
                    decimate(*mesht, decimation);,
                    decimationTiming
                );
            pcl::fromPCLPointCloud2(mesht->cloud, *surft);
            
            if(verbose)
                COVIS_MSG("\tDecimation time: " << decimationTiming << " s");
        }
        
        // Now compute inlier threshold using scene resolution
        if(verbose)
            COVIS_MSG_INFO("Estimating scene mesh resolution...");
        const double sceneRes = resolution(mesht, surft);
		const double sceneResSq = sceneRes * sceneRes;
        
        // Generate search, only used for checking validity of GT poses
        detect::PointSearch<PointT> surftSearch;
        surftSearch.setTarget(surft);
        
        // Find out which objects are present in the scene, and their pose
        bool queryMask[queries.size()];
        Eigen::Matrix4f queryPoses[queries.size()];
        for(size_t j = 0; j < queries.size(); ++j) {
            // Resolve pose file
            const std::string pobj = boost::filesystem::path(queries[j]).stem().string();
            const std::string poseFile = (poseFileModelFirst ?
                    poseDir + "/" + pobj + poseSeparator + pscn + "." + poseSuffix : 
                    poseDir + "/" + pscn + poseSeparator + pobj + "." + poseSuffix);
	      COVIS_MSG_INFO("Loading: " <<  poseFile);
            
            // Load pose
            try {
                core::read(poseFile, queryPoses[j]);
		std::cout << queryPoses[j] << std::endl;
            } catch(const std::exception& e) {
                COVIS_MSG_WARN("Object \"" << pobj << "\" is not found in scene \"" << pscn << "\"!");
                queryMask[j] = false;
                continue;
            }
            
            // Transform object
            CloudT::Ptr queryTransformed(new CloudT);
	    Eigen::Matrix4f gt_pose = queryPoses[j];//.inverse();
	    std::cout << gt_pose << std::endl;
            pcl::transformPointCloud<PointT>(*query[j], *queryTransformed, gt_pose);
            
	//    show(mesht,queryTransformed);
           
	    // TODO: Check that the pose is actually valid
            core::Correspondence::VecPtr pointCorr = surftSearch.knn(queryTransformed, 1);
            size_t numInliers = 0;
            for(core::Correspondence::Vec::const_iterator it = pointCorr->begin(); it != pointCorr->end(); ++it)
                if(it->distance[0] <= sceneResSq)
                    ++numInliers;
            
            queryMask[j] = (numInliers >= size_t(0.01 * queryTransformed->size()));
            
            // In this case, the object is non-existent
            if(!queryMask[j]) {
                COVIS_MSG_WARN("Too few valid points (" << numInliers << "/" << queryTransformed->size() <<
                        ") for object \"" << pobj << "\" in scene \"" << pscn << "\"" << "!");
                COVIS_MSG_ERROR("Ground truth pose from dataset is invalid!");
                continue;
            }
        } // End local loop over queries for finding mask and poses (j)
        
        // TODO: No objects with GT information in scene
        if(std::accumulate(queryMask, queryMask + queries.size(), int(0)) == 0) {
            COVIS_MSG_ERROR("No object with GT information found for scene! Skipping...");
            continue;
        }
        
        /**
         * Find scene seed points
         */
        // Select seed points on the scene surface
        float seedTiming;
        CloudT::Ptr target(new CloudT);
	//HTuple halcon_target;
        TIME(\
                seed(surft, fres, target);,
                seedTiming
            );
        
        if(verbose)
            COVIS_MSG("\tSeed computation time: " << seedTiming << " s");
        
       // show(mesht, target);
        
        /**
         * Compute scene features
         */
        if(verbose)
            COVIS_MSG_INFO("Computing " << target->size() << " target features...");
        std::vector<MatrixT> featt(fnum);
        Eigen::VectorXf featureTiming(fnum);
        features(mesht, surft, target, frad, sceneRes,//avgModelMeshRes,
                featt,
                featureTiming);
        
        // Print the computation times
        if(verbose) {
            for(size_t j = 0; j < fnum; ++j)
                COVIS_MSG("\t" << fnames[j] << ": " << featureTiming(j) << " s");
        }
      
	
	/*	HTuple x,y,z;
		for (size_t i = 0; i < target->points.size (); ++i){
		  x.Append(HTuple(target->points[i].x));
		  y.Append(HTuple(target->points[i].y));
		  z.Append(HTuple(target->points[i].z));
		}

	       //Creating Model of the scene!!
	       perception::Create3DObjectModel SearchModelCreator;
	       SearchModelCreator.createModelFromPoints(x,y,z);
	       SearchModelCreator.computeSurfaceNormals();
	       halcon_target = SearchModelCreator.get3DObjectModel();
	       
	if(verbose)
            COVIS_MSG("\tHalcon: 0" << "s");
*/
        /**
         * Compute PCA subspaces for the target features
         */
        Eigen::VectorXf projectionTiming(fnum);
        projectionTiming.setZero();
        if(pca) {
            // Now update the features
            if(verbose)
                COVIS_MSG_WARN("Projecting all target features to PCA subspaces...");
            pcaProject(featt, fnames, pcas, components, projectionTiming, verbose);
        }
        
        /**
         * Perform feature matching
         */
        if(verbose)
            COVIS_MSG_INFO("Matching " << featt[0].rows() << " scene features with object library...");
        
        const int checks = 512; // TODO: Promote to cmdline
        // This contains (for each feature) a complete set of correspondences scene feature --> stacked object features
        std::vector<core::Correspondence::VecPtr> featureCorr(fnum);
        Eigen::VectorXf matchTiming(fnum);
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for(size_t j = 0; j < fnum; ++j) {
            TIME(\
                    featureCorr[j] = MatcherT::knn(*trees[j], featt[j], checks);
                    ,        
                    matchTiming[j]
                );

#ifdef _OPENMP
#pragma omp critical
#endif
            if(verbose)
                COVIS_MSG("\t" << fnames[j] << ": " << matchTiming[j] << " s");
        }
        
        /**
         * Fusion
         */
        // Conservatively, we measure the full fusion time of all combinations, since it is tough to time each combination independently
        Eigen::VectorXf fusionTiming(fnumDetection);
        fusionTiming.setZero();
        if(fusion)
            fuse(featureCorr, fusionCombinations, fusionTiming);
                    
        
        /**
         * Perform object detection
         */
        core::Correspondence::VecPtr corrQuerytarget[fnumDetection][featq.size()];
        
        Eigen::VectorXf detectionTiming = Eigen::VectorXf::Zero(fnumDetection);
        Eigen::VectorXf refinementTiming = Eigen::VectorXf::Zero(fnumDetection);
        Eigen::VectorXf segmentationTiming = Eigen::VectorXf::Zero(fnumDetection);
        
	size_t J = 0;
        // Loop over number of features
        for(size_t j = 0; j < fnumDetection; ++j) {
            // Create a timer for this feature to measure the total detection time of all objects in current scene
            core::ScopedTimer::Ptr t(new core::ScopedTimer("Object detection+refinement+segmentation, " + fnamesDetection[j]));
            
            // Loop over query objects and allocate correspondences object --> scene per object
            for(size_t k = 0; k < featq.size(); ++k)
                corrQuerytarget[j][k].reset(new core::Correspondence::Vec);
            
            // Reverse all correspondences (object --> scene), and spread them out on all object models
            for(size_t k = 0; k < featureCorr[j]->size(); ++k) {
                const size_t idxMatch = (*featureCorr[j])[k].match[0]; // Index into object library
                const std::pair<size_t,size_t>& idxFeatq = mapFeatqAll[idxMatch]; // Index for <object, feature>
                corrQuerytarget[j][idxFeatq.first]->push_back(
                        core::Correspondence(idxFeatq.second, (*featureCorr[j])[k].query, (*featureCorr[j])[k].distance[0]));
            }
        
            // Take a subset of best correspondences for each object
//            const float ratioThres = 0.64; // 0.8^2 = 0.64
            std::vector<size_t> sizes(featq.size());
            for(size_t k = 0; k < featq.size(); ++k) {
//                corrQuerytarget[j][k] = detect::filterCorrespondencesDistance(*corrQuerytarget[j][k], ratioThres);
                core::sort(*corrQuerytarget[j][k]);
                corrQuerytarget[j][k]->resize(corrQuerytarget[j][k]->size() * corrFraction);
                sizes[k] = corrQuerytarget[j][k]->size();
            }
            
            // TODO: Sort the object list by descending number of correspondences
            const std::vector<size_t> order = core::sort<size_t>(sizes, false);
            
            // For evaluating poses
            detect::FitEvaluation<PointT>::Ptr eval(new detect::FitEvaluation<PointT>(target));
            eval->setOcclusionReasoning(true); //setOcclusionRemoval(true);
            
            // For doing segmentation
            detect::PointSearch<PointT> targetSearch;
            targetSearch.setTarget(target);

            // Scene detections
            core::Detection::Vec detections;
            std::vector<bool> targetMask(target->size(), true); // Will be filled with false for each segmented point
            
            // Loop over all objects and run RANSAC in the above order
            for(size_t k = 0; k < queries.size(); ++k) {
                // Current object index
                const size_t kidx = order[k];
                const std::string queryName = fs::path(queries[kidx]).stem().string();
                
                // Take correspondences only for remaining target feature points
                core::Correspondence::VecPtr corrRemain(new core::Correspondence::Vec);
                for(size_t l = 0; l < corrQuerytarget[j][kidx]->size(); ++l)
                    if(targetMask[ (*corrQuerytarget[j][kidx])[l].match[0] ])
                        corrRemain->push_back((*corrQuerytarget[j][kidx])[l]);
                
                // If this object has no correspondences in remaining scene data, continue to next
                if(corrRemain->size() < 3) {
                    if(verbose)
                        COVIS_MSG_WARN("No correspondences for object " << queryName << " - not found!");
                    continue;
                }
		               
                // Initialize RANSAC
                detect::Ransac<PointT> ransac;
                ransac.setSource(query[kidx]);
                ransac.setTarget(target);
                ransac.setCorrespondences(corrRemain);
                ransac.setFitEvaluation(eval);
                
                ransac.setIterations(ransacIterations);
                ransac.setInlierThreshold(thres); // Inlier threshold
                ransac.setInlierFraction(inlierFraction); // Inlier fraction
                ransac.setReestimatePose(true); // For more accurate poses
                ransac.setFullEvaluation(false); // Evaluation using only feature point matches
                ransac.setWeightedSampling(true); // Sample correspondences with a probability proportional to their quality
                ransac.setPrerejection(false); // TODO: Later...
                ransac.setPrerejectionSimilarity(0.9); // TODO: Later...
                
                ransac.setVerbose(verbose);

                // Perform refinement
                core::Detection d = ransac.estimate();
		
                // Increment total detection time for current feature
                detectionTiming[j] += t->seconds();
                
                // If RANSAC produced a result
                if(d) {
                    // Refine
                    if(verbose)
                        COVIS_MSG("Object " << queryName << " detected! Refining pose...");
                    core::ScopedTimer::Ptr ticp(new core::ScopedTimer("Refinement"));
                    pcl::IterativeClosestPoint<PointT,PointT> icp;
                    icp.setInputSource(query[kidx]);
                    icp.setInputTarget(target);
//                    icp.setInputSource(surfq[kidx]);
//                    icp.setInputTarget(surft);
                    icp.setMaximumIterations(icpIterations);
                    icp.setMaxCorrespondenceDistance(thres); // Set inlier threshold to lowest resolution
                    pcl::PointCloud<PointT> tmp;
                    icp.align(tmp, d.pose);
                    if(icp.hasConverged()) {
                        d.pose = icp.getFinalTransformation();
                        d.rmse = icp.getFitnessScore();
                    } else {
                        if(verbose)
                            COVIS_MSG("\tICP failed!");
                    }
                    
                    // Increment total refinement time for current feature
                    refinementTiming[j] += ticp->seconds();
                    ticp.reset();
                    
                    // Correct the object index before storing
                    d.idx = kidx;
                    detections.push_back(d);
                    
                    // Mask out the segment in the scene
                    if(verbose)
                        COVIS_MSG("Segmenting object " << queryName << " from scene...");
                    core::ScopedTimer::Ptr tseg(new core::ScopedTimer("Segmentation"));
                    CloudT queryT;
                    pcl::transformPointCloud<PointT>(*query[kidx], queryT, d.pose);
                    core::Correspondence::VecPtr corrQueryTTarget = targetSearch.radius(queryT, fres); // TODO: Radius
                    size_t cnt = 0;
                    for(size_t l = 0; l < corrQueryTTarget->size(); ++l)
                        for(size_t m = 0; m < (*corrQueryTTarget)[l].size(); ++m)
                            if(targetMask[ (*corrQueryTTarget)[l].match[m] ])
                                ++cnt, targetMask[ (*corrQueryTTarget)[l].match[m] ] = false;
                    
                    if(verbose)
                        COVIS_MSG("\tSegmented " << cnt << "/" << target->size() << " scene points");
                    
                    // Increment total refinement time for current feature
                    segmentationTiming[j] += tseg->seconds();
                }
                
                // Prepare detection outputs for current scene (row): [detected present translation_error rotation_error]
                const int rowidx = detectionOutputs[j].rows();
                detectionOutputs[j].conservativeResize(rowidx + 1, 4);
                detectionOutputs[j](rowidx, 0) = bool(d); // 1 if detected
                detectionOutputs[j](rowidx, 1) = queryMask[kidx]; // 1 if present
                detectionOutputs[j](rowidx, 2) = detectionOutputs[j](rowidx, 3) = -1; // Initialize pose errors to invalids
                
                /*
                 * Detection is done, now store results in output files
                 */
                
                if(d) { // Detected
                    // Compare the pose with GT (if existent), update detection outputs
                    if(queryMask[kidx]) { // Detected and present
                        if(verbose)
                            COVIS_MSG_INFO("TRUE POSITIVE for object " << queryName << "!");
                        // Translations
                        const Eigen::Vector3f tdet = d.pose.block<3,1>(0,3);
                        const Eigen::Vector3f tgt = queryPoses[kidx].block<3,1>(0,3);
                        // Rotations
                        const Eigen::Matrix3f Rdet = d.pose.block<3,3>(0,0);
                        const Eigen::Matrix3f Rgt = queryPoses[kidx].block<3,3>(0,0);                    
                        // Pose errors
                        const float terr = (tdet - tgt).norm();
                        float Rerr;
                        const float acosarg = 0.5f * ((Rdet.transpose() * Rgt).trace() - 1.0f); // In [-1,1]
                        if(acosarg < -1.0f + 1e-5f) // Approaching -1
                            Rerr = M_PI;
                        else if(acosarg > 1.0f - 1e-5f) // Approaching 1
                            Rerr = 0.0f;
                        else
                            Rerr = acos(acosarg);
                        Rerr *= 180.0f / M_PI;
                        // Report pose errors
                        if(terr <= translationTol && Rerr <= rotationTol) { // Pose errors small, we're good
                            if(verbose)
                                COVIS_MSG("\t- and pose is good! Translation/rotation error: [metric/deg]: " <<
                                        terr << "/" << Rerr);
                        } else {
                            if(verbose)
                                COVIS_MSG_WARN("\t- but with bad pose! Translation/rotation error: [metric/deg]: " <<
                                        terr << "/" << Rerr);
                        }
                        // Update pose errors in detection output
                        detectionOutputs[j](rowidx, 2) = terr;
                        detectionOutputs[j](rowidx, 3) = Rerr;
                    } else { // Detected, but not present
                        if(verbose)
                            COVIS_MSG_WARN("FALSE POSITIVE for object " << queryName << "!");
                    }
                } else { // Not detected
                    if(queryMask[kidx]) { // Not detected, but present
                        if(verbose)
                            COVIS_MSG_WARN("FALSE NEGATIVE for object " << queryName << "!");
                    } else { // Not detected and not present
                        if(verbose)
                            COVIS_MSG_INFO("TRUE NEGATIVE for object " << queryName << "!");
                    }
                }
            } // End loop over objects for detection (k)
          
            
            // Store timings for scene: [decimation seeds features projection matching fusion detection refinement segmentation]
            const int rowidx = detectionTimings[j].rows();
            detectionTimings[j].conservativeResize(rowidx + 1, 9);
            detectionTimings[j](rowidx, 0) = decimationTiming;
            detectionTimings[j](rowidx, 1) = seedTiming;
     
	    if(fusion) {
#define MAX3(a,b,c) (a > b ? a : (a > c ? a : c)) // Max of 3 values
#define MAX3V(v,idx1,idx2,idx3) MAX3(v[idx1], v[idx2], v[idx3]) // Max of three indexed elements in v
                const size_t idx1 = fusionCombinations[j].get<0>();
                const size_t idx2 = fusionCombinations[j].get<1>();
                const size_t idx3 = fusionCombinations[j].get<2>();
                detectionTimings[j](rowidx, 2) = MAX3V(featureTiming, idx1, idx2, idx3);
                detectionTimings[j](rowidx, 3) = MAX3V(projectionTiming, idx1, idx2, idx3);
                detectionTimings[j](rowidx, 4) = MAX3V(matchTiming, idx1, idx2, idx3);
            } else {
                detectionTimings[j](rowidx, 2) = featureTiming[j];
                detectionTimings[j](rowidx, 3) = projectionTiming[j];
                detectionTimings[j](rowidx, 4) = matchTiming[j];
            }
            
            detectionTimings[j](rowidx, 5) = fusionTiming[j];
            detectionTimings[j](rowidx, 6) = detectionTiming[j];
            detectionTimings[j](rowidx, 7) = refinementTiming[j];
            detectionTimings[j](rowidx, 8) = segmentationTiming[j];
            
            // Stop timer before visualization below
            t.reset();
            
            // Show detections
            if(detections.empty()) {
                if(verbose)
                    COVIS_MSG_ERROR("No detections for feature " << fnamesDetection[j] << "!");
            } else {
                if(visualize) {
                    visu::DetectionVisu<PointT> dvisu;
                    dvisu.setBackgroundColor(255, 255, 255);
                    dvisu.setTitle("Detections for " + fnamesDetection[j]);
                    dvisu.setQueries(meshq);
                    dvisu.setTarget(mesht);
                    dvisu.setDetections(detections);
                    dvisu.visu.loadCameraParameters("camera_parameters.cam");
                    dvisu.show();
                } else {
                    if(verbose)
                        COVIS_MSG_INFO(detections.size() << " detections for feature " << fnamesDetection[j]);
                }
            }
            J = j;
        } // End loop over feature types to use for detection (j)
      
      
      /*
       *  Halcon Recognition
       */
      
       // For doing segmentation
        detect::PointSearch<PointT> targetSearch;
        targetSearch.setTarget(target);
	    
        // Scene detections
        core::Detection::Vec halcon_detections;
        std::vector<bool> targetMask(target->size(), true); // Will be filled with false for each segmented point
                
       // Create a timer for this feature to measure the total detection time of all objects in current scene
       core::ScopedTimer::Ptr t(new core::ScopedTimer("Object detection+refinement+segmentation, halcon"));
       
/*        for(size_t k = 0; k < queries.size(); ++k) {
              // Current object index
              const size_t kidx = k;
              const std::string queryName = fs::path(queries[kidx]).stem().string();
              core::Detection d_halcon;
	      
	      //Detect Halcon
	      //Halcon parameters
	      SurfaceModelCreationParameters smcParams;
	      SurfaceModelDetectionParameters detectParams;
	      
	      //Create the surface model using the objectModel
	      smcParams.ObjectModel3D = halcon_queries_models[k];
   
	      perception::SurfaceModelCreator smc;
	      smc.setParameters(smcParams);
	      smc.createModel();
	      
	      HTuple pose, score;
	      perception::SurfaceModelDetector detector;
	      detectParams.num_matches = 10;
	      detectParams.MinScore = 0.2;
	      detector.setParameters(detectParams);
	      detector.setSurfaceModel(smc.getSurfaceModel());
 
	      pcl::console::print_info("Detecting 3D model -> %s!\n", queryName.c_str());
	      int instances = detector.detectModel(halcon_target);
	      Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
	      if(instances > 0){
		  pcl::console::print_info("Found %d instances of the model in the scene!\n", instances);

		  if(detector.getBestMatch(pose, score) == 1){
		    HTuple HomMat;
		    PoseToHomMat3d(pose,&HomMat);


		    
		    t(0,0) = (double)HomMat[0]; t(0,1) = (double)HomMat[1]; t(0,2) = (double)HomMat[2]; t(0,3) = (double)HomMat[3];
		    t(1,0) = (double)HomMat[4]; t(1,1) = (double)HomMat[5]; t(1,2) = (double)HomMat[6]; t(1,3) = (double)HomMat[7];
		    t(2,0) = (double)HomMat[8]; t(2,1) = (double)HomMat[9]; t(2,2) = (double)HomMat[10]; t(2,3) = (double)HomMat[11];
		    t(3,0) = 0; t(3,1) = 0; t(3,2) = 0; t(3,3) = 1;
	
		  }	
	 //     }
	       
		  d_halcon.pose = t;
		  d_halcon.idx = kidx;
		  HTuple score;
		  detector.getScoreBeforeRefining(0,score);
		  d_halcon.rmse = static_cast<float>(score);
	
		 // d_halcon.rmse = detec
	      //  if(d) {
	//  if(d_halcon) {
                    // Refine
                    if(verbose)
                        COVIS_MSG("Object " << queryName << " detected! Refining pose...");
                    core::ScopedTimer::Ptr ticp(new core::ScopedTimer("Refinement"));
                    pcl::IterativeClosestPoint<PointT,PointT> icp;
                    icp.setInputSource(query[kidx]);
                    icp.setInputTarget(target);
//                    icp.setInputSource(surfq[kidx]);
//                    icp.setInputTarget(surft);
                    icp.setMaximumIterations(icpIterations);
                    icp.setMaxCorrespondenceDistance(thres); // Set inlier threshold to lowest resolution
                    pcl::PointCloud<PointT> tmp;
                    icp.align(tmp, d_halcon.pose);
                    if(icp.hasConverged()) {
                        d_halcon.pose = icp.getFinalTransformation();
                        d_halcon.rmse = icp.getFitnessScore();
                    } else {
                        if(verbose)
                            COVIS_MSG("\tICP failed!");
                    }
                   
                   // Correct the object index before storing
                   d_halcon.idx = kidx;
                   halcon_detections.push_back(d_halcon);
		   
		      // Mask out the segment in the scene
                    if(verbose)
                        COVIS_MSG("Segmenting object " << queryName << " from scene...");
                    core::ScopedTimer::Ptr tseg(new core::ScopedTimer("Segmentation"));
                    CloudT queryT;
                    pcl::transformPointCloud<PointT>(*query[kidx], queryT, d_halcon.pose);
		    std::stringstream ss; ss << "/home/thso/query_"; ss << kidx; ss << ".pcd";
		    pcl::io::savePCDFile(ss.str(),queryT);
		    pcl::io::savePCDFile("/home/thso/target.pcd",*target);
                    core::Correspondence::VecPtr corrQueryTTarget = targetSearch.radius(queryT, fres); // TODO: Radius
                    size_t cnt = 0;
                    for(size_t l = 0; l < corrQueryTTarget->size(); ++l)
                        for(size_t m = 0; m < (*corrQueryTTarget)[l].size(); ++m)
                            if(targetMask[ (*corrQueryTTarget)[l].match[m] ])
                                ++cnt, targetMask[ (*corrQueryTTarget)[l].match[m] ] = false;
                    
                    if(verbose)
                        COVIS_MSG("\tSegmented " << cnt << "/" << target->size() << " scene points");
                    
		  }
		  
		    // Prepare detection outputs for current scene (row): [detected present translation_error rotation_error]
                const int rowidx = detectionOutputs[J+1].rows();
                detectionOutputs[J+1].conservativeResize(rowidx + 1, 4);
                detectionOutputs[J+1](rowidx, 0) = bool(d_halcon); // 1 if detected
                detectionOutputs[J+1](rowidx, 1) = queryMask[kidx]; // 1 if present
                detectionOutputs[J+1](rowidx, 2) = detectionOutputs[J+1](rowidx, 3) = -1; // Initialize pose errors to invalids
   */
	      
	         /*
                 * Detection is done, now store results in output files
                 */
                
   /*            if(d_halcon) { // Detected
                    // Compare the pose with GT (if existent), update detection outputs
                    if(queryMask[kidx]) { // Detected and present
                        if(verbose)
                            COVIS_MSG_INFO("TRUE POSITIVE for object " << queryName << "!");
                        // Translations
                        const Eigen::Vector3f tdet = d_halcon.pose.block<3,1>(0,3);
                        const Eigen::Vector3f tgt = queryPoses[kidx].block<3,1>(0,3);
                        // Rotations
                        const Eigen::Matrix3f Rdet = d_halcon.pose.block<3,3>(0,0);
                        const Eigen::Matrix3f Rgt = queryPoses[kidx].block<3,3>(0,0);                    
                        // Pose errors
                        const float terr = (tdet - tgt).norm();
                        float Rerr;
                        const float acosarg = 0.5f * ((Rdet.transpose() * Rgt).trace() - 1.0f); // In [-1,1]
                        if(acosarg < -1.0f + 1e-5f) // Approaching -1
                            Rerr = M_PI;
                        else if(acosarg > 1.0f - 1e-5f) // Approaching 1
                            Rerr = 0.0f;
                        else
                            Rerr = acos(acosarg);
                        Rerr *= 180.0f / M_PI;
                        // Report pose errors
                        if(terr <= translationTol && Rerr <= rotationTol) { // Pose errors small, we're good
                            if(verbose)
                                COVIS_MSG("\t- and pose is good! Translation/rotation error: [metric/deg]: " <<
                                        terr << "/" << Rerr);
                        } else {
                            if(verbose)
                                COVIS_MSG_WARN("\t- but with bad pose! Translation/rotation error: [metric/deg]: " <<
                                        terr << "/" << Rerr);
                        }
                        // Update pose errors in detection output
                        detectionOutputs[J+1](rowidx, 2) = terr;
                        detectionOutputs[J+1](rowidx, 3) = Rerr;
                    } else { // Detected, but not present
                        if(verbose)
                            COVIS_MSG_WARN("FALSE POSITIVE for object " << queryName << "!");
                    }
                } else { // Not detected
                    if(queryMask[kidx]) { // Not detected, but present
                        if(verbose)
                            COVIS_MSG_WARN("FALSE NEGATIVE for object " << queryName << "!");
                    } else { // Not detected and not present
                        if(verbose)
                            COVIS_MSG_INFO("TRUE NEGATIVE for object " << queryName << "!");
                    }
            } 
                     
	}// End loop over objects for detection (k)
	*/
/*	    if(halcon_detections.empty()) {
                if(verbose)
                    COVIS_MSG_ERROR("No detections for halcon!");
            } else {
                if(visualize) {
                    visu::DetectionVisu<PointT> dvisu;
                    dvisu.setBackgroundColor(255, 255, 255);
                    dvisu.setTitle("Detections for Halcon");
                    dvisu.setQueries(meshq);
                    dvisu.setTarget(mesht);
                    dvisu.setDetections(halcon_detections);
                    dvisu.visu.loadCameraParameters("camera_parameters.cam");
                    dvisu.show();
                }// else {
                 //   if(verbose)
                     //   COVIS_MSG_INFO(detections.size() << " detections for feature " << fnamesDetection[j]);
               // }
            }
            */
	
    } // End loop over all scenes (i)
    
    
    
    /*
     * Report overall recognition results
     */
    if(verbose) {
        COVIS_MSG_INFO("OVERALL RECOGNITION RESULTS: PRECISION/RECALL");
        
        for(size_t j = 0; j < fnumDetection; ++j) {
            const size_t positives = detectionOutputs[j].col(1).sum();
            size_t truePositives = 0;
            size_t falsePositives = 0;
            for(int r = 0; r < detectionOutputs[j].rows(); ++r) {
                if(detectionOutputs[j](r,0) > 0) { // 1 if detected
                    if(detectionOutputs[j](r,1) > 0 && // 1 if present
                            detectionOutputs[j](r,2) <= translationTol &&
                            detectionOutputs[j](r,3) <= rotationTol)
                        ++truePositives;
                    else
                        ++falsePositives;
                }
            }
            
            const float precision = float(truePositives) / float(truePositives + falsePositives);
            const float recall = float(truePositives) / float(positives);
            COVIS_MSG("\t" << fnamesDetection[j] << ": " << precision << "/" << recall);
        }
    }
    
    /*
     * Store output files
     */
    if(!dryrun) {
        const std::string suffix = ".txt";
        for(size_t j = 0; j < fnumDetection; ++j) {
            core::write(outputDir + "/detection_output_" + fnamesDetection[j] + suffix, detectionOutputs[j], true, false, append);
            core::write(outputDir + "/meta_detection_timings_" + fnamesDetection[j] + suffix, detectionTimings[j], true, false, append);
        }
    }
    
    return 0;
}
