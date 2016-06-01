// CoViS
#include <covis/covis.h>
using namespace covis;

// Boost
#include <boost/range.hpp>

// PCL
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/vtk_lib_io.h"

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
    po.addOption("resolution", 'e', 5, "resolution of the feature seed points, given as a multiple of the average query mr (<= for full resolution)");
    po.addFlag('c', "pca", "use PCA versions of a predefined subset of descriptors");
    po.addOption("pca-variations", "99,95,90", "if PCA is enabled, use this list of variations [%] for selecting the number of components");
    po.addFlag('f', "fusion", "enable feature fusion");
    
    // Matching
    po.addOption("metrics", "all", "select a subset of distance functions - in case of 'all', all metrics are enabled");
    po.addOption("threshold", 't', 1, "inlier distance threshold, given as a multiple of the average query mr");
    po.addFlag('a', "match-scene-objects", "reverse matching order: target --> query");
    po.addFlag('n', "remove-non-overlap", "remove non-overlapping points from query and target");
    
    // General
    po.addOption("pose-separator", 'p', "-", "pose file separator inserted between scene and model file (e.g. '-' for the UWA dataset)");
    po.addFlag('m', "pose-file-model-first", "pose file order, set this flag for query model name first");
    po.addFlag('p', "append", "append data to output files");
    po.addFlag('v', "verbose", "print debug information");
    po.addFlag('o', "omp", "use OpenMP, if available");
    po.addFlag('u', "dry-run", "don't save any outputs");
    po.addFlag('b', "correspondences", "show correspondences");
    
    // Parse
    if(!po.parse(argc, argv))
        return 1;

    const bool verbose = po.getFlag("verbose");
    const bool correspondences = po.getFlag("correspondences");
    
    if(verbose)
        po.print();

    // Get positionals
    const std::vector<std::string> queries = po.getVector("query");
    const std::vector<std::string> targets = po.getVector("target");
    const std::string poseDir = po.getValue("pose-directory");
    const std::string poseSuffix = po.getValue("pose-suffix");
    std::string outputDir = po.getValue("output-directory");
    COVIS_ASSERT(!queries.empty() && !targets.empty());
    COVIS_ASSERT(!poseDir.empty() && !poseSuffix.empty() && !outputDir.empty());

    // Get feature list
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
    
    if(verbose)
        core::print(fnames, std::cout, "Testing the following feature(s): [", "]");
    
    // Get feature options
    const double decimation = po.getValue<double>("decimation");
    std::vector<double> fradMul = po.getVector<double>("radius");
    if(fradMul.size() == 1 && fnum > 1) {
        COVIS_MSG_WARN("Only one radius specified - applying to all features!");
        fradMul.resize(fnum, fradMul[0]);
    }
    std::vector<double> frad = fradMul;
    double fres = po.getValue<double>("resolution");

    // Get metric list
    mnames = po.getVector("metrics");
    if(mnames.empty() || (mnames.size() == 1 && boost::iequals(mnames[0], "all"))) { // No metrics provided, or "all"
        mnames = std::vector<std::string>(boost::begin(mnamesAll), boost::end(mnamesAll));
    } else { // Metrics provided
        for(size_t i = 0; i < mnames.size(); ++i) {
            boost::to_upper(mnames[i]);
            COVIS_ASSERT_MSG(std::find(boost::begin(mnamesAll), boost::end(mnamesAll), mnames[i]) != boost::end(mnamesAll),
                    "Unknown metric: " << mnames[i] << "!");
        }
    }
    mnum = mnames.size();
    
    if(verbose)
        core::print(mnames, std::cout, "Testing the following metric(s): [", "]");
    
    // Get PCA flag
    const bool pca = po.getFlag("pca");
    if(pca && mnum > 1)
        COVIS_THROW("PCA feature test only possible for one metric at a time!");
    
    // Get PCA variations
    const std::vector<float> pcaVariations = po.getVector<float>("pca-variations");
    if(pca && verbose)
        core::print(pcaVariations, std::cout, "\tTesting the following PCA varation(s) [%]: [", "]");
    
    // Set feature names for PCA, and increase the list of radius multipliers for getting correct output file names
    std::vector<std::string> fnamesPCA;
    if(pca) {
        std::vector<double> fradMulPCA;
        for(size_t i = 0; i < fnum; ++i) {
            for(size_t j = 0; j < boost::size(pcaVariations); ++j) {
                fnamesPCA.push_back(fnames[i] + "_pca" + core::stringify(pcaVariations[j]));
                fradMulPCA.push_back(fradMul[i]); // Same radius for all variations
            }
        }
        fradMul = fradMulPCA;
    }
    const size_t fnumPCA = fnamesPCA.size();
    
    // Get fusion flag
    const bool fusion = po.getFlag("fusion");
    if(fusion && mnum > 1)
        COVIS_THROW("Feature fusion only possible for one metric at a time!");
    if(fusion && pca)
        COVIS_THROW("Feature fusion and PCA not possible at the same time!");
    
    // Get fusion tuples
    std::vector<boost::tuple<int,int,int> > fusionCombinations; // A -1 means don't use
    if(fusion && fnum > 1) {
        if(verbose)
            COVIS_MSG_WARN("Feature fusion enabled! Testing the following combinations:");
        for(int i = 0; i < int(fnum) - 1; ++i) {
            for(int j = i + 1; j < int(fnum); ++j) {
                fusionCombinations.push_back(boost::make_tuple(i,j,-1));
                if(verbose)
                    COVIS_MSG("\t" << fnames[i] << "-" << fnames[j]);
                // Triplets
                if(fnum > 2 && i < int(fnum) - 2 && j < int(fnum) - 1) {
                    for(int k = j + 1; k < int(fnum); ++k) {
                        fusionCombinations.push_back(boost::make_tuple(i,j,k));
                        if(verbose)
                            COVIS_MSG("\t" << fnames[i] << "-" << fnames[j] << "-" << fnames[k]);
                    }
                }
            }
        }
    }
    const size_t fnumFusion = fusionCombinations.size();
    
    // Set fature names for fusion
    std::vector<std::string> fnamesFusion;
    for(size_t i = 0; i < fnumFusion; ++i) {
        const int idx1 = fusionCombinations[i].get<0>();
        const int idx2 = fusionCombinations[i].get<1>();
        const int idx3 = fusionCombinations[i].get<2>();
        std::string name = fnames[idx1] + "-" + fnames[idx2];
        if(idx3 != -1) // Triplet
            name += "-" + fnames[idx3];
        fnamesFusion.push_back(name);
    }
    
    // Get matching options
    const double thresMul = (po.getValue<double>("threshold") <= 0.0 ? 1.0 : po.getValue<double>("threshold"));
    const bool matchSceneObjects = po.getFlag("match-scene-objects");
    const bool removeNonOverlap = po.getFlag("remove-non-overlap");
    
    // Get misc. options
    const std::string poseSeparator = po.getValue("pose-separator");
    const bool poseFileModelFirst = po.getFlag("pose-file-model-first");
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

    const std::string outputDirObjects = outputDir + "/objects";
    const std::string outputDirScenes = outputDir + "/scenes";
    if(!dryrun) {
        if(!boost::filesystem::is_directory(boost::filesystem::path(outputDirObjects.c_str()))) {
            if(verbose)
                COVIS_MSG("Object model output directory \"" << outputDirObjects << "\" does not exist, creating it...");
            COVIS_ASSERT(boost::filesystem::create_directory(boost::filesystem::path(outputDirObjects.c_str())));
        }
        
        if(!boost::filesystem::is_directory(boost::filesystem::path(outputDirScenes.c_str()))) {
            if(verbose)
                COVIS_MSG("Scene model output directory \"" << outputDirScenes << "\" does not exist, creating it...");
            COVIS_ASSERT(boost::filesystem::create_directory(boost::filesystem::path(outputDirScenes.c_str())));
        }
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
        
        for(size_t i = 0; i < queries.size(); ++i) {
            const boost::filesystem::path qpath(queries[i]);
            if(verbose)
                COVIS_MSG("\t" << qpath.stem().string());
            decimate(*meshq[i], decimation);
            pcl::fromPCLPointCloud2(meshq[i]->cloud, *surfq[i]);

            sizes.first += surfq[i]->size();
            sizes.second += meshq[i]->polygons.size();
            
            // TODO: Save decimated mesh
            if(!dryrun) {
                const std::string pathSurf = outputDirObjects + "/" + qpath.stem().string() +
                        "_surface" + qpath.extension().string();
                if(verbose)
                    COVIS_MSG("Saving decimated mesh \"" << pathSurf << "\"...");
                pcl::io::savePLYFileBinary(pathSurf, *meshq[i]);
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
    fres *= avgModelMeshRes;
    
    if(verbose) {
        COVIS_MSG("\t" << avgModelMeshRes);
        core::print(frad, std::cout, "Setting feature radii to: [", "]");
        if(fres > 0)
            COVIS_MSG_INFO("Setting feature resolution to: " << fres);
        else
            COVIS_MSG_INFO("Setting feature resolution to full resolution!");
    }
    
    /*
     * Seed the queries
     */
    if(verbose) {
        if(fres > 0.0)
            COVIS_MSG_INFO("Selecting seed points for description using object feature resolution...");
        else
            COVIS_MSG_INFO("Selecting all surface points for description...");
    }
    
    query.resize(queries.size());
    size_t size = 0;
    for(size_t i = 0; i < queries.size(); ++i) {
        if(fres > 0.0)
            seed(surfq[i], fres, query[i]);
        else
            query[i] = boost::make_shared<CloudT>(*surfq[i]);
        
        size += query[i]->size();

        if(!dryrun) {
            const boost::filesystem::path qpath(queries[i]);
            const std::string pathSeed = outputDirObjects + "/" + qpath.stem().string() +
                    "_seeds" + qpath.extension().string();
            if(verbose)
                COVIS_MSG("Saving mesh seeds \"" << pathSeed << "\"...");
            pcl::io::savePLYFileBinary(pathSeed, *query[i]);
        }
        
//        show(meshq[i], query[i]);
    }
    
    COVIS_MSG_INFO("Got " << queries.size() << " query model(s) with an average of " <<
            size / queries.size() << " points");
    
    
    /**
     * Compute features for model library
     */
    if(verbose)
        COVIS_MSG_INFO("Computing features for query model(s)...");
    
    // Each entry becomes a vector of length fnum
    std::vector<std::vector<MatrixT> > featq(queries.size());
    
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
        features(meshq[i], surfq[i], query[i], frad, avgModelMeshRes,
                featq[i],
                dummy);
    }
    
    /**
     * Compute PCA
     */
    std::vector<cv::PCA> pcas;
    std::vector<std::vector<size_t> > components;
    if(pca)
        pcaTrain(featq, fnames, pcaVariations, pcas, components, verbose);
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Start loop over all scenes
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Eigen::MatrixXi featureNumbers(targets.size(), 2 + fnum); // First column is the query, second is the target, the rest the neighbor numbers for each feature
    Eigen::MatrixXf featureTimings(targets.size(), (pca ? fnumPCA : fnum));
    Eigen::MatrixXf matchingOutput[mnum][ ( pca ? fnumPCA : (fusion ? fnumFusion : fnum) ) ];
    Eigen::MatrixXf matchTimings[mnum];
    for(size_t i = 0; i < mnum; ++i)
        matchTimings[i] = Eigen::MatrixXf(targets.size(), (pca ? fnumPCA : fnum));
    
    for(size_t i = 0; i < targets.size(); ++i) {
        const boost::filesystem::path tpath(targets[i]);
        const std::string pscn = tpath.stem().string();
        
        if(verbose)
            COVIS_MSG_INFO("---------- PROCESSING SCENE " << i+1 << "/" << targets.size() <<
                    " (" << pscn << ") ----------");
        
        CloudT::Ptr surft(new CloudT);
        pcl::PolygonMesh::Ptr mesht(new pcl::PolygonMesh);
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
        if(decimation > 0.0 && decimation < 1.0) {
            if(verbose)
                COVIS_MSG_INFO("Decimating scene...");
            decimate(*mesht, decimation);
            pcl::fromPCLPointCloud2(mesht->cloud, *surft);
        }
        
        // Now compute inlier threshold using scene resolution
        if(verbose)
            COVIS_MSG_INFO("Estimating scene mesh resolution...");
        const double sceneRes = resolution(mesht, surft);

        const double thres = thresMul * sceneRes;
        if(verbose)
            COVIS_MSG_INFO("Setting inlier threshold to: " << thres);
        const double thressq = thres * thres;
        
        // TODO: Save decimated/noisy mesh
        if(!dryrun) {
            if(decimation > 0.0 && decimation < 1.0) {
                const std::string pathSurf = outputDirScenes + "/" + tpath.stem().string() +
                        "_surface" + tpath.extension().string();
                if(verbose)
                    COVIS_MSG("Saving decimated scene mesh \"" << pathSurf << "\"...");
                pcl::io::savePLYFileBinary(pathSurf, *mesht);
            }
        }
        
        // Generate search
        detect::PointSearch<PointT> surftSearch;
        surftSearch.setTarget(surft);
        
        // Virtual scene: all seed points placed in the scene by their GT pose
        CloudT::Ptr queriesTransformed(new CloudT);
        bool queryMask[queries.size()];
        
        // All object features stacked, minus invalid objects (due to bad pose or too much occlusion, see below)
        std::vector<MatrixT> featqScene(fnum);
        size_t validObjects = 0;
        
        for(size_t j = 0; j < queries.size(); ++j) {
            // Resolve pose file
            const std::string pobj = boost::filesystem::path(queries[j]).stem().string();
            const std::string poseFile = (poseFileModelFirst ?
                    poseDir + "/" + pobj + poseSeparator + pscn + "." + poseSuffix : 
                    poseDir + "/" + pscn + poseSeparator + pobj + "." + poseSuffix);
            
            // Load pose
            Eigen::Matrix4f pose;
            try {
                core::read(poseFile, pose);
            } catch(const std::exception& e) {
                COVIS_MSG_WARN("Object \"" << pobj << "\" is left out when testing scene \"" << pscn << "\"!");
                queryMask[j] = false;
                continue;
            }
            
            // Transform object
            CloudT::Ptr queryTransformed(new CloudT);
            pcl::transformPointCloud<PointT>(*query[j], *queryTransformed, pose);
            
            // TODO: Check that at least some fraction of the object is visible (has a neighbor up to threshold)
            core::Correspondence::VecPtr pointCorr = surftSearch.knn(queryTransformed, 1);
            size_t numInliers = 0;
            for(core::Correspondence::Vec::const_iterator it = pointCorr->begin(); it != pointCorr->end(); ++it)
                if(it->distance[0] <= thressq)
                    ++numInliers;
            
            queryMask[j] = (numInliers >= size_t(0.05 * queryTransformed->size()));
            if(!queryMask[j]) {
                COVIS_MSG_WARN("Too few valid points (" << numInliers << "/" << queryTransformed->size() <<
                        ") for object \"" << pobj << "\" in scene \"" << pscn << "\"" << "!");
                continue;
            }
            
            // Now add points/features to the virtual scene
            *queriesTransformed += *queryTransformed;
            for(size_t k = 0; k < fnum; ++k) {
                const size_t rowsFirst = featqScene[k].rows();
                const size_t rowsSecond = featq[j][k].rows();
                const size_t cols = featq[j][k].cols();
                COVIS_ASSERT(rowsFirst + rowsSecond > 0 && cols > 0);
                featqScene[k].conservativeResize(rowsFirst + rowsSecond, cols);
                featqScene[k].block(rowsFirst, 0, rowsSecond, cols) = featq[j][k];
            }
            
            ++ validObjects;
        } // End local loop over queries (j)
        
        // Check that we have the correct number of features
        for(size_t k = 0; k < fnum; ++k)
            COVIS_ASSERT(size_t(featqScene[k].rows()) == queriesTransformed->size());
        
        if(queriesTransformed->empty()) {
            COVIS_MSG_WARN("No objects with ground truth information found for scene \"" << pscn << "\"! Skipping...");
            featureNumbers(i, 0) = featureNumbers(i, 1) = 0;
            continue;
        }
        
        /**
         * Find scene seed points
         */
        // Select seed points on the scene surface as those points that are close to object points up to threshold
        CloudT::Ptr target(new CloudT);
        target->reserve(queriesTransformed->size());
        core::Correspondence::VecPtr corrQueryTarget = surftSearch.knn(queriesTransformed, 1);
        std::vector<bool> queryTargetMask(queriesTransformed->size()); // Only for non-overlap removal
        for(size_t j = 0; j < corrQueryTarget->size(); ++j) {
            queryTargetMask[j] = ((*corrQueryTarget)[j].distance[0] <= thressq); // Set to true if query point has a valid target correspondence
            if(queryTargetMask[j])
                target->push_back(surft->points[(*corrQueryTarget)[j].match[0]]);
        }
        
//        show(mesht, target);
        
        /**
         * TODO: In case of scene-scene matching, also remove non-overlapping data from query
         */
        if(removeNonOverlap) {
            COVIS_MSG_WARN("Non-overlap removal enabled - query seeds are now altered!");
            COVIS_ASSERT_MSG(!matchSceneObjects, "Non-overlap removal only makes sense for query --> target matching direction!");
            COVIS_ASSERT_MSG(targets.size() == 1 && queries.size() == 1, "Non-overlap removal only makes sense for single scene-scene data!");

            CloudT queryNonOverlap;
            CloudT queriesTransformedNonOverlap;
            queryNonOverlap.reserve(query[0]->size());
            queriesTransformedNonOverlap.reserve(queriesTransformed->size());
            for(size_t j = 0; j < queryTargetMask.size(); ++j) {
                if(queryTargetMask[j]) {
                    queryNonOverlap.push_back(query[0]->points[j]);
                    queriesTransformedNonOverlap.push_back(queriesTransformed->points[j]);
                }
            }
            *query[0] = queryNonOverlap;
            *queriesTransformed = queriesTransformedNonOverlap;
            
            for(size_t k = 0; k < fnum; ++k) {
                MatrixT featqSceneNonOverlap;
                const MatrixT::Index cols = featqScene[k].row(0).cols();
                for(size_t j = 0; j < queryTargetMask.size(); ++j) {
                    if(queryTargetMask[j]) {
                        featqSceneNonOverlap.conservativeResize(featqSceneNonOverlap.rows() + 1, cols);
                        featqSceneNonOverlap.row(featqSceneNonOverlap.rows() - 1) = featqScene[k].row(j);
                    }
                }
                featqScene[k] = featqSceneNonOverlap;
            }

            if(!dryrun) {
                const boost::filesystem::path qpath(queries[0]);
                const std::string pathSeed = outputDirObjects + "/" + qpath.stem().string() +
                        "_seeds" + qpath.extension().string();
                COVIS_MSG_WARN("Re-saving query seeds to \"" << pathSeed << "\"...");
                pcl::io::savePLYFileBinary(pathSeed, queryNonOverlap);
            }
        }
        
        // Save query/target number of points
        if(matchSceneObjects) {
            featureNumbers(i, 0) = target->size();
            featureNumbers(i, 1) = queriesTransformed->size();
        } else {
            featureNumbers(i, 0) = queriesTransformed->size();;
            featureNumbers(i, 1) = target->size();
        }

        // Save scene seed points
        if(!dryrun) {
            const std::string pathSeed = outputDirScenes + "/" + tpath.stem().string() +
                    "_seeds" + tpath.extension().string();
            if(verbose)
                COVIS_MSG("Saving scene mesh seeds \"" << pathSeed << "\"...");
            pcl::io::savePLYFileBinary(pathSeed, *target);
        }
        
        /**
         * Compute scene features
         */
        if(verbose)
            COVIS_MSG_INFO("Computing " << target->size() << " target features...");
        std::vector<MatrixT> featt(fnum);
        Eigen::VectorXf row(fnum);
        features(mesht, surft, target, frad, sceneRes,//avgModelMeshRes,
                featt,
                row);
        
        // Generate the avg. support size of a feature
        if(verbose)
            COVIS_MSG_INFO("Avg. support sizes:");
        
        for(size_t j = 0; j < fnum; ++j) {
            core::Correspondence::VecPtr c = surftSearch.radius(target, frad[j]);
            size_t nnum = 0;
            for(size_t k = 0; k < c->size(); ++k)
                nnum += (*c)[k].match.size();
            nnum /= c->size();
            
            if(verbose)
                COVIS_MSG("\t" << fnames[j] << ": " << nnum);
            
            featureNumbers(i,2+j) = nnum;
        }
        
        // Print the computation times
        if(verbose) {
            COVIS_MSG_INFO("Total feature timings:");
            for(size_t j = 0; j < fnum; ++j)
                COVIS_MSG("\t" << fnames[j] << ": " << row(j) << " s");
        }
        
        // Just set equal feature timings for all PCA versions
        if(pca) {
            size_t cnt = 0;
            for(size_t j = 0; j < fnum; ++j)
                for(size_t k = 0; k < boost::size(pcaVariations); ++k)
                    featureTimings(i,cnt++) = row(j);
        } else {
            featureTimings.row(i) = row;
        }
        
        /**
         * Compute PCA subspaces
         */
        if(pca) {
            // Now update the features
            if(verbose)
                COVIS_MSG_INFO("Projecting all query features to PCA subspaces...");
            Eigen::VectorXf projectionTiming;
            projectionTiming.setZero(fnum);
            pcaProject(featqScene, fnames, pcas, components, projectionTiming, verbose);

            if(verbose)
                COVIS_MSG_INFO("Projecting all target features to PCA subspaces...");
            pcaProject(featt, fnames, pcas, components, projectionTiming, verbose);
        }
        
        /**
         * Perform matching
         */
        if(verbose) {
            if(matchSceneObjects)
                COVIS_MSG_INFO("Matching a total of " << target->size() << " --> " << queriesTransformed->size() <<
                        " features from scene --> " << validObjects << " objects...");
            else
                COVIS_MSG_INFO("Matching a total of " << queriesTransformed->size() << " --> " << target->size() <<
                        " features from " << validObjects << " objects --> scene...");
        }
        
        // Loop over all metrics
        for(size_t j = 0; j < mnum; ++j) {
            const DISTANCE_METRIC mj = str2m(mnames[j]);
            if(verbose)
                COVIS_MSG(m2str(mj) << "...");
            
            // Find feature correspondences
            std::vector<core::Correspondence::VecPtr> featureCorr( (pca ? fnumPCA : fnum) );    
#ifdef _OPENMP
#pragma omp parallel for
#endif
            for(size_t k = 0; k < (pca ? fnumPCA : fnum); ++k) {
                TIME(\
                        if(matchSceneObjects)
                            featureCorr[k] = match(featt[k], featqScene[k], mj);
                        else
                            featureCorr[k] = match(featqScene[k], featt[k], mj);
                        ,
                        matchTimings[j](i,k)
                );
            } // End loop over all features types (k)
            
            // Print match timings
            if(verbose)
                for(size_t k = 0; k < (pca ? fnumPCA : fnum); ++k)
                    COVIS_MSG("\t" << (pca ? fnamesPCA[k] : fnames[k]) << ": " << matchTimings[j](i,k) << " s");
            
            /**
             * Fuse matches - modifies featureCorr to contain fnumFusion entries
             */
            Eigen::VectorXf fusionTimings;
            if(fusion)
                fuse(featureCorr, fusionCombinations, fusionTimings);
          /**************************************************************************************/  
	  
	  if(correspondences){
            // TODO: Show correspondences
            for(size_t k = 0; k < (pca ? fnumPCA : (fusion ? fnumFusion : fnum)); ++k) {
                if(!query.size() == 1)
                    continue;
                
                visu::CorrVisu<PointT> v;
                
                v.setTitle((pca ? fnamesPCA[k] : (fusion ? fnamesFusion[k] : fnames[k])) + " correspondences");
                v.setBackgroundColor(255, 255, 255);
                const Eigen::Vector3f t = v.separate(query[0], target, -1.5);

                pcl::PolygonMesh::Ptr meshqt(new pcl::PolygonMesh);
                CloudT::Ptr cloudqt(new CloudT);
                *meshqt = *meshq[0];
                pcl::fromPCLPointCloud2<PointT>(meshqt->cloud, *cloudqt);
                pcl::transformPointCloud<PointT>(*cloudqt, *cloudqt, t, Eigen::Quaternionf::Identity());
                pcl::toPCLPointCloud2<PointT>(*cloudqt, meshqt->cloud);
                
                CloudT::Ptr queryt(new CloudT);
                pcl::transformPointCloud<PointT>(*query[0], *queryt, t, Eigen::Quaternionf::Identity());
                
                v.setQuery(queryt);
                v.setTarget(target);
                v.setShowPoints(false),
                v.addMesh(meshqt, "object");
                v.addMesh(mesht, "scene");
                v.addColor<PointT>(queryt, 255, 0, 0, "query-seeds");
                v.addColor<PointT>(target, 255, 0, 0, "target-seeds");
                
                core::Correspondence::Vec corrCopy = *featureCorr[k];
                core::sort(corrCopy);
                corrCopy.resize(100);
                
                
                v.setCorr(corrCopy);
                v.setLevel(1);
                v.show();
            }
	  }
            /**************************************************************************************/
            /**
             * Resolving correct matches
             */
            if(verbose)
                COVIS_MSG_INFO("Resolving " << (pca ? "PCA" : (fusion ? "fusion" : "")) << " inliers...");
            
            // Setup a correspondence filter with an identity transformation (because objects are already transformed)
            detect::CorrespondenceFilterTransform<PointT> filter;
            if(matchSceneObjects) {
                filter.setQuery(target);
                filter.setTarget(queriesTransformed);
            } else {
                filter.setQuery(queriesTransformed);
                filter.setTarget(target);
            }
            filter.setThreshold(thres);
            filter.setTransformation(Eigen::Matrix4f::Identity());
            
            // Resolve inliers
            core::Correspondence::VecPtr featureInliers[(pca ? fnumPCA : (fusion ? fnumFusion : fnum))];
            std::vector<bool> featureMask[(pca ? fnumPCA : (fusion ? fnumFusion : fnum))];
            for(size_t k = 0; k < (pca ? fnumPCA : (fusion ? fnumFusion : fnum)); ++k) {
                core::sort(*featureCorr[k]);
                featureInliers[k] = filter.filter(*featureCorr[k]); // Only for printing below
                featureMask[k] = filter.getMask();
                COVIS_ASSERT(featureMask[k].size() == featureCorr[k]->size());
                
                if(verbose) {
                    if(pca)
                        COVIS_MSG("\t" << fnamesPCA[k] << ": " << featureInliers[k]->size() << "/" << featureCorr[k]->size());
                    else if(fusion)
                        COVIS_MSG("\t" << fnamesFusion[k] << ": " << featureInliers[k]->size() << "/" << featureCorr[k]->size());
                    else
                        COVIS_MSG("\t" << fnames[k] << ": " << featureInliers[k]->size() << "/" << featureCorr[k]->size());
                }
            }
            
            // Add to matching output
            for(size_t k = 0; k < (pca ? fnumPCA : (fusion ? fnumFusion : fnum)); ++k) {
                const Eigen::MatrixXf::Index offset = matchingOutput[j][k].rows();
                matchingOutput[j][k].conservativeResize(offset + Eigen::MatrixXf::Index(featureCorr[k]->size()), 2);
                for(Eigen::MatrixXf::Index l = 0; l < Eigen::MatrixXf::Index(featureCorr[k]->size()); ++l)
                    matchingOutput[j][k].row(offset + l) =
                            Eigen::Vector2f((*featureCorr[k])[l].distance[0], float(featureMask[k][l]));
            }
        } // End loop over metrics to find feature matches (j)
    } // End loop over all scenes (i)
    
    /*
     * Generate output files
     */
    if(!dryrun) {
        // Write matching output [distance inlier_status] and timings
        for(size_t i = 0; i < mnum; ++i) {
            const DISTANCE_METRIC mi = str2m(mnames[i]);
            
            for(size_t j = 0; j < (pca ? fnumPCA : (fusion ? fnumFusion : fnum)); ++j) {
                // TODO: Don't use any feature radius suffix in PCA/fusion cases
                const std::string suffixMatchingOutput = (fusion ? 
                        m2str(mi) + ".txt" :
                        core::stringify(fradMul[j]) + "_" + m2str(mi) + ".txt");
                
                std::string name;
                if(pca)
                    name = fnamesPCA[j];
                else if(fusion)
                    name = fnamesFusion[j];
                else
                    name = fnames[j];
                core::write(outputDir + "/matching_output_" + name + "_" + suffixMatchingOutput, matchingOutput[i][j], true, true, append);
            }
            
            // TODO: Also store the matching results using all components

            // Write meta
            const std::string suffix = ".txt";
            core::write(outputDir + "/meta_feature_timings" + suffix, featureTimings, true, false, append);
            core::write(outputDir + "/meta_feature_numbers" + suffix, featureNumbers, true, false, append);
            core::write(outputDir + "/meta_match_timings" + suffix, matchTimings[i], true, false, append);
        }
    }
    
    return 0;
}
