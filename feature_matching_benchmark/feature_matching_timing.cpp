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
    po.addPositional("target", "scene/target model files (PLY)");
    po.addPositional("output-directory", "path to output directory");
    
    // Features
    po.addOption("features", "all", "select a subset of features - in case of 'all', all features are enabled.");
    po.addOption("radius", 'r', 0.02, "feature radii, given as an absolute distance");
    
    // General
    po.addFlag('p', "append", "append data to output files");
    po.addFlag('v', "verbose", "print debug information");
    po.addFlag('o', "omp", "use OpenMP, if available");
    po.addFlag('u', "dry-run", "don't save any outputs");
    
    // Parse
    if(!po.parse(argc, argv))
        return 1;

    const bool verbose = po.getFlag("verbose");
    
    if(verbose)
        po.print();

    // Get positionals
    const std::vector<std::string> targets = po.getVector("target");
    std::string outputDir = po.getValue("output-directory");
    COVIS_ASSERT(!targets.empty() && !outputDir.empty());

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
    std::vector<double> frad = po.getVector<double>("radius");
    if(frad.size() == 1 && fnum > 1) {
        COVIS_MSG_WARN("Only one radius specified - applying to all features!");
        frad.resize(fnum, frad[0]);
    }
    
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
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Start loop over all scenes
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Eigen::MatrixXi featureNumbers(targets.size(), 2 + fnum); // First two columns are target, only for compatibility, the rest columns number of neighbors
    Eigen::MatrixXf featureTimings(targets.size(), fnum);
    
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
        
        /**
         * Find scene seed points
         */
        // Select seed points on the scene surface
        CloudT::Ptr target = boost::make_shared<CloudT>(*surft);
        
        // Set feature numbers
        featureNumbers(i,0) = featureNumbers(i,1) = target->size();
        
//        show(mesht, target);
        
        if(verbose)
            COVIS_MSG_INFO("Estimating scene mesh resolution...");
        const double sceneRes = resolution(mesht, surft);
        
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

        detect::PointSearch<PointT> surftSearch;
        surftSearch.setTarget(surft);
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

        featureTimings.row(i) = row;
    } // End loop over all scenes (i)
    
    /*
     * Generate output files
     */
    if(!dryrun) {
        // Write timings
        const std::string suffix = ".txt";
        core::write(outputDir + "/meta_feature_timings" + suffix, featureTimings, true, false);
        core::write(outputDir + "/meta_feature_numbers" + suffix, featureNumbers, true, false);
    }
    
    return 0;
}
