// Boost
#include <boost/random/normal_distribution.hpp>

// PCL
#include <pcl/io/ply_io.h>

// Own
#include "../feature_matching_tools.h"

boost::random::mt19937 gen;

int main(int argc, char** argv) {
    if(argc != 4) {
        std::cout << "Usage: " << argv[0] << " <noise-factor> <input.ply> <output.ply>" << std::endl;
        return 0;
    }
    
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::io::loadPLYFile(argv[2], *mesh);
    
    CloudT::Ptr surf(new CloudT);
    pcl::fromPCLPointCloud2(mesh->cloud, *surf);
    
    const double noise = atof(argv[1]) * resolution(mesh, surf);
    
    const double sigma = noise; // Naive way, probably how others did it - power becomes noise*sqrt(3)
//    const double sigma = noise * sqrtf(1.0f / 3.0f); // To get a noise power of noise
    boost::random::normal_distribution<> dist(0.0, sigma);
    for(size_t j = 0; j < surf->size(); ++j)
        surf->points[j].getVector3fMap() += Eigen::Vector3f(dist(gen), dist(gen), dist(gen));
    
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::copyPointCloud(*surf, tmp);
    pcl::toPCLPointCloud2(tmp, mesh->cloud);

    pcl::io::savePLYFileBinary(argv[3], *mesh);
}
