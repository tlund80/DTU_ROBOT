// Boost
#include <boost/algorithm/string/predicate.hpp>

// Covis
#include <covis/core/stat.h>
using namespace covis;

// PCL
#include <pcl/io/ply_io.h>

// VTK
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkDelaunay2D.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>

// Own
#include "../feature_matching_tools.h"
#include "../feature_matching_features.h"

int main(int argc, char** argv) {
    if(argc < 3 || argc > 5) {
        std::cout << "Usage: " << argv[0] << " <input.ply|pcd> <output.ply> [<edge_tol_multiplier> <flip_faces>]" << std::endl;
        return 0;
    }
    
    const float tol = (argc == 4 ? atof(argv[3]) : 1);
    const bool flip = (argc == 5 ? atoi(argv[4]) : false);
        
    if(!boost::iends_with(argv[2], ".ply"))
        COVIS_THROW("Invalid extension for output file " << argv[2]);
    
    // Load mesh and point cloud
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    CloudT::Ptr surf(new CloudT);
    if(boost::iends_with(argv[1], ".ply")) {
        COVIS_MSG_INFO("Loading point cloud from PLY file " << argv[1] << "...");
        pcl::io::loadPLYFile(argv[1], *mesh);
        pcl::fromPCLPointCloud2(mesh->cloud, *surf);
    } else if(boost::iends_with(argv[1], ".pcd")) {
        COVIS_MSG_INFO("Loading point cloud from PCD file " << argv[1] << "...");
        pcl::io::loadPCDFile(argv[1], *surf);
        pcl::toPCLPointCloud2(*surf, mesh->cloud);
    } else {
        COVIS_THROW("Invalid extension for input file " << argv[1]);
    }
    
    // Insert projected xy data into polydata
    vtkSmartPointer<vtkPolyData> poly = vtkPolyData::New();
    vtkSmartPointer<vtkPoints> points = vtkPoints::New();
    for(size_t i = 0; i < surf->size(); ++i)
        points->InsertNextPoint(surf->points[i].x / surf->points[i].z, surf->points[i].y / surf->points[i].z, 0.0);
    poly->SetPoints(points);

    // Perform triangulation
    COVIS_MSG_INFO("Triangulating...");
    vtkSmartPointer<vtkDelaunay2D> del = vtkDelaunay2D::New();;
    del->SetInput(poly);
    del->Update();
    vtkSmartPointer<vtkPolyData> tri = del->GetOutput();
    COVIS_ASSERT(size_t(tri->GetNumberOfPoints()) == surf->size());
    
    // Reconstruct the point cloud data
    points = tri->GetPoints();
    for(size_t i = 0; i < surf->size(); ++i)
        points->SetPoint(i, surf->points[i].x, surf->points[i].y, surf->points[i].z);
    
    // Get triangles
    vtkSmartPointer<vtkCellArray> polys = tri->GetPolys();
    
    // Compute resolution in 3D
    vtkIdType npts;
    vtkIdType* pts = new vtkIdType[3];
    std::vector<float> distances;
    distances.reserve(3 * polys->GetNumberOfCells());
    for(vtkIdType i = 0; i < polys->GetNumberOfCells(); ++i) {
        polys->GetNextCell(npts, pts);
        COVIS_ASSERT(npts == 3);
        const Eigen::Vector3f p1 = surf->points[pts[0]].getVector3fMap();
        const Eigen::Vector3f p2 = surf->points[pts[1]].getVector3fMap();
        const Eigen::Vector3f p3 = surf->points[pts[2]].getVector3fMap();
        distances.push_back((p1 - p2).norm());
        distances.push_back((p2 - p3).norm());
        distances.push_back((p3 - p1).norm());
    }
    const float mean = core::mean(distances);
    const float std = core::stddev(distances, mean);
    
    // Compute tolerance in metric units
    const float tolabs = mean + tol * std;
    
    COVIS_MSG_INFO("Removing large edge faces (tolerance = " << tolabs << " metric units)...");
    if(flip)
        COVIS_MSG_INFO("\tAlso flipping all faces!");

    // Remove large edge faces by 3D x, y and z coordinates (and flip if chosen)
    vtkSmartPointer<vtkCellArray> cellsFiltered = vtkCellArray::New();
    size_t numRemoved = 0;
    polys->InitTraversal();
    for(vtkIdType i = 0; i < polys->GetNumberOfCells(); ++i) {
        polys->GetNextCell(npts, pts);
        if(distances[3 * i] > tolabs || distances[3 * i + 1] > tolabs || distances[3 * i + 2] > tolabs) {
            ++numRemoved;
        } else {
            if(flip)
                std::swap(pts[0], pts[2]);
            cellsFiltered->InsertNextCell(npts, pts);
        }
    }
    COVIS_MSG("\tRemoved " << numRemoved << "/" << polys->GetNumberOfCells() << " triangles!");
    tri->SetPolys(cellsFiltered);
    
    if(numRemoved > 0) {
        COVIS_MSG_INFO("Removing unused points...");
        vtkSmartPointer<vtkCleanPolyData> clean = vtkCleanPolyData::New();
        clean->PointMergingOff();
        clean->SetInput(tri);
        clean->Update();
        vtkSmartPointer<vtkPolyData> polyClean = clean->GetOutput();
        COVIS_MSG("\tRemoved " << tri->GetNumberOfPoints() - polyClean->GetNumberOfPoints() << "/" << tri->GetNumberOfPoints() << " and " <<
                tri->GetNumberOfPolys() - polyClean->GetNumberOfPolys() << "/" << tri->GetNumberOfPolys() << " points and triangles!");
        tri = polyClean;
    }
    
    // Store
    COVIS_MSG_INFO("Done! Saving result...");
    pcl::PolygonMesh meshOut;
    vtk2mesh(tri, meshOut, false);
    pcl::io::savePLYFileBinary(argv[2], meshOut);
}
