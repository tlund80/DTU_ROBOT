#ifndef FEATURE_MATCHING_TOOLS_H
#define FEATURE_MATCHING_TOOLS_H

// Own
#include <pcl/PolygonMesh.h>

// Covis
#include <covis/core/correspondence.h>

// VTK
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include "feature_matching_features.h"
    
// Macro for timing one or more commands 
#define TIME(cmds, out)\
    { const int64 t = cv::getTickCount();\
    cmds\
    out = double(cv::getTickCount() - t) / cv::getTickFrequency(); }

void show(pcl::PolygonMesh::ConstPtr mesh,
        CloudT::ConstPtr seeds = boost::make_shared<CloudT>(),
        const std::string& title = "Mesh");

double resolution(pcl::PolygonMesh::ConstPtr mesh, CloudT::ConstPtr surf);

void vtk2mesh(vtkSmartPointer<vtkPolyData> vtkpoly, pcl::PolygonMesh& mesh, bool normals = true);

void normals(vtkSmartPointer<vtkPolyData>& vtkpoly);

void normals(pcl::PolygonMesh& mesh);

void decimate(pcl::PolygonMesh& mesh, float decimation);

void seed(CloudT::ConstPtr surf, double res, CloudT::Ptr& result);

void pcaTrain(const std::vector<std::vector<MatrixT> >& features,
        const std::vector<std::string>& fnames,
        const std::vector<float>& variations,
        std::vector<cv::PCA>& pcas,
        std::vector<std::vector<size_t> >& components,
        bool verbose = false);

void pcaProject(std::vector<MatrixT>& features,
        const std::vector<std::string>& fnames,
        const std::vector<cv::PCA>& pcas,
        std::vector<std::vector<size_t> >& components,
        Eigen::VectorXf& timings,
        bool verbose = false);

void fuse(std::vector<covis::core::Correspondence::VecPtr>& featureCorr,
        const std::vector<boost::tuple<int,int,int> >& fusionCombinations,
        Eigen::VectorXf& timings);

#endif
