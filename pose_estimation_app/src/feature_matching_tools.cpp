#include "feature_matching_tools.h"

// Covis
#include <covis/detect/point_search.h>
#include <covis/visu/visu_3d.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

// VTK
#include <vtkCellArray.h>
#include <vtkDecimatePro.h>
#include <vtkFloatArray.h>
#include <vtkPLYReader.h>
#include <vtkPointData.h>
#include <vtkPolyDataNormals.h>
#include <vtkQuadricDecimation.h>
#include <vtkVersion.h>

using namespace covis;

void show(pcl::PolygonMesh::ConstPtr mesh,
        CloudT::ConstPtr seeds,
        const std::string& title) {
    visu::Visu3D v(title);
    v.visu.setCameraPosition(0, 0, -0.5, 0, 0, 0, 0, 1, 0); // [camera focal up]
    v.setBackgroundColor(255, 255, 255);
    v.addMesh(mesh, title);
    if(seeds) {
        v.addColor<PointT>(seeds, 255, 0, 0, "seeds");
        v.visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "seeds");
    }
    v.show();
}

double resolution(pcl::PolygonMesh::ConstPtr mesh, CloudT::ConstPtr surf) {
    double accu = 0.0;
    size_t count = 0;
    
    for(size_t j = 0; j < mesh->polygons.size(); ++j) {
        COVIS_ASSERT_MSG(mesh->polygons[j].vertices.size() == 3, "Not a triangular mesh!");
        for(size_t k = 0; k < mesh->polygons[j].vertices.size(); ++k) {
            if(k < mesh->polygons[j].vertices.size() - 1) {
                accu += (surf->points[mesh->polygons[j].vertices[k]].getVector3fMap() -
                        surf->points[mesh->polygons[j].vertices[k + 1]].getVector3fMap()).norm();
            } else {
                accu += (surf->points[mesh->polygons[j].vertices[k]].getVector3fMap() -
                        surf->points[mesh->polygons[j].vertices[0]].getVector3fMap()).norm();
            }
            ++ count;
        }
    }
    
    return accu / double(count);
}

void vtk2mesh(vtkSmartPointer<vtkPolyData> vtkpoly, pcl::PolygonMesh& mesh, bool normals) {
    // Copy polygons to PCL
    mesh.polygons.resize(vtkpoly->GetNumberOfPolys());
    vtkIdType* cellPoints;
    vtkIdType nrCellPoints;
    vtkCellArray* meshPolygons = vtkpoly->GetPolys();
    meshPolygons->InitTraversal ();
    size_t idpoly = 0;
    while (meshPolygons->GetNextCell(nrCellPoints, cellPoints)) {
      mesh.polygons[idpoly].vertices.resize(nrCellPoints);
      for(vtkIdType i = 0; i < nrCellPoints; ++i)
        mesh.polygons[idpoly].vertices[i] = size_t(cellPoints[i]);
      ++idpoly;
    }

    // Get points and normals
    vtkSmartPointer<vtkPoints> p = vtkpoly->GetPoints();
    vtkSmartPointer<vtkFloatArray> n;
    if(normals) {
        n = vtkFloatArray::SafeDownCast(vtkpoly->GetPointData()->GetNormals());
        COVIS_ASSERT(p->GetNumberOfPoints() == n->GetNumberOfTuples());
    }
    
    // Copy to a normal point cloud
    CloudT cloudn;
    pcl::PointCloud<pcl::PointXYZ> cloudxyz;
    if(normals)
        cloudn.resize(p->GetNumberOfPoints());
    else
        cloudxyz.resize(p->GetNumberOfPoints());
    for(vtkIdType i = 0; i < p->GetNumberOfPoints(); ++i) {
        if(normals) {
            cloudn[i].x = p->GetPoint(i)[0];
            cloudn[i].y = p->GetPoint(i)[1];
            cloudn[i].z = p->GetPoint(i)[2];
            cloudn[i].normal_x = n->GetTuple(i)[0];
            cloudn[i].normal_y = n->GetTuple(i)[1];
            cloudn[i].normal_z = n->GetTuple(i)[2];
        } else {
            cloudxyz[i].x = p->GetPoint(i)[0];
            cloudxyz[i].y = p->GetPoint(i)[1];
            cloudxyz[i].z = p->GetPoint(i)[2];
        }
    }
    
    // Copy normal point cloud to PCL mesh
    if(normals)
        pcl::toPCLPointCloud2(cloudn, mesh.cloud);
    else
        pcl::toPCLPointCloud2(cloudxyz, mesh.cloud);
}

void normals(vtkSmartPointer<vtkPolyData>& vtkpoly) {
    vtkSmartPointer<vtkPolyDataNormals> pdn = vtkPolyDataNormals::New();
    pdn->ComputePointNormalsOn();
//    pdn->AutoOrientNormalsOn(); // Does not work entirely
//    pdn->NonManifoldTraversalOn();
    pdn->SplittingOff();
    pdn->SetInput(vtkpoly);
    pdn->Update();
    vtkpoly = pdn->GetOutput();
}

void normals(pcl::PolygonMesh& mesh) {
    vtkSmartPointer<vtkPolyData> vtkpoly;
    pcl::VTKUtils::convertToVTK(mesh, vtkpoly);
    normals(vtkpoly);
    vtk2mesh(vtkpoly, mesh);
}

void decimate(pcl::PolygonMesh& mesh, float decimation) {
    // Convert from PCL mesh representation to the VTK representation
    vtkSmartPointer<vtkPolyData> vtkpoly;
    pcl::VTKUtils::convertToVTK(mesh, vtkpoly);

    // Apply the decimation algorithm
    vtkSmartPointer<vtkQuadricDecimation> filter = vtkSmartPointer<vtkQuadricDecimation>::New();
//    vtkSmartPointer<vtkDecimatePro> filter = vtkSmartPointer<vtkDecimatePro>::New();
    filter->SetTargetReduction (1.0f - decimation);
  #if VTK_MAJOR_VERSION < 6
    filter->SetInput (vtkpoly);
  #else
    filter->SetInputData (vtkpoly);
  #endif
    filter->Update ();
    
    // Reestimating normals here because decimation removes normals
    vtkpoly = filter->GetOutput();
    normals(vtkpoly);

    // Convert the result back to the PCL representation
    vtk2mesh(vtkpoly, mesh);
}

void seed(CloudT::ConstPtr surf, double res, CloudT::Ptr& result) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(surf);
    vg.setLeafSize(res, res, res);
    result.reset(new CloudT);
    vg.filter(*result);
    
    // Project the points to the original surface
    core::Correspondence::VecPtr c = detect::knnSearch<PointT>(result, surf, 1);
    for(size_t i = 0; i < result->size(); ++i)
        result->points[i] = surf->points[(*c)[i].match[0]];
}

void pcaTrain(const std::vector<std::vector<MatrixT> >& features,
        const std::vector<std::string>& fnames,
        const std::vector<float>& variations,
        std::vector<cv::PCA>& pcas,
        std::vector<std::vector<size_t> >& components,
        bool verbose) {
    COVIS_ASSERT(!features.empty()); // Outer dimension is the number of models
    
    const size_t fnum = fnames.size(); // Equal to the inner dimension of 'features'
    
    // Outputs
    pcas.resize(fnum);
    components.resize(fnum, std::vector<size_t>(variations.size())); // Number of components for the chosen variations
    
    // One PCA per feature type
    for(size_t i = 0; i < fnum; ++i) {
        if(verbose)
            COVIS_MSG("Performing PCA training for " << fnames[i] << "...");
        
        // Stack the features of all models
        cv::Mat_<float> featcv;
        for(size_t j = 0; j < features.size(); ++j) {
            COVIS_ASSERT(features[j].size() == fnum);
            cv::Mat_<float> featij(features[j][i].rows(), features[j][i].cols(), const_cast<float*>(features[j][i].data()));
            if(featcv.cols == 0) {
                featij.copyTo(featcv);
            } else {
                cv::Rect bottom(0, featcv.rows, featcv.cols, featij.rows);
                featcv.resize(featcv.rows + featij.rows);
                featij.copyTo(featcv(bottom));
            }
        }
        
        // Perform PCA for this type of feature
        COVIS_ASSERT_MSG(cv::checkRange(featcv), "NaN or inf found in features!"); // Check for NaN
        pcas[i] = cv::PCA(featcv, cv::noArray(), CV_PCA_DATA_AS_ROW);
        
        // Compute the energies
        cv::Mat_<float> energies = pcas[i].eigenvalues;
        for(int j = 1; j < energies.rows; ++j)
            energies(j,0) += energies(j-1,0);
        energies /= energies(energies.rows - 1, 0);
        
        // Find the subspace dimensions covering the energies
        for(size_t j = 0; j < variations.size(); ++j) {
            components[i][j] = 0;
            for(int k = 0; k < energies.rows; ++k)
                if(energies(k,0) < variations[j] / 100.0f)
                    ++components[i][j];
            COVIS_ASSERT(components[i][j] > 0);
            if(verbose)
                COVIS_MSG("\t" << variations[j] << " % energy: " << components[i][j] << " components");
        }
    } // End loop over feature types (i)
}

void pcaProject(std::vector<MatrixT>& features,
        const std::vector<std::string>& fnames,
        const std::vector<cv::PCA>& pcas,
        std::vector<std::vector<size_t> >& components,
        Eigen::VectorXf& timings,
        bool verbose) {
    timings.resize(features.size());
    std::vector<MatrixT> featuresPCA;
    for(size_t i = 0; i < features.size(); ++i) {
        if(verbose)
            COVIS_MSG("\t" << fnames[i] << "...");
        cv::Mat_<float> featcv(features[i].rows(), features[i].cols(), features[i].data());
        // Project
        cv::Mat_<float> featcvPCA;
        TIME(\
                pcas[i].project(featcv, featcvPCA);,
                timings[i]
            );
        if(verbose)
            COVIS_MSG("\t\t" << timings[i] << "s");
        // Map back to Eigen
        MatrixT tmp = Eigen::Map<MatrixT,Eigen::RowMajor>(featcvPCA.ptr<float>(), featcvPCA.rows, featcvPCA.cols);
        // Take columns corresponding to number of components
        for(size_t j = 0; j < components[i].size(); ++j)
            featuresPCA.push_back(tmp.block(0, 0, tmp.rows(), components[i][j]));
    }
    
    features = featuresPCA;
}

void fuse(std::vector<core::Correspondence::VecPtr>& featureCorr,
        const std::vector<boost::tuple<int,int,int> >& fusionCombinations,
        Eigen::VectorXf& timings) {
    std::vector<core::Correspondence::VecPtr> featureCorrFusion(fusionCombinations.size());
    timings.resize(fusionCombinations.size());
    for(size_t k = 0; k < fusionCombinations.size(); ++k) {
        TIME(\
            int idx1 = fusionCombinations[k].get<0>();
            int idx2 = fusionCombinations[k].get<1>();
            int idx3 = fusionCombinations[k].get<2>();
            featureCorrFusion[k].reset(new core::Correspondence::Vec(featureCorr[idx1]->size()));
            for(size_t l = 0; l < featureCorr[idx1]->size(); ++l) {
                const float d1 = (*featureCorr[idx1])[l].distance[0];
                const float d2 = (*featureCorr[idx2])[l].distance[0];
                const float d3 = (idx3 == -1 ? -1 : (*featureCorr[idx3])[l].distance[0]);
                if(idx3 == -1) // Pair
                    (*featureCorrFusion[k])[l] = ( d1 < d2 ? (*featureCorr[idx1])[l] : (*featureCorr[idx2])[l] );
                else // Triplet
                    (*featureCorrFusion[k])[l] = ( d1 < d2 ?
                            (d1 < d3 ? (*featureCorr[idx1])[l] : (*featureCorr[idx3])[l]) :
                            (d2 < d3 ? (*featureCorr[idx2])[l] : (*featureCorr[idx3])[l]) );
            },
            timings[k]
        );
    }
    
    featureCorr = featureCorrFusion;
}
