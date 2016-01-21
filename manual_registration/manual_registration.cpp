#include "manual_registration.h"

// Qt
#include <QApplication>
#include <QEvent>
#include <QMessageBox>
#include <QMutexLocker>
#include <QObject>

// VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/distances.h>

using namespace pcl;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
ManualRegistration::ManualRegistration () {
    // Initialize bogus
    res_ = 0.001;
    cloud_src_present_ = false;
    cloud_dst_present_ = false;
    src_point_selected_ = false;
    dst_point_selected_ = false;
    color_toggle_ = false;
    trfm_computed_ = true;
    
    //Create a timer
    vis_timer_ = new QTimer (this);
    vis_timer_->start (5);//5ms

    connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot()));

    ui_ = new Ui::MainWindow;
    ui_->setupUi (this);

    this->setWindowTitle ("PCL Manual Registration");

    // Set up the source window
    vis_src_.reset (new pcl::visualization::PCLVisualizer ("", false));
    ui_->qvtk_widget_src->SetRenderWindow (vis_src_->getRenderWindow ());
    vis_src_->setupInteractor (ui_->qvtk_widget_src->GetInteractor (), ui_->qvtk_widget_src->GetRenderWindow ());
    vis_src_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    ui_->qvtk_widget_src->update ();

    vis_src_->registerPointPickingCallback (&ManualRegistration::SourcePointPickCallback, *this);

    // Set up the destination window
    vis_dst_.reset (new pcl::visualization::PCLVisualizer ("", false));
    ui_->qvtk_widget_dst->SetRenderWindow (vis_dst_->getRenderWindow ());
    vis_dst_->setupInteractor (ui_->qvtk_widget_dst->GetInteractor (), ui_->qvtk_widget_dst->GetRenderWindow ());
    vis_dst_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    ui_->qvtk_widget_dst->update ();

    vis_dst_->registerPointPickingCallback (&ManualRegistration::DstPointPickCallback, *this);


    // Connect all buttons
    //  connect (ui_->confirmSrcPointButton, SIGNAL(clicked()), this, SLOT(confirmSrcPointPressed()));
    //  connect (ui_->confirmDstPointButton, SIGNAL(clicked()), this, SLOT(confirmDstPointPressed()));
    connect (ui_->calculateButton, SIGNAL(clicked()), this, SLOT(calculatePressed()));
    connect (ui_->clearButton, SIGNAL(clicked()), this, SLOT(clearPressed()));
    connect (ui_->btnApplyTrfm, SIGNAL(clicked()), this, SLOT(applyTrfmPressed()));
    //  connect (ui_->orthoButton, SIGNAL(stateChanged(int)), this, SLOT(orthoChanged(int)));
    //  connect (ui_->applyTransformButton, SIGNAL(clicked()), this, SLOT(applyTransformPressed()));
    //  connect (ui_->refineButton, SIGNAL(clicked()), this, SLOT(refinePressed()));
    //  connect (ui_->undoButton, SIGNAL(clicked()), this, SLOT(undoPressed()));
    //  connect (ui_->safeButton, SIGNAL(clicked()), this, SLOT(safePressed()));

    cloud_src_modified_ = true; // first iteration is always a new pointcloud
    cloud_dst_modified_ = true;

    src_pc_.reset(new Cloud);
    dst_pc_.reset(new Cloud);
}

double ManualRegistration::ComputeCloudResolution(const ManualRegistration::CloudConstPtr &cloud){
 
 // PointCloud<PointXYZ>::Ptr xyz_source (new PointCloud<PointXYZ> ());
 // fromPCLPointCloud2 (*cloud, *xyz_source);
   PCL_INFO ("Computing cloud resolution ....\n");
  
   pcl::search::KdTree<PointT> s;
    const int k = 5;
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;

    s.setInputCloud(cloud);
    s.nearestKSearch(*cloud, std::vector<int>(), 5, idx, distsq);
    double res_src = 0.0f;
    for(size_t i = 0; i < cloud->size(); ++i) {
        double resi = 0.0f;
        for(int j = 1; j < k; ++j)
            resi += sqrtf(distsq[i][j]);
        resi /= double(k - 1);
        res_src += resi;
    }
    res_src /= double(cloud->size());
  
    PCL_INFO ("Cloud resolution = %f \n", res_src);
    
    return res_src;
}


double ManualRegistration::estimateOcclusion(const ManualRegistration::CloudConstPtr &model, const ManualRegistration::CloudConstPtr &scene){
  
  ///Occlusion = 1 - visible object area / total object area
  
    std::vector<float> distances;
    KdTreeFLANN<PointT>::Ptr tree (new KdTreeFLANN<PointT> ());
    tree->setInputCloud (scene);

    for (size_t point_i = 0; point_i < model->points.size (); ++ point_i)
    {
      if (!pcl_isfinite (model->points[point_i].x) || !pcl_isfinite (model->points[point_i].y) || !pcl_isfinite (model->points[point_i].z))
        continue;

      std::vector<int> nn_indices (1);
      std::vector<float> nn_distances (1);
      if (!tree->nearestKSearch (model->points[point_i], 1, nn_indices, nn_distances))
        continue;
      size_t point_nn_i = nn_indices.front();

      float dist = squaredEuclideanDistance (model->points[point_i], scene->points[point_nn_i]);
     
      /// Save the point if the distance is below 5 * resolution 
      if(dist < 2 * res_)
	distances.push_back(dist);
    }
    
    double occlusion = 1- (double(distances.size())/double(model->points.size()));

    PCL_INFO("Estimated occlusion = %f\n", occlusion);
    return occlusion;
    
}

double ManualRegistration::estimateClutter(const ManualRegistration::CloudConstPtr &model, const ManualRegistration::CloudConstPtr &scene){

  ///Clutter = 1 - Visible object area/total scene area
    std::vector<float> distances;
    KdTreeFLANN<PointT>::Ptr tree (new KdTreeFLANN<PointT> ());
    tree->setInputCloud (scene);

    for (size_t point_i = 0; point_i < model->points.size (); ++ point_i)
    {
      if (!pcl_isfinite (model->points[point_i].x) || !pcl_isfinite (model->points[point_i].y) || !pcl_isfinite (model->points[point_i].z))
        continue;

      std::vector<int> nn_indices (1);
      std::vector<float> nn_distances (1);
      if (!tree->nearestKSearch (model->points[point_i], 1, nn_indices, nn_distances))
        continue;
      size_t point_nn_i = nn_indices.front();

      float dist = squaredEuclideanDistance (model->points[point_i], scene->points[point_nn_i]);
     
      /// Save the point if the distance is below 2 * resolution 
      if(dist < 2 * res_)
	distances.push_back(dist);
    }
    
    double clutter = 1 - (double(distances.size())/double(scene->points.size()));

    PCL_INFO("Estimated clutter = %f\n", clutter);
    return clutter;
}

void ManualRegistration::SourcePointPickCallback (const pcl::visualization::PointPickingEvent& event, void*) {
    // Check to see if we got a valid point. Early exit.
    int idx = event.getPointIndex ();
    if (idx == -1)
        return;

    // Get the point that was picked
    event.getPoint (src_point_.x, src_point_.y, src_point_.z);
    PCL_INFO ("Src Window: Clicked point %d with X:%f Y:%f Z:%f\n", idx, src_point_.x, src_point_.y, src_point_.z);
    src_point_selected_ = true;

    if(src_point_selected_)
    {
        src_pc_->points.push_back(src_point_);
        PCL_INFO ("Selected %d source points\n", src_pc_->points.size());
        src_point_selected_ = false;
        src_pc_->width = src_pc_->points.size();
        
        std::ostringstream oss;
        oss << src_pc_->size();
        vis_src_->addSphere<PointT>(src_point_, 5 * res_, 0, 1, 0, "sphere_src_" + oss.str());

//        if(vis_src_->getCloudActorMap()->find("src_pc") == vis_src_->getCloudActorMap()->end()) {
//            // First time
//            vis_src_->addPointCloud(src_pc_,
//                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(src_pc_, 255, 0, 0),
//                    "src_pc");
//            vis_src_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "src_pc");
//        } else {
//            vis_src_->updatePointCloud(src_pc_,
//                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(src_pc_, 255, 0, 0),
//                    "src_pc");
//        }
    }
    else
    {
        PCL_INFO ("Please select a point in the source window first\n");
    }
}

void ManualRegistration::DstPointPickCallback (const pcl::visualization::PointPickingEvent& event, void*) {
    // Check to see if we got a valid point. Early exit.
    int idx = event.getPointIndex ();
    if (idx == -1)
        return;

    // Get the point that was picked
    event.getPoint (dst_point_.x, dst_point_.y, dst_point_.z);
    PCL_INFO ("Dst Window: Clicked point %d with X:%f Y:%f Z:%f\n", idx, dst_point_.x, dst_point_.y, dst_point_.z);
    dst_point_selected_ = true;

    if(dst_point_selected_)
    {
        dst_pc_->points.push_back(dst_point_);
        PCL_INFO ("Selected %d destination points\n", dst_pc_->points.size());
        dst_point_selected_ = false;
        dst_pc_->width = dst_pc_->points.size();
        
        std::ostringstream oss;
        oss << dst_pc_->size();
        vis_dst_->addSphere<PointT>(dst_point_, 5 * res_, 1, 0, 0, "sphere_dst_" + oss.str());

//        if(vis_dst_->getCloudActorMap()->find("dst_pc") == vis_dst_->getCloudActorMap()->end()) {
//            // First time
//            vis_dst_->addPointCloud(dst_pc_,
//                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(dst_pc_, 255, 0, 0),
//                    "dst_pc");
//            vis_dst_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "dst_pc");
//        } else {
//            vis_dst_->updatePointCloud(dst_pc_,
//                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(dst_pc_, 255, 0, 0),
//                    "dst_pc");
//        }
    }
    else
    {
        PCL_INFO ("Please select a point in the destination window first\n");
    }
}

void ManualRegistration::calculatePressed() {
    if(dst_pc_->points.size() != src_pc_->points.size())
    {
        PCL_INFO ("You haven't selected an equal amount of points, please do so!\n");
        QMessageBox::warning(this,
                QString("Warning"),
                QString("You haven't selected an equal amount of points, please do so!"));
        return;
    }

    const double voxel_size =  5 * res_;
    const double inlier_threshold_ransac =  2 * voxel_size;
    const double inlier_threshold_icp =  2 * voxel_size;

    CloudPtr cloud_src_ds(new Cloud);
    CloudPtr cloud_dst_ds(new Cloud);
    if(ui_->robustBox->isChecked() || ui_->refineBox->isChecked()) {
        PCL_INFO("Downsampling point clouds with a voxel size of %f...\n", voxel_size);
        pcl::VoxelGrid<PointT> grid;
        grid.setLeafSize(voxel_size, voxel_size, voxel_size);
        grid.setInputCloud(cloud_src_);
        grid.filter(*cloud_src_ds);
        grid.setInputCloud(cloud_dst_);
        grid.filter(*cloud_dst_ds);
    }

    if(ui_->robustBox->isChecked()) {
        //      PCL_INFO("Generating correspondences for the downsampled models...\n");
        //      pcl::search::KdTree<PointT> ssrc, sdst;
        //      ssrc.setInputCloud(cloud_src_ds);
        //      sdst.setInputCloud(cloud_dst_ds);
        //      
        //      pcl::Correspondences corr_ds(src_pc_->size());
        //      for(size_t i = 0; i < src_pc_->size(); ++i) {
        //          std::vector<int> idx(1);
        //          std::vector<float> distsq(1);
        //          
        //          ssrc.nearestKSearch(src_pc_->points[corr_ds[i].index_query], 1, idx, distsq);
        //          corr_ds[i].index_query = idx[0];
        //          
        //          sdst.nearestKSearch(dst_pc_->points[corr_ds[i].index_match], 1, idx, distsq);
        //          corr_ds[i].index_match = idx[0];
        //      }

        pcl::Correspondences corr(src_pc_->size());
        for(size_t i = 0; i < src_pc_->size(); ++i)
            corr[i].index_query = corr[i].index_match = i;

        PCL_INFO("Computing pose using RANSAC with an inlier threshold of %f...\n", inlier_threshold_ransac);
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
        //      sac.setInputSource(cloud_src_ds);
        //      sac.setInputTarget(cloud_dst_ds);
        sac.setInputSource(src_pc_);
        sac.setInputTarget(dst_pc_);
        sac.setInlierThreshold(inlier_threshold_ransac);

        pcl::Correspondences inliers;
        //      sac.getRemainingCorrespondences(corr_ds, inliers);
        sac.getRemainingCorrespondences(corr, inliers);

        // Abort if RANSAC fails
        if(sac.getBestTransformation().isIdentity()) {
            PCL_ERROR("RANSAC failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("RANSAC failed!"));
            return;
        }

        transform_ = sac.getBestTransformation();
    } else {
        PCL_INFO("Computing pose using clicked correspondences...\n");
        pcl::registration::TransformationEstimationSVD<PointT,PointT> tfe;
        tfe.estimateRigidTransformation(*src_pc_, *dst_pc_, transform_);
    }

     float fine_res  = res_;
     float fitness_score = 0;
    if(ui_->refineBox->isChecked()) {
      
      
     
      pcl::GeneralizedIterativeClosestPoint6D gicp6d;
      Cloud tmp;
      
      PCL_INFO("Refining pose using Color ICP with an inlier threshold of %f...\n", inlier_threshold_icp);  
      gicp6d.setInputSource(cloud_src_ds);
      gicp6d.setInputTarget(cloud_dst_ds);
      gicp6d.setMaximumIterations(200);
      gicp6d.setMaxCorrespondenceDistance(inlier_threshold_icp);
      gicp6d.align(tmp, transform_);
      
       if(!gicp6d.hasConverged()) {
            PCL_ERROR("Color-ICP failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("Color-ICP failed!"));
            return;
        }
        
        PCL_INFO("Rerunning fine Color-ICP with an inlier threshold of %f...\n", 0.5 * inlier_threshold_icp);
        gicp6d.setMaximumIterations(100);
        gicp6d.setMaxCorrespondenceDistance(0.5 * inlier_threshold_icp);
        gicp6d.align(tmp, gicp6d.getFinalTransformation());

        if(!gicp6d.hasConverged()) {
            PCL_ERROR("Fine ICP failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("Fine Color-ICP failed!"));
            return;
        }
        
        PCL_INFO("Rerunning ultra-fine Color-ICP at full resolution with an inlier threshold of %f...\n", 0.1 * inlier_threshold_icp);
        gicp6d.setInputSource(cloud_src_);
        gicp6d.setInputTarget(cloud_dst_);
        gicp6d.setMaximumIterations(50);
        gicp6d.setMaxCorrespondenceDistance(0.1 * inlier_threshold_icp);
        gicp6d.align(tmp, gicp6d.getFinalTransformation());

        if(!gicp6d.hasConverged()) {
            PCL_ERROR("Ultra-fine ICP failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("Ultra-fine Color-ICP failed!"));
            return;
        }
  /*      
        PCL_INFO("Rerunning ultra-ultra-fine Color-ICP at full resolution with an inlier threshold of %f...\n", 0.5*res_);
        gicp6d.setInputSource(cloud_src_);
        gicp6d.setInputTarget(cloud_dst_);
        gicp6d.setMaximumIterations(25);
        gicp6d.setMaxCorrespondenceDistance(0.5*res_);
        gicp6d.align(tmp, gicp6d.getFinalTransformation());

        if(!gicp6d.hasConverged()) {
            PCL_ERROR("Ultra-Ultra-fine Color-ICP failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("Ultra-Ultra-fine ICP failed!"));
            return;
        }
*/
      /*
        pcl::IterativeClosestPoint<PointT,PointT> icp;
        Cloud tmp;

        PCL_INFO("Refining pose using ICP with an inlier threshold of %f...\n", inlier_threshold_icp);
        icp.setInputSource(cloud_src_ds);
        icp.setInputTarget(cloud_dst_ds);
        icp.setMaximumIterations(200);
	//icp.setEuclideanFitnessEpsilon(1e9);
	//icp.setTransformationEpsilon(1e9);
        icp.setMaxCorrespondenceDistance(inlier_threshold_icp);
        icp.align(tmp, transform_);

	
        if(!icp.hasConverged()) {
            PCL_ERROR("ICP failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("ICP failed!"));
            return;
        }

        PCL_INFO("Rerunning fine ICP with an inlier threshold of %f...\n", voxel_size);
        icp.setMaximumIterations(100);
        icp.setMaxCorrespondenceDistance(0.1 * inlier_threshold_icp);
        icp.align(tmp, icp.getFinalTransformation());

        if(!icp.hasConverged()) {
            PCL_ERROR("Fine ICP failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("Fine ICP failed!"));
            return;
        }
        
        PCL_INFO("Rerunning ultra-fine ICP at full resolution with an inlier threshold of %f...\n", res_);
        icp.setInputSource(cloud_src_);
        icp.setInputTarget(cloud_dst_);
        icp.setMaximumIterations(50);
        icp.setMaxCorrespondenceDistance(res_);
        icp.align(tmp, icp.getFinalTransformation());

        if(!icp.hasConverged()) {
            PCL_ERROR("Ultra-fine ICP failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("Ultra-fine ICP failed!"));
            return;
        }
        
        PCL_INFO("Rerunning ultra-ultra-fine ICP at full resolution with an inlier threshold of %f...\n", 0.5*res_);
        icp.setInputSource(cloud_src_);
        icp.setInputTarget(cloud_dst_);
        icp.setMaximumIterations(25);
        icp.setMaxCorrespondenceDistance(0.5*res_);
        icp.align(tmp, icp.getFinalTransformation());

        if(!icp.hasConverged()) {
            PCL_ERROR("Ultra-Ultra-fine ICP failed!\n");
            QMessageBox::warning(this,
                    QString("Error"),
                    QString("Ultra-Ultra-fine ICP failed!"));
            return;
        }
        fine_res  = 0.5*res_;
        for(int i = 4; i>=1;i--)
	{
	fine_res = 0.1 *res_* i;   
	PCL_INFO("Rerunning mega-fine ICP at full resolution with an inlier threshold of %f...\n", fine_res);
	icp.setInputSource(cloud_src_);
        icp.setInputTarget(cloud_dst_);
        icp.setMaximumIterations(25);
        icp.setMaxCorrespondenceDistance(fine_res);
        icp.align(tmp, icp.getFinalTransformation());
	  
	 if(icp.hasConverged()) {
            //PCL_ERROR("mega-fine ICP failed!\n");)
            continue;
	 }else{
	   PCL_ERROR("mega-fine ICP failed!\n");
	   break;
	 }
	  
	}
	
	 fine_res = 0.01* res_ * 10;
	 for(int i = 9; i>=1;i--)
	{
	  
	PCL_INFO("Rerunning mega-fine ICP at full resolution with an inlier threshold of %f...\n", fine_res);
	icp.setInputSource(cloud_src_);
        icp.setInputTarget(cloud_dst_);
        icp.setMaximumIterations(25);
        icp.setMaxCorrespondenceDistance(fine_res);
        icp.align(tmp, icp.getFinalTransformation());
	  
	 if(icp.hasConverged()) {
	   fine_res = 0.01* res_ * i; 
            //PCL_ERROR("mega-fine ICP failed!\n");)
            continue;
	 }else{
	   PCL_ERROR("mega-fine ICP failed!\n");
	   break;
	 }
	 
	  
	}
        fitness_score = icp.getFitnessScore();
        PCL_INFO("ICP has converged with Fitness score = %f\n", icp.getFitnessScore());
        transform_ = icp.getFinalTransformation();
	
	*/
    }
    
    trfm_computed_ = true;

    PCL_INFO("All done! The final refinement was done with an inlier threshold of %f, "
            "and you can expect the resulting pose to be accurate within this bound.\n", fine_res);
    
    std::cout << "Transform: " << std::endl << transform_ << std::endl;
    
    std::ofstream file("trfm.txt");
    if (file.is_open()){
      std::cout << "saving trfm" << std::endl;
    file << transform_ << '\n';
    file << fitness_score << '\n';
   // file << "m" << '\n' <<  colm(m) << '\n';
   file.close();
  }
    
    std::cout << "The transform can be used to place the source (leftmost) point cloud into the target, and thus places observations (points, poses) relative to the source camera in the target camera (rightmost). If you need the other way around, use the inverse:" << std::endl << transform_.inverse() << std::endl;

    cloud_aligned_.reset(new Cloud);
    pcl::transformPointCloud<PointT>(*cloud_src_, *cloud_aligned_, transform_);
    
       computeRMSError(cloud_aligned_, cloud_dst_, "nn");
       estimateOcclusion(cloud_aligned_, cloud_dst_);
       estimateClutter(cloud_aligned_, cloud_dst_);
       

   // pcl::visualization::PCLVisualizer vpose("Pose visualization. Red: scene. Green: aligned object");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vpose (new pcl::visualization::PCLVisualizer ("Pose visualization. Red: scene. Green: aligned object"));
    vpose->registerKeyboardCallback(&ManualRegistration::keyboardEventOccurred,*this, (void*)&vpose);
    
    vpose->addText("t -> Toggle color", 30,50);
    vpose->addText("n -> Toggle normals", 30,30);
    
    vpose->addPointCloud<PointT>(cloud_dst_,
            pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_dst_, 255, 0, 0),
            "scene");
    vpose->addPointCloud<PointT>(cloud_aligned_,
            pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_aligned_, 0, 255, 0),
            "aligned_object");
    vpose->spin();
}

void ManualRegistration::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void){
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "t" && event.keyDown ()){
     color_toggle_ = !color_toggle_;
    
     viewer->removeAllPointClouds();
     //   if(!viewer->updatePointCloud(cloud_dst_, "scene")){
	  std::vector<PCLPointField> point_fields;
	  if((pcl::getFieldIndex<PointT>(*cloud_dst_, "rgb", point_fields) > 0
	      || pcl::getFieldIndex<PointT>(*cloud_dst_, "rgba", point_fields) > 0) && color_toggle_){
            viewer->addPointCloud(cloud_dst_,
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_dst_),
                    "scene");
	  }else{
	    viewer->addPointCloud (cloud_dst_,
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_dst_, 255, 0, 0),
                    "scene");
	  }
       // }
        
  //       if(!viewer->updatePointCloud(cloud_aligned_, "aligned_object")){
	//  std::vector<PCLPointField> point_fields;
	  if((pcl::getFieldIndex<PointT>(*cloud_aligned_, "rgb", point_fields) > 0 
	    || pcl::getFieldIndex<PointT>(*cloud_aligned_, "rgba", point_fields) > 0) && color_toggle_){
            viewer->addPointCloud (cloud_aligned_,
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_aligned_),
                    "aligned_object");
	  }else{
	    viewer->addPointCloud (cloud_aligned_,
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_aligned_, 0, 255, 0),
                    "aligned_object");
	  }
       // }
 
    }
 //   viewer->update
 //   ui_->qvtk_widget_src->update();
 //   ui_->qvtk_widget_dst->update();
 // }
}

void ManualRegistration::clearPressed() {
    dst_point_selected_ = false;
    src_point_selected_ = false;
    src_pc_->points.clear();
    dst_pc_->points.clear();
    src_pc_->height = 1; src_pc_->width = 0;
    dst_pc_->height = 1; dst_pc_->width = 0;
//    vis_src_->removePointCloud("src_pc");
//    vis_dst_->removePointCloud("dst_pc");
    vis_src_->removeAllShapes();
    vis_dst_->removeAllShapes();
}

void ManualRegistration::applyTrfmPressed() {

  if(trfm_computed_){
     ManualRegistration::CloudPtr cloud_res (new ManualRegistration::Cloud);
    pcl::transformPointCloud(*cloud_src_, *cloud_res, transform_);
    pcl::io::savePCDFile("transformed_src_cloud.pcd", *cloud_res);
    
  }else
    PCL_ERROR("Transfomation not computed. Please press 'calculate pose'");
  
}

void ManualRegistration::timeoutSlot () {
    if(cloud_src_present_ && cloud_src_modified_)
    {
        if(!vis_src_->updatePointCloud(cloud_src_, "cloud_src_"))
        {
	  std::vector<PCLPointField> point_fields;
	  if(pcl::getFieldIndex<PointT>(*cloud_src_, "rgb", point_fields) > 0 
	    || pcl::getFieldIndex<PointT>(*cloud_src_, "rgba", point_fields) > 0 ){
	    std::cout << "rgb" << std::endl;
           vis_src_->addPointCloud (cloud_src_,
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_src_),
                    "cloud_src_");
	 
	  }/*else if(pcl::getFieldIndex<PointT>(*cloud_src_, "rgba", point_fields) > 0 ){
	     std::cout << "rgba" << std::endl;
	     vis_src_->addPointCloud (cloud_src_,
                    pcl::visualization::PointCloudColorHandlerRGBAField<PointT>(cloud_src_),
                    "cloud_src_");
	  }*/else{   
	     std::cout << "no color" << std::endl;
	       vis_src_->addPointCloud (cloud_src_,
                    pcl::visualization::PointCloudColorHandlerGenericField<PointT>(cloud_src_, "z"),
                    "cloud_src_");
	  }
            vis_src_->resetCameraViewpoint("cloud_src_");
        }
        cloud_src_modified_ = false;
    }
    if(cloud_dst_present_ && cloud_dst_modified_)
    {
        if(!vis_dst_->updatePointCloud(cloud_dst_, "cloud_dst_"))
        {
	    std::vector<PCLPointField> point_fields;
	  std::cout << "Index rgba" << pcl::getFieldIndex<PointT>(*cloud_dst_, "rgba", point_fields) << std::endl;
	    std::cout << "Index rgb" << pcl::getFieldIndex<PointT>(*cloud_dst_, "rgb", point_fields) << std::endl;
	 
	   if(pcl::getFieldIndex<PointT>(*cloud_dst_, "rgb", point_fields) > 0
	     || pcl::getFieldIndex<PointT>(*cloud_dst_, "rgba", point_fields) > 0){
	      vis_dst_->addPointCloud (cloud_dst_,
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_dst_),
                    "cloud_dst_");
	   }/*else if(pcl::getFieldIndex<PointT>(*cloud_dst_, "rgba", point_fields) > 0){
	     vis_dst_->addPointCloud (cloud_dst_,
                    pcl::visualization::PointCloudColorHandlerRGBAField<PointT>(cloud_dst_),
                    "cloud_dst_");
	   }*/else{
	      vis_dst_->addPointCloud (cloud_dst_,
                    pcl::visualization::PointCloudColorHandlerGenericField<PointT>(cloud_dst_, "z"),
                    "cloud_dst_");
	   }
            vis_dst_->resetCameraViewpoint("cloud_dst_");
        }
        cloud_dst_modified_ = false;
    }
    ui_->qvtk_widget_src->update();
    ui_->qvtk_widget_dst->update();
}

void ManualRegistration::computeRMSError(const ManualRegistration::CloudConstPtr &cloud_source, const ManualRegistration::CloudConstPtr &cloud_target,std::string correspondence_type){
  // Estimate
  pcl::console::TicToc tt;
  tt.tic ();

  float rmse = 0.0f;
  float median_error = 0.0f;
  float mean_error = 0.0f;
  float std_dev = 0.0f;
  std::vector<float> distances;
 
  if (correspondence_type == "nn"){
    pcl::console::print_highlight (stderr, "Computing using the nearest neighbor correspondence heuristic.\n");

    KdTreeFLANN<PointT>::Ptr tree (new KdTreeFLANN<PointT> ());
    tree->setInputCloud (cloud_target);

    for (size_t point_i = 0; point_i < cloud_source->points.size (); ++ point_i)
    {
      if (!pcl_isfinite (cloud_source->points[point_i].x) || !pcl_isfinite (cloud_source->points[point_i].y) || !pcl_isfinite (cloud_source->points[point_i].z))
        continue;

      std::vector<int> nn_indices (1);
      std::vector<float> nn_distances (1);
      if (!tree->nearestKSearch (cloud_source->points[point_i], 1, nn_indices, nn_distances))
        continue;
      size_t point_nn_i = nn_indices.front();

      float dist = squaredEuclideanDistance (cloud_source->points[point_i], cloud_target->points[point_nn_i]);
     // std::cout << "dist: " << dist << std::endl; 
      if(dist > 5* res_) dist = 0.0f;
      rmse += dist;
      distances.push_back(dist);
    }
    std::cout << "number of points: " << static_cast<float> (cloud_source->points.size ()) << std::endl;
    std::cout << "number of computed distances: " << static_cast<float> (distances.size()) << std::endl;
    std::cout << "rmse: " << rmse << std::endl;
    rmse = sqrtf (rmse / static_cast<float> (distances.size()));
    median_error = median(distances);  
    mean_error = mean(distances);
    std_dev = stdDev(distances);
    
  }
  else if (correspondence_type == "nnplane")
  {
    pcl::console::print_highlight (stderr, "Computing using the nearest neighbor plane projection correspondence heuristic.\n");

    PointCloud<Normal>::Ptr normals_target (new PointCloud<Normal> ());
    pcl::copyPointCloud(*cloud_target, *normals_target);
    //fromPCLPointCloud2 (*cloud_target, *normals_target);

    KdTreeFLANN<PointT>::Ptr tree (new KdTreeFLANN<PointT> ());
    tree->setInputCloud (cloud_target);

    for (size_t point_i = 0; point_i < cloud_source->points.size (); ++ point_i)
    {
      if (!pcl_isfinite (cloud_source->points[point_i].x) || !pcl_isfinite (cloud_source->points[point_i].y) || !pcl_isfinite (cloud_source->points[point_i].z))
        continue;

      std::vector<int> nn_indices (1);
      std::vector<float> nn_distances (1);
      if (!tree->nearestKSearch (cloud_source->points[point_i], 1, nn_indices, nn_distances))
        continue;
      size_t point_nn_i = nn_indices.front();

      Eigen::Vector3f normal_target = normals_target->points[point_nn_i].getNormalVector3fMap (),
          point_source = cloud_source->points[point_i].getVector3fMap (),
          point_target = cloud_target->points[point_nn_i].getVector3fMap ();

      float dist = normal_target.dot (point_source - point_target);
      rmse += dist * dist;
      distances.push_back(dist);
    }
    rmse = sqrtf (rmse / static_cast<float> (cloud_source->points.size ()));
    median_error = median(distances); 
    mean_error = mean(distances);
    std_dev = stdDev(distances);
  }
  else
  {
    pcl::console::print_error ("Unrecognized correspondence type. Check legal arguments by using the -h option\n");
    return;
  }

  pcl::console::print_info ("[done, "); pcl::console::print_value ("%g", tt.toc ()); pcl::console::print_info (" seconds]\n");
  pcl::console::print_highlight ("RMSE Error: %e (%f mm)\n", rmse, rmse); //*1000
  pcl::console::print_highlight ("Median Error: %e (%f mm)\n", median_error, median_error); //*1000
  pcl::console::print_highlight ("Mean Error: %e (%f mm)\n", mean_error, mean_error); //*1000
  pcl::console::print_highlight ("Std_dev: %e (%f mm)\n", std_dev, std_dev); //*1000
}

float ManualRegistration::mean(std::vector<float> &v){
   // mean
   float sum2 = 0.0f;
   for(int i = 0; i<v.size();i++)
      sum2 += v.at(i);
     
   float sum = std::accumulate( v.begin(), v.end(), 0.0f )/ static_cast<float> (v.size());
   sum2 = sum2/static_cast<float> (v.size());
 //  std::cout << "accumulate sum: " << sum << std::endl;
 //  std::cout << "sqrt accumulate sum: " << sqrtf(sum) << std::endl;
 //  std::cout << "sum: " << sum2 << std::endl;
   return sum2;
}

float ManualRegistration::variance(std::vector<float> &v){
        double mean_ = mean(v);
        double temp = 0;
        for(std::vector<float>::iterator it = v.begin(); it !=v.end(); it++){
            temp += (mean_-*it)*(mean_-*it);
	}
        return temp/v.size();
    }

float ManualRegistration::stdDev(std::vector<float> &v){
     return std::sqrt(variance(v));
}

float ManualRegistration::median(std::vector<float> &v){
   std::size_t size = v.end() - v.begin();
   std::size_t middleIdx = size/2;
   
   std::vector<float>::iterator target = v.begin() + middleIdx;
   std::nth_element(v.begin(), target, v.end());
  
    size_t n = v.size() / 2;

    if(size % 2 != 0){ //Odd number of elements
      return  *target;
  }else{            //Even number of elements
     double a = *target;
    std::vector<float>::iterator targetNeighbor= target-1;
    std::nth_element(v.begin(), targetNeighbor, v.end());
    return (a+*targetNeighbor)/2.0;
  }
}

int main (int argc, char** argv) {
    QApplication app(argc, argv);

    ManualRegistration::CloudPtr cloud_src (new ManualRegistration::Cloud);
    ManualRegistration::CloudPtr cloud_dst (new ManualRegistration::Cloud);

    if(argc < 3) {
        PCL_ERROR ("Usage:\n\t%s <source_cloud.pcd> <target_cloud.pcd>\n", argv[0]);
        return 0;
    }

     std::cout << "Loading source cloud..." << std::endl; 
    // TODO do this with PCL console
 
    int lastindex = std::string(argv[1]).find_last_of(".");
    if(std::string(argv[1]).substr(lastindex,std::string(argv[1]).length()).compare(std::string(".pcd")) == 0){
      if (pcl::io::loadPCDFile<ManualRegistration::PointT> (argv[1], *cloud_src) == -1) {
	  PCL_ERROR ("Couldn't read PCD file %s \n", argv[1]);
	  return (-1);
      }
     }else{
       std::cout << "Loading ply ..." << std::endl;
       pcl::PLYReader ply;
       if(ply.read<ManualRegistration::PointT> (argv[1], *cloud_src) == -1){ 
	  PCL_ERROR ("Couldn't read PLY file %s \n", argv[1]);
	  return (-1);
      }
     }
     
    std::cout << "Finish..." << std::endl;
    std::cout << "Loading target cloud..." << std::endl; 
    lastindex = std::string(argv[2]).find_last_of(".");
    if(std::string(argv[2]).substr(lastindex,std::string(argv[2]).length()).compare(std::string(".pcd")) == 0){
    if (pcl::io::loadPCDFile<ManualRegistration::PointT> (argv[2], *cloud_dst) == -1) {
        PCL_ERROR ("Couldn't read PCD file %s \n", argv[2]);
        return (-1);
    }
    }else if(std::string(argv[2]).substr(lastindex,std::string(argv[2]).length()).compare(std::string(".ply")) == 0){
       std::cout << "Loading ply ..." << std::endl;
      pcl::PLYReader ply;
      if(ply.read<ManualRegistration::PointT>(argv[2], *cloud_dst) == -1){ 
	PCL_ERROR ("Couldn't read PLY file %s \n", argv[1]);
	return (-1);
      }
    }else{
      std::cout << "Loading obj ..." << std::endl;
  /*    pcl:: ply;
      if(ply.read<ManualRegistration::PointT>(argv[2], *cloud_dst) == -1){ 
	PCL_ERROR ("Couldn't read PLY file %s \n", argv[1]);
	return (-1);
      }
      */
    }
    
     std::cout << "Finish..." << std::endl; 
     
    // Remove NaNs
    std::vector<int> dummy;
    pcl::removeNaNFromPointCloud(*cloud_src, *cloud_src, dummy);
    pcl::removeNaNFromPointCloud(*cloud_dst, *cloud_dst, dummy);
 /*   
    pcl::search::KdTree<ManualRegistration::PointT> s;
    const int k = 5;
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;

     std::cout << "Computing source resolution..." << std::endl; 
     
    s.setInputCloud(cloud_src);
    s.nearestKSearch(*cloud_src, std::vector<int>(), 5, idx, distsq);
    double res_src = 0.0f;
    for(size_t i = 0; i < cloud_src->size(); ++i) {
        double resi = 0.0f;
        for(int j = 1; j < k; ++j)
            resi += sqrtf(distsq[i][j]);
        resi /= double(k - 1);
        res_src += resi;
    }
    res_src /= double(cloud_src->size());
    
    std::cout << "Finish..." << std::endl; 
    std::cout << "Computing target resolution..." << std::endl; 
    
    s.setInputCloud(cloud_dst);
    s.nearestKSearch(*cloud_dst, std::vector<int>(), 5, idx, distsq);
    double res_dst = 0.0f;
//    #pragma omp parallel for
    for(size_t i = 0; i < cloud_dst->size(); ++i) {
        double resi = 0.0f;
        for(int j = 1; j < k; ++j)
            resi += sqrtf(distsq[i][j]);
        resi /= double(k - 1);
        res_dst += resi;
    }
    res_dst /= double(cloud_dst->size());
     std::cout << "Finish..." << std::endl; 
   */ 
    ManualRegistration man_reg;
    
    double resolution_ = man_reg.ComputeCloudResolution(cloud_src);

    man_reg.setSrcCloud(cloud_src);
    man_reg.setResolution(resolution_);
    //man_reg.setResolution(std::max<double>(res_src, res_dst));
    man_reg.setDstCloud(cloud_dst);

    man_reg.show();

    return (app.exec());
    
     return 0;
}
