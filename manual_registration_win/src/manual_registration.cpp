#include "manual_registration.h"

// Qt
#include <QApplication>
#include <QEvent>
#include <QMessageBox>
#include <QMutexLocker>
#include <QObject>
#include <QFileDialog>

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
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////
ManualRegistration::ManualRegistration () {
    // Initialize bogus
    res_ = 0.01;
	res_scene_ = 0.01;
    cloud_src_present_ = false;
    cloud_dst_present_ = false;
    src_point_selected_ = false;
    dst_point_selected_ = false;
    color_toggle_ = false;
    trfm_computed_ = true;
	model_name_ = "";
	scene_name_ = "";
    
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
	vis_src_->addText("+/-					-> Increase/decrease point size", 30, 130);
	vis_src_->addText("shift + left click	-> Select point", 30, 110);
	vis_src_->addText("r					-> Reset camera view", 30, 90);
	vis_src_->addText("f					-> Zoom to point", 30, 70);
	vis_src_->addText("t					-> Toggle color", 30, 50);
	vis_src_->addText("n					-> Toggle normals", 30, 30);
    ui_->qvtk_widget_src->update ();

    vis_src_->registerPointPickingCallback (&ManualRegistration::SourcePointPickCallback, *this);
	vis_src_->registerKeyboardCallback(&ManualRegistration::keyboardEventOccurred, *this, (void*)&vis_src_);

    // Set up the destination window
    vis_dst_.reset (new pcl::visualization::PCLVisualizer ("", false));
    ui_->qvtk_widget_dst->SetRenderWindow (vis_dst_->getRenderWindow ());
    vis_dst_->setupInteractor (ui_->qvtk_widget_dst->GetInteractor (), ui_->qvtk_widget_dst->GetRenderWindow ());
    vis_dst_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
	vis_dst_->addText("+/-					-> Increase/decrease point size", 30, 130);
	vis_dst_->addText("shift + left click	-> Select point", 30, 110);
	vis_dst_->addText("r					-> Reset camera view", 30, 90);
	vis_dst_->addText("f					-> Zoom to point", 30, 70);
	vis_dst_->addText("t					-> Toggle color", 30, 50);
	vis_dst_->addText("n					-> Toggle normals", 30, 30);
    ui_->qvtk_widget_dst->update ();

    vis_dst_->registerPointPickingCallback (&ManualRegistration::DstPointPickCallback, *this);
	vis_dst_->registerKeyboardCallback(&ManualRegistration::keyboardEventOccurred, *this, (void*)&vis_dst_);

    // Connect all buttons
    connect (ui_->calculateButton, SIGNAL(clicked()), this, SLOT(calculatePressed()));
    connect (ui_->clearButton, SIGNAL(clicked()), this, SLOT(clearPressed()));
    connect (ui_->btnApplyTrfm, SIGNAL(clicked()), this, SLOT(applyTrfmPressed()));


//    cloud_src_modified_ = true; // first iteration is always a new pointcloud
//    cloud_dst_modified_ = true;

    src_pc_.reset(new Cloud);
    dst_pc_.reset(new Cloud);
}

void ManualRegistration::on_btnModel_clicked(){

	QString filters("PLY files(*.ply);;PCD files (*.pcd);; All files(*.*)");

	//get a filename to open
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open Model point cloud"), "C:", filters);

	ManualRegistration::CloudPtr cloud_src(new ManualRegistration::Cloud);
	ManualRegistration::MeshPtr mesh_src_(new ManualRegistration::Mesh);

	if (!fileName.isEmpty()){
		int lastindex = fileName.toStdString().find_last_of(".");
		QString modelname_ = fileName.split(".", QString::SkipEmptyParts).at(0);
		int last = modelname_.toStdString().find_last_of("/");
		model_name_ = QString::fromStdString(modelname_.toStdString().substr(last+1, fileName.toStdString().length()));

		if (fileName.toStdString().substr(lastindex, fileName.toStdString().length()).compare(std::string(".pcd")) == 0){
			if (pcl::io::loadPCDFile<ManualRegistration::PointT>(fileName.toStdString(), *cloud_src) == -1) {
				QMessageBox::warning(this,
					QString("Warning"),
					QString("Could not read .pcd model from" + fileName));

			}
			this->setSrcCloud(cloud_src);
		
		}else{
			std::cout << "Loading ply from -> " << fileName.toStdString() << std::endl;
		//	pcl::PLYReader ply;
			//if (ply.read<ManualRegistration::PointT>(fileName.toStdString(), *cloud_src) == -1){
			if (pcl::io::loadPLYFile(fileName.toStdString(), *mesh_src_) == -1){
				QMessageBox::warning(this,
					QString("Warning"),
					QString("Could not read .ply model from " + fileName));
			}
			pcl::fromPCLPointCloud2(mesh_src_->cloud, *cloud_src);
			this->setSrcCloud(cloud_src);
			
			
		}

		ui_->textModelPath->setText(fileName);
		double resolution_ = ComputeCloudResolution(cloud_src);

		if (ui_->cbDownsample->isChecked()){
			pcl::VoxelGrid<PointT> sor;
			sor.setInputCloud(cloud_src);
			sor.setLeafSize(resolution_ * 2, resolution_ * 2, resolution_ * 2);
			sor.filter(*cloud_src);

		}

		
		this->setResolution(resolution_);
		this->cloud_src_present_ = true;
		cloud_src_modified_ = true; // first iteration is always a new pointcloud
	}

}

void ManualRegistration::on_btnScene_clicked(){

	QString filters("PLY files(*.ply);;PCD files (*.pcd);;All files(*.*)");

	//get a filename to open
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open Scene point cloud"), "C:", filters);

	ManualRegistration::CloudPtr cloud_dst(new ManualRegistration::Cloud);
	ManualRegistration::MeshPtr mesh_dst_(new ManualRegistration::Mesh);

	
	if (!fileName.isEmpty()){
		int lastindex = fileName.toStdString().find_last_of(".");
		QString scenename_ = fileName.split(".", QString::SkipEmptyParts).at(0);
		int last = scenename_.toStdString().find_last_of("/");
		base_path_ = QString::fromStdString(scenename_.toStdString().substr(0, last));
		scene_name_ = QString::fromStdString(scenename_.toStdString().substr(last+1, fileName.toStdString().length()));

		if (fileName.toStdString().substr(lastindex, fileName.toStdString().length()).compare(std::string(".pcd")) == 0){
			if (pcl::io::loadPCDFile<ManualRegistration::PointT>(fileName.toStdString(), *cloud_dst) == -1) {
				QMessageBox::warning(this,
					QString("Warning"),
					QString("Could not read .pcd model from" + fileName));

			}
			this->setDstCloud(cloud_dst);	
		}
		else{
			std::cout << "Loading ply from -> " << fileName.toStdString() << std::endl;
			pcl::PLYReader ply;
			if (pcl::io::loadPLYFile(fileName.toStdString(), *mesh_dst_) == -1){
				QMessageBox::warning(this,
					QString("Warning"),
					QString("Could not read .ply model from " + fileName));
			}
			pcl::fromPCLPointCloud2(mesh_dst_->cloud, *cloud_dst);
			this->setDstCloud(cloud_dst);
			
		}

		this->res_scene_ = ComputeCloudResolution(cloud_dst);
		if (ui_->cbDownsample->isChecked()){
			pcl::VoxelGrid<PointT> sor;
			sor.setInputCloud(cloud_dst);
			sor.setLeafSize(res_scene_ * 5, res_scene_ * 5, res_scene_ * 5);
			sor.filter(*cloud_dst);

		}
		
		ui_->textScenePath->setText(fileName);
		this->cloud_dst_present_ = true;
		cloud_dst_modified_ = true; // first iteration is always a new pointcloud

		
	}
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
     
      /// Save the point if the distance is below 2 * resolution 
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

	if (cloud_dst_ == nullptr){
		QMessageBox::warning(this,
			QString("Info"),
			QString("Please load a model and a scene before starting the registration!"));
		return;
	}

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
		QMessageBox::warning(this,
			QString("Info"),
			QString("Please select a point in the source window first"));
    }
}

void ManualRegistration::DstPointPickCallback (const pcl::visualization::PointPickingEvent& event, void*) {
    // Check to see if we got a valid point. Early exit.
    int idx = event.getPointIndex ();
	if (idx == -1){
	
		return;
	}

	if (cloud_src_ == nullptr){
		QMessageBox::warning(this,
		QString("Info"),
		QString("Please load a model and a scene before starting the registration!"));
		return;
	}

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
		QMessageBox::warning(this,
			QString("Info"),
			QString("Please select a point in the destination window first"));

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
     
 /*     pcl::GeneralizedIterativeClosestPoint6D gicp6d;
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

    }
    
    trfm_computed_ = true;

    //std::cout << "Transform: " << std::endl << transform_ << std::endl;

	cloud_aligned_.reset(new Cloud);
	pcl::transformPointCloud<PointT>(*cloud_src_, *cloud_aligned_, transform_);

	float rms_ = computeRMSError(cloud_aligned_, cloud_dst_, "nn");
	double occlusion_ = estimateOcclusion(cloud_aligned_, cloud_dst_);
	double clutter_ = estimateClutter(cloud_aligned_, cloud_dst_);


	PCL_INFO("All done! The final refinement was done with an inlier threshold of %f, "
		"and you can expect the resulting pose to be accurate within this bound.\n", fine_res);
	QMessageBox::warning(this,
		QString("Info"),
		QString("All done! The final refinement was done with an inlier threshold of " + QString::number(fine_res) + " and you can expect the resulting pose to be accurate within this bound.\n RMS_Error = "
		        + QString::number(rms_) + "\n Occlusion = " + QString::number(occlusion_) + "\n Clutter = " + QString::number(clutter_)));

	//Compute poses in the 11 sensor views
	for (int i = 0; i < 10; i++)
	{
		//Load transform 
		QString path(base_path_ + "/trfm/stl_to_world_trfm_" + QString::number(i) + ".txt");
		Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
		mat = readMatrix(path);
		Eigen::Matrix4f trfm = mat.inverse();

		//Allocate clouds
		CloudPtr local_cloud_aligned_(new Cloud);
		CloudPtr scene_xx(new Cloud);
		MeshPtr mesh_(new Mesh);

		//Load scene
		QString cloud_path(base_path_ + "/stl_" + QString::number(i) + ".ply");
		
		//Load clouds		
		pcl::PLYReader ply;
		if (pcl::io::loadPLYFile(cloud_path.toStdString(), *mesh_) == -1){
			QMessageBox::warning(this,
				QString("Warning"),
				QString("Could not read .ply model from " + cloud_path));
		}
		
		pcl::fromPCLPointCloud2(mesh_->cloud, *scene_xx);

	//	pcl::IterativeClosestPoint<PointT, PointT> icp;

//		icp.setInputSource(local_cloud_aligned_);
	//	icp.setInputTarget(scene_xx);
	//	icp.setMaximumIterations(10);
		//icp.setEuclideanFitnessEpsilon(1e9);
		//icp.setTransformationEpsilon(1e9);
	//	icp.setMaxCorrespondenceDistance(inlier_threshold_icp);
	//	icp.align(*local_cloud_aligned_);


	//	if (!icp.hasConverged()) {
	//		PCL_ERROR("ICP failed!\n");
	//		QMessageBox::warning(this,
	//			QString("Error"),
	//			QString("ICP failed!"));
	//	}
	//	else{

			Eigen::Matrix4f final_trfm = trfm * transform_.inverse(); //*icp.getFinalTransformation();
			pcl::transformPointCloud<PointT>(*cloud_aligned_, *local_cloud_aligned_, final_trfm);
			pcl::io::savePCDFile("local_cloud_aligned_.pcd", *local_cloud_aligned_);

		//	CloudPtr local_full_scene(new Cloud);
			//pcl::transformPointCloud<PointT>(*cloud_dst_, *local_full_scene, icp.getFinalTransformation() * trfm);

			pcl::io::savePCDFile("scene.pcd", *scene_xx);
		//	pcl::io::savePCDFile("local_full_scene.pcd", *local_full_scene);

			float rms_ = computeRMSError(local_cloud_aligned_, scene_xx, "nn");
			double occlusion_ = estimateOcclusion(local_cloud_aligned_, scene_xx);
			double clutter_ = estimateClutter(local_cloud_aligned_, scene_xx);

			//Save ground truth pose
			QString name(base_path_ + "/stl_" + QString::number(i) + "_" + model_name_ + "_pose.txt");
			std::ofstream file(name.toStdString());
			if (file.is_open()){
				file << final_trfm << '\n';
			//	file << icp.getFinalTransformation() << '\n';
			//	file << icp.getFitnessScore() << '\n';
				file << rms_ << '\n';
				file << occlusion_ << '\n';
				file << clutter_ << '\n';
				file.close();
			}
		//}
		
	}
	
	
	QString name(base_path_ + "/" + scene_name_ + "_" + model_name_ + "_pose.txt");

	QMessageBox::warning(this,
		QString("Info"),
		QString("Saving result to: " + name));

    std::ofstream file(name.toStdString());
    if (file.is_open()){
		file << transform_ << '\n';
		file << fitness_score << '\n';
		file << rms_ << '\n';
		file << occlusion_ << '\n';
		file << clutter_ << '\n';
		//file << "m" << '\n' <<  colm(m) << '\n';
		file.close();
  }
    
    std::cout << "The transform can be used to place the source (leftmost) point cloud into the target, and thus places observations (points, poses) relative to the source camera in the target camera (rightmost). If you need the other way around, use the inverse:" << std::endl << transform_.inverse() << std::endl;
       

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

void ManualRegistration::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void){
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym() == "q" && event.keyDown()){
	  viewer->close();
	  return;
  }

  if (event.getKeySym () == "t" && event.keyDown ()){
     color_toggle_ = !color_toggle_;
    

     //   if(!viewer->updatePointCloud(cloud_dst_, "scene")){
	  std::vector<PCLPointField> point_fields;
	  // toggle colors for cloud_dst_
	  if (!cloud_dst_->empty() && cloud_dst_present_){
		  viewer->removeAllPointClouds();
		  cloud_dst_present_ = false;
		  if ((pcl::getFieldIndex<PointT>(*cloud_dst_, "rgb", point_fields) > 0
			  || pcl::getFieldIndex<PointT>(*cloud_dst_, "rgba", point_fields) > 0) && color_toggle_){
			  viewer->addPointCloud(cloud_dst_,
				  pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_dst_),
				  "cloud_dst_");
		  }
		  else{
			  viewer->addPointCloud(cloud_dst_,
				  pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_dst_, 255, 0, 0),
				  "cloud_dst_");
		  }
		  cloud_dst_present_ = true;
	  }
	  // toggle colors for cloud_src_
	  if (!cloud_src_->empty() && cloud_src_present_){
//		  viewer->removeAllPointClouds();
		/*  if ((pcl::getFieldIndex<PointT>(*cloud_src_, "rgb", point_fields) > 0
			  || pcl::getFieldIndex<PointT>(*cloud_src_, "rgba", point_fields) > 0) && color_toggle_){
			  viewer->addPointCloud(cloud_src_,
				  pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_src_),
				  "model");
		  }
		  else{
			  viewer->addPointCloud(cloud_src_,
				  pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_src_, 0, 255, 0),
				  "model");
		  }
		  */
	  }
	  // toggle colors for cloud_src_
/*	  if((pcl::getFieldIndex<PointT>(*cloud_aligned_, "rgb", point_fields) > 0 
	    || pcl::getFieldIndex<PointT>(*cloud_aligned_, "rgba", point_fields) > 0) && color_toggle_ && !cloud_aligned_->empty()){
            viewer->addPointCloud (cloud_aligned_,
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_aligned_),
                    "aligned_object");
	  }else{
	    viewer->addPointCloud (cloud_aligned_,
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_aligned_, 0, 255, 0),
                    "aligned_object");
	  }
	  */
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

	//Re-draw text in dst window
	vis_dst_->addText("+/-					-> Increase/decrease point size", 30, 130);
	vis_dst_->addText("shift + left click	-> Select point", 30, 110);
	vis_dst_->addText("r					-> Reset camera view", 30, 90);
	vis_dst_->addText("f					-> Zoom to point", 30, 70);
	vis_dst_->addText("t					-> Toggle color", 30, 50);
	vis_dst_->addText("n					-> Toggle normals", 30, 30);

	//Re-draw text in src window
	vis_src_->addText("+/-					-> Increase/decrease point size", 30, 130);
	vis_src_->addText("shift + left click	-> Select point", 30, 110);
	vis_src_->addText("r					-> Reset camera view", 30, 90);
	vis_src_->addText("f					-> Zoom to point", 30, 70);
	vis_src_->addText("t					-> Toggle color", 30, 50);
	vis_src_->addText("n					-> Toggle normals", 30, 30);
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
				vis_src_->addPointCloud (cloud_src_,
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_src_),
                    "cloud_src_");
			}else{   
				vis_src_->addPointCloud (cloud_src_,
                    pcl::visualization::PointCloudColorHandlerGenericField<PointT>(cloud_src_, "z"),
                    "cloud_src_");
			}
			

        vis_src_->resetCameraViewpoint("cloud_src_");
		vis_src_->resetCamera();
		vis_src_->removeAllCoordinateSystems();
		vis_src_->addCoordinateSystem(100);
		
        }
        cloud_src_modified_ = false;
    }

    if(cloud_dst_present_ && cloud_dst_modified_)
    {
        if(!vis_dst_->updatePointCloud(cloud_dst_, "cloud_dst_"))
        {
			std::vector<PCLPointField> point_fields;
		//	std::cout << "Index rgba" << pcl::getFieldIndex<PointT>(*cloud_dst_, "rgba", point_fields) << std::endl;
		//	std::cout << "Index rgb" << pcl::getFieldIndex<PointT>(*cloud_dst_, "rgb", point_fields) << std::endl;
	 
			if(pcl::getFieldIndex<PointT>(*cloud_dst_, "rgb", point_fields) > 0
				|| pcl::getFieldIndex<PointT>(*cloud_dst_, "rgba", point_fields) > 0){
					vis_dst_->addPointCloud (cloud_dst_,
						pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud_dst_),
						"cloud_dst_");
			}else{
					vis_dst_->addPointCloud (cloud_dst_,
						pcl::visualization::PointCloudColorHandlerGenericField<PointT>(cloud_dst_, "z"),
						"cloud_dst_");
			}
		
         vis_dst_->resetCameraViewpoint("cloud_dst_");
		 vis_dst_->resetCamera();
		 vis_dst_->removeAllCoordinateSystems();
		 vis_dst_->addCoordinateSystem(100);

        }
        cloud_dst_modified_ = false;
    }
    ui_->qvtk_widget_src->update();
    ui_->qvtk_widget_dst->update();
}

float ManualRegistration::computeRMSError(const ManualRegistration::CloudConstPtr &cloud_source, const ManualRegistration::CloudConstPtr &cloud_target,std::string correspondence_type){
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
      if(dist > 2* res_) dist = 0.0f;
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
    return -1;
  }

  pcl::console::print_info ("[done, "); pcl::console::print_value ("%g", tt.toc ()); pcl::console::print_info (" seconds]\n");
  pcl::console::print_highlight ("RMSE Error: %e (%f mm)\n", rmse, rmse); //*1000
  pcl::console::print_highlight ("Median Error: %e (%f mm)\n", median_error, median_error); //*1000
  pcl::console::print_highlight ("Mean Error: %e (%f mm)\n", mean_error, mean_error); //*1000
  pcl::console::print_highlight ("Std_dev: %e (%f mm)\n", std_dev, std_dev); //*1000

  return rmse;
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

Eigen::Matrix4f ManualRegistration::readMatrix(QString filename){
	using namespace std;
	int cols = 0, rows = 0;
	double buff[1000];

	// Read numbers from file into buffer.
	ifstream infile;
	infile.open(filename.toStdString().c_str());
	if (!infile.is_open()){
		QMessageBox::warning(this,QString("Info"),QString("Could not open file: " + filename));	
	}

	while (!infile.eof())
	{
		string line;
		getline(infile, line);
		//QMessageBox::warning(this, QString("Info"), QString(QString::fromStdString(line)));
		int temp_cols = 0;
		stringstream stream(line);
		while (!stream.eof())
			stream >> buff[cols*rows + temp_cols++];

		if (temp_cols == 0)
			continue;

		if (cols == 0)
			cols = temp_cols;

		rows++;
	}	

	infile.close();

	//rows--;

	// Populate matrix with numbers.
	Eigen::Matrix4f result(rows, cols);
	for (int i = 0; i < rows; i++)
	for (int j = 0; j < cols; j++)
		result(i, j) = buff[cols*i + j];

	return result;
	
}

int main (int argc, char** argv) {
    QApplication app(argc, argv);

    ManualRegistration man_reg;
    man_reg.show();

    return (app.exec());
    
     return 0;
}
