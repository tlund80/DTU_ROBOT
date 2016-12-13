#include <QCoreApplication>
#include <QDir>
#include <QMap>
#include <QTextStream>

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
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::PolygonMesh Mesh;
typedef pcl::PolygonMesh::Ptr MeshPtr;

//Global variables
double res_;

float mean(std::vector<float> &v){
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

float variance(std::vector<float> &v){
        double mean_ = mean(v);
        double temp = 0;
        for(std::vector<float>::iterator it = v.begin(); it !=v.end(); it++){
            temp += (mean_-*it)*(mean_-*it);
    }
        return temp/v.size();
    }

float stdDev(std::vector<float> &v){
     return std::sqrt(variance(v));
}

float median(std::vector<float> &v){
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

float computeRMSError(const CloudConstPtr &cloud_source, const CloudConstPtr &cloud_target,std::string correspondence_type){
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

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
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

    pcl::PointCloud<pcl::Normal>::Ptr normals_target (new pcl::PointCloud<pcl::Normal> ());
    pcl::copyPointCloud(*cloud_target, *normals_target);
    //fromPCLPointCloud2 (*cloud_target, *normals_target);

    pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
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

double estimateOcclusion(const CloudConstPtr &model, const CloudConstPtr &scene){

  ///Occlusion = 1 - visible object area / total object area

    std::vector<float> distances;
    pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
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

double estimateClutter(const CloudConstPtr &model, const CloudConstPtr &scene){

  ///Clutter = 1 - Visible object area/total scene area
    std::vector<float> distances;
    pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
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

Eigen::Matrix4f readMatrix(QString filename){
    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly)) {
 //       pcl::console::print_error("%s\n", file.errorString().toStdString());
    }

    QTextStream in(&file);
    std::vector<double> data;
    while(!in.atEnd()) {
        QString line = in.readLine();
        QStringList fields = line.split(" ");
        fields.removeAll("");
        foreach(QString str, fields){
            data.push_back(str.toDouble());
        }
    }

    file.close();
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result << data[0], data[1], data[2],data[3],
              data[4],data[5],data[6],data[7],
              data[8],data[9],data[10],data[11],
              data[12],data[13],data[14],data[15];

    return result;

}

double ComputeCloudResolution(const CloudConstPtr &cloud){

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

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
 //   pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

    if(argc < 3){
        pcl::console::print_error("Usage: %s %s", argv[0], "<path to scenes> <path to objects>\n" );
        return 0;
    }

    double inlier_threshold_icp = 0.1;
    QString base_path_ = QString::fromStdString(argv[1]);
    QString base_object_path_ = QString::fromStdString(argv[2]);

    //Create ground_truth dir
    QDir gt_dir(base_path_ + "ground_truth/");
    if(!gt_dir.exists())
        gt_dir.mkdir(base_path_ + "ground_truth/");

    //Allocate clouds
    CloudPtr object_aligned_(new Cloud);
    CloudPtr local_object_aligned_(new Cloud);
    CloudPtr scene_full(new Cloud);
    CloudPtr scene_xx(new Cloud);
    CloudPtr object_cloud(new Cloud);
    CloudPtr scene_xx_sampled(new Cloud);
    CloudPtr local_object_aligned_sampled_(new Cloud);
    MeshPtr mesh_(new Mesh);
    MeshPtr object_mesh_(new Mesh);

    //Load full scene
    QString cloud_path(base_path_ + "stl/stl_full.ply");
    QString scene_name = base_path_;
    scene_name.remove(0, scene_name.lastIndexOf("/")-9).remove(9,9);

    std::cout << "Scene name: " << scene_name.toStdString() << std::endl;

    pcl::console::print_info("Loading full scene: %s\n", cloud_path.toStdString().c_str());
    //Load clouds
    if (pcl::io::loadPLYFile(cloud_path.toStdString(), *mesh_) == -1){
            pcl::console::print_error("Could not read .ply model from %s",cloud_path.toStdString().c_str());
    }

    pcl::fromPCLPointCloud2(mesh_->cloud, *scene_full);

    //Loading all annotated poses
    QDir dir(base_path_ + "stl/");                            //Opens the path
    dir.setFilter(QDir::Files | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    QStringList filter;  filter << "stl_full_*.txt";


    //Loop through all the found poses
    foreach(QString object, dir.entryList(filter)) {
        QString p(base_path_ + "stl/" + object);
        object.remove(0,9); //Remove stl_full

        Eigen::Matrix4f  transform_ = Eigen::Matrix4f::Identity();
        transform_ = readMatrix(p);
        std::cout << transform_ << std::endl;

        //Load model
        QString object_name = object.remove(object.length()-9,9);
        QString object_path(base_object_path_ + object_name + "/" + object_name + ".ply");

        pcl::console::print_info("Loading object: %s\n", object_name.toStdString().c_str());
        //Load clouds
        if (pcl::io::loadPLYFile(object_path.toStdString(), *object_mesh_) == -1){
            pcl::console::print_error("Could not read .ply model from %s",object_path.toStdString().c_str());
        }

        //Apply object transformation
        pcl::fromPCLPointCloud2(object_mesh_->cloud, *object_cloud);
        pcl::transformPointCloud<PointT>(*object_cloud, *object_aligned_, transform_);
       // pcl::io::savePCDFile("object_aligned.pcd", *object_aligned_);
       // pcl::io::savePCDFile("scene_full.pcd", *scene_full);

        //Compute poses in the 11 sensor views
        for (int i = 0; i < 11; i++){

            PCL_INFO("Aligning view nr: %d\n",i );

            //Load transform from view to world frame
            QString path(base_path_ + "stl/trfm/stl_to_world_trfm_" + QString::number(i) + ".txt");
            Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
            mat = readMatrix(path);

            //Load ICP transform
            QString path_icp(base_path_ + "stl/trfm/stl_icp_trfm_" + QString("%1").arg(i,2, 10, QChar('0')) + ".txt");
            std::cout << "Loading: " << path_icp.toStdString() << std::endl;
            Eigen::Matrix4f mat_icp = Eigen::Matrix4f::Identity();
            mat_icp = readMatrix(path_icp);
            Eigen::Matrix4f trfm = mat.inverse() * mat_icp.inverse();

            //Load scene
            QString cloud_path(base_path_ + "stl/stl_" + QString::number(i) + ".ply");

            pcl::console::print_info("Loading scene: %s\n", cloud_path.toStdString().c_str());
            //Load clouds
            if (pcl::io::loadPLYFile(cloud_path.toStdString(), *mesh_) == -1){
                pcl::console::print_error("Could not read .ply model from %s",cloud_path.toStdString().c_str());
            }

            pcl::fromPCLPointCloud2(mesh_->cloud, *scene_xx);

         //    std::stringstream ss; ss.str("");
         //  ss << "scene_"; ss << i; ss << ".pcd";
        //    pcl::io::savePCDFile(ss.str(), *scene_xx);;

            pcl::transformPointCloud<PointT>(*object_aligned_, *local_object_aligned_, trfm);

        //    ss.str(""); ss << object_name.toStdString(); ss << "_in_local_frame"; ss << i; ss << ".pcd";
        //    pcl::io::savePCDFile(ss.str(), *local_object_aligned_);
            //Compute resolution
            res_ = ComputeCloudResolution(scene_xx);

           if (true){
                pcl::VoxelGrid<PointT> sor;
                sor.setInputCloud(scene_xx);
                sor.setLeafSize(res_ * 4, res_ * 4, res_ * 4);
                sor.filter(*scene_xx_sampled);

                sor.setInputCloud(local_object_aligned_);
                sor.setLeafSize(res_ * 4, res_ * 4, res_ * 4);
                sor.filter(*local_object_aligned_sampled_);

            }


            PCL_INFO("Running ICP  with an inlier threshold of %f and %d iterations...\n", res_ * 2, 20);
            Cloud tmp;
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            icp.setInputSource(local_object_aligned_sampled_);
            icp.setInputTarget(scene_xx_sampled);
            icp.setMaximumIterations(10);
            icp.setMaxCorrespondenceDistance(res_ * 2);
            icp.align(*local_object_aligned_);
            PCL_INFO("Fitness_score: %d\n",icp.getFitnessScore());

            if (!icp.hasConverged()) {
                PCL_ERROR("ICP failed!\n");
            }

  /*          PCL_INFO("Rerunning ICP  with an inlier threshold of %f and %d iterations...\n", res_ * 5, 20);
            icp.setMaximumIterations(20);
            icp.setMaxCorrespondenceDistance(res_ * 5);
            icp.align(tmp, icp.getFinalTransformation());
            PCL_INFO("Fitness_score: %d\n",icp.getFitnessScore());

            if(!icp.hasConverged()) {
                PCL_ERROR("Fine ICP failed!\n");
            }


            PCL_INFO("Rerunning ICP  in full resolution with an inlier threshold of %f and %d iterations...\n", res_ * 2, 20);
          //  icp.setInputSource(local_object_aligned_);
          //  icp.setInputTarget(scene_xx);
            icp.setMaximumIterations(20);
            icp.setMaxCorrespondenceDistance(res_ * 2);
            icp.align(tmp, icp.getFinalTransformation());
            PCL_INFO("Fitness_score: %d\n",icp.getFitnessScore());

            if(!icp.hasConverged()) {
                PCL_ERROR("Fine ICP failed!\n");
            }
*/
            else{

              //  std::cout << "icp.getFinalTransformation():\n" << icp.getFinalTransformation() << std::endl;

                Eigen::Matrix4f final_trfm = trfm * transform_;
                Eigen::Matrix4f final_trfm_icp = icp.getFinalTransformation() * final_trfm;
                CloudPtr temp(new Cloud);
                pcl::transformPointCloud<PointT>(*object_cloud, *temp, final_trfm_icp);
                //ss.str(""); ss << object_name.toStdString(); ss << "_in_local_frame"; ss << i; ss << ".pcd";
                //pcl::io::savePCDFile(ss.str(), *temp);
              //  pcl::io::savePCDFile("local_cloud_aligned_.pcd", *local_object_aligned_);


                float rms_ = computeRMSError(local_object_aligned_, scene_xx, "nn");
                double occlusion_ = estimateOcclusion(local_object_aligned_, scene_xx);
                double clutter_ = estimateClutter(local_object_aligned_, scene_xx);

                //Save ground truth pose
                QString name(base_path_ + "ground_truth/" + scene_name + "_" + QString("%1").arg(i,2, 10, QChar('0')) + "-" + object_name + ".xf");

                std::cout << "Saving ground_truth .xf: " << name.toStdString() << std::endl;
                std::ofstream file(name.toStdString().c_str());
                if (file.is_open()){
                    file << final_trfm << '\n';
                //	file << icp.getFinalTransformation() << '\n';
                //	file << icp.getFitnessScore() << '\n';
                    file << rms_ << '\n';
                    file << occlusion_ << '\n';
                    file << clutter_ << '\n';
                    file.close();
                }


            }

        }
       // return 0;
    }


    return 0;
}
