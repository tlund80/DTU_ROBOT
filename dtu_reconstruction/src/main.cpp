#include <QtCore/QCoreApplication>
#include <QDebug>
#include <QStringList>
#include <QRegExp>
#include <QDir>
#include <QSettings>

//#include "stereo_match.h"
#include "viewdata.h"
#include "scenedata.h"

#include <pcl/common/bivariate_polynomial.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
//#include <pcl/registration/gicp6d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include "normal_correction_manifold.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <vector>

#include "SMCalibrationWorker.h"
#include "SMCalibrationParameters.h"
#include "AlgorithmGrayCode.h"
#include "AlgorithmLineShift.h"

using namespace Eigen;

void reconstructSTL(SMFrameSequence &frames, std::string path);

void help()
{
    std::cout << "Help" << std::endl;
    std::cout << "Calibrate stereo -calibrate <path-to-images>" << std::endl;
    std::cout << "Path to images -folder <path-to-images>" << std::endl;
    std::cout << "Reconstruct all in `folder`-all" << std::endl;
    std::cout << "Reconstruct only stereo in `folder`-stereo" << std::endl;
    std::cout << "Reconstruct only stl in `folder`-stl" << std::endl;
    std::cout << "Align kinect -align <path-to-clouds> -transform <path-to-handEye>" << std::endl;
}

void loadCalibrationImages(QString calib_path,  std::vector<SMCalibrationSet> &calibration_set){
    //Load calibration images and calibrate
     std::cout << "Loading calibration images from " << calib_path.toStdString();

     std::vector<cv::Mat> left_calib_;
     std::vector<cv::Mat> right_calib_;
      std::vector<cv::Mat> kinect_calib_;
     QDir dir(calib_path);                            //Opens the path
     dir.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
     QFileInfoList subfolders = dir.entryInfoList();

     foreach(const QFileInfo &subfolder, subfolders) { //Loops through the found views (left, right, kinect).
        QDir file_directory(subfolder.absoluteFilePath());
        QStringList files = file_directory.entryList(QDir::Files);

        if(subfolder.baseName().compare("left",Qt::CaseInsensitive) == 0){
            int nr = 0;
            foreach(const QString &file, files) {
                QString file_path = subfolder.absoluteFilePath() + "/" + file;
                if(file.contains(".png")){
                   std::cout << "Loading  " << file_path.toStdString() << std::endl;
                   cv::Mat left_tmp_ = cv::imread(file_path.toStdString(),cv::IMREAD_GRAYSCALE);

                   if( left_tmp_.empty() )
                     std::cerr <<  "Could not open or find the left calibration image nr: " << nr << std::endl;

                   left_calib_.push_back(left_tmp_);
                   nr++;
                }
           }
        }else if(subfolder.baseName().compare("right",Qt::CaseInsensitive) == 0){
            int nr = 0;
            foreach(const QString &file, files) {
                QString file_path = subfolder.absoluteFilePath() + "/" + file;
                if(file.contains(".png")){
                      std::cout << "Loading: " << file_path.toStdString() << std::endl;
                       cv::Mat right_tmp_ = cv::imread(file_path.toStdString(),cv::IMREAD_GRAYSCALE);

                       if( right_tmp_.empty() )
                           std::cerr <<  "Could not open or find the right calibration image nr: " << nr << std::endl ;

                       right_calib_.push_back(right_tmp_);
                       nr++;
                }
            }
           }else if(subfolder.baseName().compare("kinect",Qt::CaseInsensitive) == 0){
            int nr = 0;
            foreach(const QString &file, files) {
                QString file_path = subfolder.absoluteFilePath() + "/" + file;
                if(file.contains(".png")){
                      std::cout << "Loading: " << file_path.toStdString() << std::endl;
                       cv::Mat kinect_tmp_ = cv::imread(file_path.toStdString(),cv::IMREAD_GRAYSCALE);
                        kinect_tmp_ =  kinect_tmp_+ cv::Scalar(-75,-75,-75);
                    //  cv::imwrite("/home/thso/DTUData/calibration/run_002/kinect_eq.png",kinect_tmp_);

                       if( kinect_tmp_.empty() )
                           std::cerr <<  "Could not open or find the kinect calibration image nr: " << nr << std::endl ;

                       kinect_calib_.push_back(kinect_tmp_);
                       nr++;
                }
            }
           }

     }

     for(unsigned int i = 0; i<left_calib_.size(); i++ ){
         SMCalibrationSet calibSet;
         calibSet.frame0 = left_calib_[i];
         calibSet.frame1 = right_calib_[i];
         calibSet.id = i;
         calibSet.rotationAngle = 0;
         calibSet.checked = true;
         calibration_set.push_back(calibSet);
     }

     left_calib_.clear();
     right_calib_.clear();

     std::cout << " ---> Done!" << std::endl;
}

void loadView(QString folder_name, ViewData &single_view){

    std::cout << "Loading view in " << folder_name.toStdString() << ".....";

    std::string left_stereo_path;
    std::string right_stereo_path;
    std::string left_rgb_stereo_path;
    std::string right_rgb_stereo_path;
    std::vector<std::string> stl_left;
    std::vector<std::string> stl_right;

    QDir directory(folder_name);
    directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    QFileInfoList subfolders = directory.entryInfoList();

    /// Loop throught each folder (left, right, kinect).
    foreach(const QFileInfo &subfolder, subfolders) {
       QDir file_directory(subfolder.absoluteFilePath());
       QStringList files = file_directory.entryList(QDir::Files);

       unsigned int num = 0;
       /// Loops through the found subfolders (left, right, kinect).
       foreach(const QString &file, files) {
        if(subfolder.baseName().compare("kinect",Qt::CaseInsensitive) == 0){
           QString file_path = subfolder.absoluteFilePath() + "/" + file;
            if(file.contains(".pcd")){
                 if(!single_view.loadKinectcloud(file_path.toStdString()))
                    std::cout << "Could not load kinect data from: " << file_path.toStdString() << std::endl;
                }
            if(file.contains(".jpg"))
                    single_view.loadKinectRGB(file_path.toStdString());
            if(file.contains(".png"))
                    single_view.loadKinectDepth(file_path.toStdString());
        }else if(subfolder.baseName().compare("left",Qt::CaseInsensitive) == 0){
            QString file_path = subfolder.absoluteFilePath() + "/" + file;
         //   std::cout << "Loading left from: " << file_path.toStdString() << std::endl;
        /*    if(num == 0)
                left_rgb_stereo_path = file_path.toStdString();


            if(num == 20){
                    left_stereo_path = file_path.toStdString();
               //     std::cout << "Loading left from: " << file_path.toStdString() << std::endl;

                 }else
*/
            if(num < 30) //gray code = 22 //line shift = 30
                    stl_left.push_back(file_path.toStdString());

                     num++;

        }else if(subfolder.baseName().compare("right",Qt::CaseInsensitive) == 0){
                QString file_path = subfolder.absoluteFilePath() + "/" + file;
            //    std::cout << "Loading right from: " << file_path.toStdString() << std::endl;

            /*        if(num == 0)
                        right_rgb_stereo_path = file_path.toStdString();

                    if(num == 20){
                        right_stereo_path = file_path.toStdString();
                    }else
*/
                if(num < 30)
                        stl_right.push_back(file_path.toStdString());

                        num++;
             }
        }
     }

    // single_view.loadStereoTexture(left_stereo_path,right_stereo_path);
    // single_view.loadStereoRGB(left_rgb_stereo_path,right_rgb_stereo_path);
     single_view.loadStructuredLight(stl_left, stl_right);
     stl_left.clear();
     stl_right.clear();

     std::cout << "... Done!" << std::endl;

}

void loadscene(QString folder_name, SceneData &scene){

    std::cout << "Loading scene from " << folder_name.toStdString() << "....." << std::endl;

    QDir view_directory(folder_name);
    view_directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    QFileInfoList views = view_directory.entryInfoList();

    unsigned int view_id= 0;
    scene.setName(view_directory.dirName().toStdString());

    /// Loops through the found views Pos_xx
    foreach(const QFileInfo &view, views) {
     boost::shared_ptr<ViewData> view_data(new ViewData(view_id));

     std::string left_stereo_path;
     std::string right_stereo_path;
     std::vector<std::string> stl_left;
     std::vector<std::string> stl_right;

     QDir directory(view.absoluteFilePath());
     directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
     QFileInfoList subfolders = directory.entryInfoList();

     /// Loop throught each folder (left, right, kinect).
     foreach(const QFileInfo &subfolder, subfolders) {
       QDir file_directory(subfolder.absoluteFilePath());
       QStringList files = file_directory.entryList(QDir::Files);

       /// Loops through the found subfolders (left, right, kinect).
       foreach(const QString &file, files) {

        if(subfolder.baseName().compare("kinect",Qt::CaseInsensitive) == 0){
           QString file_path = subfolder.absoluteFilePath() + "/" + file;
            if(file.contains(".pcd")){
                 if(!view_data->loadKinectcloud(file_path.toStdString()))
                    std::cout << "Could not load kinect data from: " << file_path.toStdString() << std::endl;
                }
            if(file.contains(".jpg"))
                    view_data->loadKinectRGB(file_path.toStdString());
            if(file.contains(".png"))
                    view_data->loadKinectDepth(file_path.toStdString());
        }else if(subfolder.baseName().compare("left",Qt::CaseInsensitive) == 0){
            QString file_path = subfolder.absoluteFilePath() + "/" + file;
            std::cout << "Loading left from: " << file_path.toStdString() << std::endl;
                 static unsigned int num = 0;
                 if(num == 20)
                    left_stereo_path = file_path.toStdString();
                 else
                    stl_left.push_back(file_path.toStdString());

                     num++;
            }else if(subfolder.baseName().compare("right",Qt::CaseInsensitive) == 0){
                QString file_path = subfolder.absoluteFilePath() + "/" + file;
                std::cout << "Loading right from: " << file_path.toStdString() << std::endl;
                   static unsigned int num = 0;
                    if(num == 20)
                        right_stereo_path = file_path.toStdString();
                    else
                        stl_right.push_back(file_path.toStdString());

                        num++;
             }
            }
         }


     view_data->loadStereoTexture(left_stereo_path,right_stereo_path);
     view_data->loadStructuredLight(stl_left, stl_right);
     view_id++;
     stl_left.clear();
     stl_right.clear();
     scene.data.push_back(view_data);

     } //End views

   std::cout << " ---> Done!" << std::endl;

}

void loadAllData(QString folder_name, std::vector<SceneData> &scenes){

    std::cout << "Loading all data in " << folder_name.toStdString() << "....." << std::endl;
    //Reconstruct all in folder_name
    QDir dir(folder_name);                            //Opens the path
    dir.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    QFileInfoList folders = dir.entryInfoList(); //Gets the file list

    foreach(const QFileInfo &folder, folders) {//Loops through the found scenes.
        std::cout << folder.absoluteFilePath().toStdString() << std::endl;

        QDir view_directory(folder.absoluteFilePath());
        view_directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
        QFileInfoList views = view_directory.entryInfoList();

        unsigned int view_id= 0;
        SceneData scene;
        scene.setName(view_directory.dirName().toStdString());


        foreach(const QFileInfo &view, views) { //Loops through the found views Pos_xx
             std::cout << "view " << view.absoluteFilePath().toStdString() << std::endl;

          bool left_loaded = false;
          bool right_loaded = false;
          bool kinect_loaded = false;

          std::string left_stereo_path;
          std::string right_stereo_path;
          std::vector<std::string> stl_left;
          std::vector<std::string> stl_right;

          QDir directory(view.absoluteFilePath());
          directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
          QFileInfoList subfolders = directory.entryInfoList();

          foreach(const QFileInfo &subfolder, subfolders) { // Loop throught each folder    (left, right, kinect).

            std::cout << "sub" << subfolder.absoluteFilePath().toStdString() << std::endl;

            QDir file_directory(subfolder.absoluteFilePath());
            QStringList files = file_directory.entryList(QDir::Files);
            boost::shared_ptr<ViewData> views(new ViewData(view_id));

            if(subfolder.baseName().compare("kinect",Qt::CaseInsensitive) == 0){
                     foreach(const QString &file, files) { //Loops through the found subfolders (left, right, kinect).
                         QString file_path = subfolder.absoluteFilePath() + "/" + file;
                         if(file.contains(".pcd"))
                             views->loadKinectcloud(file_path.toStdString());
                         if(file.contains(".jpg"))
                             views->loadKinectRGB(file_path.toStdString());
                         if(file.contains(".png"))
                             views->loadKinectDepth(file_path.toStdString());
                     }
                     kinect_loaded = true;
            }else if(subfolder.baseName().compare("left",Qt::CaseInsensitive) == 0){
                      unsigned int num = 0;
                      foreach(const QString &file, files) { //Loops through the found subfolders (left, right, kinect).
                          QString file_path = subfolder.absoluteFilePath() + "/" + file;

                          if(num == 3)
                              left_stereo_path = file_path.toStdString();
                          else
                              stl_left.push_back(file_path.toStdString());

                          num++;

                      }
                      left_loaded = true;
            }else if(subfolder.baseName().compare("right",Qt::CaseInsensitive) == 0){
                    unsigned int num = 0;
                    foreach(const QString &file, files) { //Loops through the found subfolders (left, right, kinect).
                        QString file_path = subfolder.absoluteFilePath() + "/" + file;

                        if(num == 3)
                            right_stereo_path = file_path.toStdString();
                        else
                            stl_right.push_back(file_path.toStdString());

                        num++;
                    }
                    right_loaded = true;
                }

            if(kinect_loaded && left_loaded && right_loaded){
            //        std::cout << "left_stereo_path: " << left_stereo_path << std::endl;
            //        std::cout << "right_stereo_path: " << right_stereo_path << std::endl;

                    views->loadStereoTexture(left_stereo_path,right_stereo_path);

                    views->loadStructuredLight(stl_left, stl_right);
                    view_id++;
                    stl_left.clear();
                    stl_right.clear();
                    scene.data.push_back(views);
                }

         }
     }

        scenes.push_back(scene);
    }

   std::cout << " ---> Done!" << std::endl;

}

void reconstructStereo(ViewData &view, unsigned int view_id){

/*    QSettings settings;
    SMCalibrationParameters calibration;// = settings.value("calibration/parameters").value<SMCalibrationParameters>();
    calibration.importFromXML("Calibration.xml");
   // calibration.print();

    StereoMatch sm;
    sm.setAlgorithm(STEREO_BM);
    sm.setRectified(false);
    sm.setMinDisparity(-46);//-46
    sm.setMaxDisparity(160); //96
    sm.setBlockSize(15);
    sm.setVerbose(false);
    sm.setCalibrationParams(calibration);
  //  sm.show_rectification(true);

    std::cout << "Reconstruct view nr: " << view.getViewNr() << ".....";

    std::pair<cv::Mat, cv::Mat> tex = view.getStereoTexturePair();
      std::pair<cv::Mat, cv::Mat> rgb = view.getStereoRGBPair();
    cv::imwrite("/home/thso/left.png", rgb.first);
    cv::imwrite("/home/thso/right.png", rgb.second);

    CloudPtr cloud = sm.compute(rgb.first, rgb.second,rgb.first);
    std::stringstream ss; ss << "/home/thso/stereo_"; ss << view_id; ss << ".ply";
    pcl::io::savePLYFileBinary(ss.str(), *cloud);
    std::cout << ".....Done!!" << std::endl;
       //  pcl::io::savePCDFile(ss.str(), *cloud);
       */
}

pcl::PointCloud<PointNT>::Ptr reconstructSTL(ViewData &view, unsigned int view_id){

    std::cout << "Reconstruct view nr: " << view.getViewNr() << ".....";

    SMCalibrationParameters calibration;// = settings.value("calibration/parameters").value<SMCalibrationParameters>();
    calibration.importFromXML("Calibration.xml");
   // calibration.print();

    std::vector<cv::Point3f> Q;
    std::vector<cv::Vec3b> color;
    SMFrameSequence frames = view.getSTLFramesequence();

 //   AlgorithmGrayCode* gc = new AlgorithmLineShift(1024, 800); //1024
 //   gc->get3DPoints(calibration,frames.frames0, frames.frames1,Q,color);
 //   delete gc;

    AlgorithmLineShift* ls = new AlgorithmLineShift(1024, 800); //1024
    ls->get3DPoints(calibration,frames.frames0, frames.frames1,Q,color);
    delete ls;


    std::cout << "Q: " << Q.size() << std::endl;

    // Convert point cloud to PCL format
    pcl::PointCloud<PointNT>::Ptr pointCloudPCL(new pcl::PointCloud<PointNT>);

    pointCloudPCL->width = Q.size();
    pointCloudPCL->height = 1;
    pointCloudPCL->is_dense = true;
    pointCloudPCL->points.resize(Q.size());

   for(unsigned int i=0; i<Q.size(); i++){
      PointNT point;
      point.x = Q[i].x; point.y = Q[i].y; point.z = Q[i].z;
      point.r = color[i][2]; point.g = color[i][1]; point.b = color[i][0];
      pointCloudPCL->points[i] = point;
   }

   //Compute cloud resolution
    pcl::search::KdTree<PointNT> s;
    const int k = 5;
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;

    s.setInputCloud(pointCloudPCL);
    s.nearestKSearch(*pointCloudPCL, std::vector<int>(), 5, idx, distsq);
    double resolution = 0.0f;
    for(size_t i = 0; i < pointCloudPCL->size(); ++i) {
        double resi = 0.0f;
        for(int j = 1; j < k; ++j)
            resi += sqrtf(distsq[i][j]);
        resi /= double(k - 1);
        resolution += resi;
    }
    resolution /= double(pointCloudPCL->size());

    //Estimate surface normals
    pcl::NormalEstimationOMP<PointNT, PointNT> ne;
    ne.setInputCloud (pointCloudPCL);
    pcl::search::KdTree<PointNT>::Ptr search_tree (new pcl::search::KdTree<PointNT>);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (5.0f * resolution);
    //ne.useSensorOriginAsViewPoint();
    ne.setViewPoint(0,0,1);
    ne.compute (*pointCloudPCL);


    return pointCloudPCL;
}

std::vector<Affine3f> loadRobotPoses(QString path){

    std::cout << "Loading robot poses: " << path.toStdString() << std::endl;

    std::string line;
    std::vector<Affine3f> T;
    std::ifstream infile(path.toStdString());

    while(!infile.eof()) // To get you all the lines.
    {
        std::getline(infile,line); // Saves the line in STRING.

        //Erase brances
        line.erase(std::remove(line.begin(), line.end(), '['), line.end());
        line.erase(std::remove(line.begin(), line.end(), ']'), line.end());

        //Split string
        std::vector<std::string> v;
        int i = 0;
        int j = line.find(',');
        while (j != line.npos) {
              v.push_back(line.substr(i, j-i));
              i = ++j;
              j = line.find(',', j);

              if (j == line.npos)
                 v.push_back(line.substr(i, line.length()));
           }

        //Erase space
        for(int k = 0; k<v.size();k++)
            v[k].erase(std::remove(v[k].begin(),v[k].end(),' '),v[k].end());

        if(v.size() != 7)
            break;

       //Store Eigen affine data structure
        Affine3f transform = Affine3f::Identity();
        transform.translation() << std::stof(v[0]), std::stof(v[1]), std::stof(v[2]);
        transform.rotate (Quaternionf(std::stof(v[3]), std::stof(v[4]), std::stof(v[5]), std::stof(v[6])));
        T.push_back(transform);

        //std::cout<<transform.matrix() << std::endl; // Prints our STRING.
        //std::cout<<line << std::endl; // Prints our STRING.
    }
    infile.close();

    return T;
}

double computeResolution(CloudNormal::Ptr cloud){


        //Compute cloud resolution for each sub scan
         pcl::search::KdTree<PointNT> s;
         const int k = 5;
         std::vector<std::vector<int> > idx;
         std::vector<std::vector<float> > distsq;

         s.setInputCloud(cloud);
         s.nearestKSearch(*cloud, std::vector<int>(), 5, idx, distsq);
         double resolution = 0.0f;
         for(size_t i = 0; i < cloud->size(); ++i) {
             double resi = 0.0f;
             for(int j = 1; j < k; ++j)
                 resi += sqrtf(distsq[i][j]);
             resi /= double(k - 1);
             resolution += resi;
         }
         resolution /= double(cloud->size());


    std::cout << "Cloud resolution: " << resolution << std::endl;
    return resolution;

}

cv::Mat loadHandEye(QString path)
{
    int _validImgs;
    cv::Size _imageSize;
    double _squareSize;
    cv::Size _boardSize;
    unsigned int _count;
    cv::Mat _handEyeTransform;
    cv::Mat _calibrationTargetTransform;

    cv::FileStorage fs(path.toStdString(), cv::FileStorage::READ);

    if (!fs.isOpened()){
        fs.open(path.toStdString(), cv::FileStorage::READ);
    }
    if (fs.isOpened()) {
    //	fs << "%--> HAND EYE CALIBRATION RESULTS <-- \n";
    //	fs << " --> Image acqusition parameters <-- \n";

         _validImgs = (int) fs["image_count"];
         fs["image_size"] >> _imageSize;
 //       _squareSize = fs["square_size"];
 //       _boardSize = fs["pattern_size"];
 //       _count = fs["robot_pose_count"];

        fs["hand_eye_transform"] >> _handEyeTransform;
        fs["calibration_Target_Transform"] >>  _calibrationTargetTransform;

        fs.release();
    }//else
      //  Q_EMIT consoleSignal("Error: can not save the Hand/Eye parameters\n", dti::VerboseLevel::ERROR);


    std::cout << "Hand eye transform: \n" << _handEyeTransform << std::endl;

    return _handEyeTransform;

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


void alignKinectScene(QString base_path){

  //  pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

    //Load robot poses
    std::stringstream ss; ss << base_path.toStdString(); ss << "robot_positions.txt";
    std::vector<Affine3f> poses = loadRobotPoses(QString::fromStdString(ss.str()));

    //Load the corresponding stl_world scene
    CloudNormal::Ptr cloud_full_stl (new CloudNormal);
    pcl::PLYReader ply_reader;
    ss.str(""); ss << base_path.toStdString(); ss << "full_stl.ply";
    std::cout << "Loading: " << ss.str() << std::endl;
    if(ply_reader.read(ss.str(),*cloud_full_stl) < 0){
        pcl::console::print_error("Could not load full point cloud. Make sure to align the stl clouds before kinect!!");// << ss.str());
        return;
    }


    std::vector<CloudPtr > clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > cloudsInitial;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > cloudsICP;

     Matrix3f rot_color2Ir =  Matrix3f::Identity();
     rot_color2Ir<< 9.9999556395900735e-01, 2.9693376835843075e-03, 2.3472543028155737e-04,
                    -2.9681013242091147e-03, 9.9998257512681676e-01, -5.1029224244224982e-03,
                    -2.4987364007190722e-04, 5.1022030987889991e-03, 9.9998695245823221e-01;
     Affine3f color2ir = Affine3f::Identity();
    color2ir.translation() << -5.2011890662796961e-02, 3.3168926537064601e-04, -1.1583846472618690e-03;
    color2ir.rotate(rot_color2Ir);

    std::cout << "color to ir transform: \n" << color2ir.matrix() << std::endl;

    //Load Hand eye transform
   cv::Mat handeye = loadHandEye("Kinect_he_alignment.yml");
   Affine3f he = Affine3f::Identity();
   he.translation() << static_cast<float>(handeye.at<double>(0,3)), static_cast<float>(handeye.at<double>(1,3)), static_cast<float>(handeye.at<double>(2,3));

   Matrix3f rot =  Matrix3f::Identity();
   rot<< static_cast<float>(handeye.at<double>(0,0)), static_cast<float>(handeye.at<double>(0,1)), static_cast<float>(handeye.at<double>(0,2)),
         static_cast<float>(handeye.at<double>(1,0)), static_cast<float>(handeye.at<double>(1,1)), static_cast<float>(handeye.at<double>(1,2)),
         static_cast<float>(handeye.at<double>(2,0)), static_cast<float>(handeye.at<double>(2,1)), static_cast<float>(handeye.at<double>(2,2));

   he.rotate(rot);

   std::cout << "Hand eye transform: \n" << he.matrix() << std::endl;

   //Load kinect clouds
   QDir view_directory(base_path);
   view_directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
   QFileInfoList views = view_directory.entryInfoList();

   assert(poses.size() == views.size());

   unsigned int view_id= 0;
   /// Loops through the found views Pos_xx
   foreach(const QFileInfo &view, views) {
    //Load .ply files
    Cloud::Ptr cloud(new Cloud);
    pcl::PCDReader reader;
    std::stringstream ss; ss << view.absoluteFilePath().toStdString(); ss << "/kinect/kinect_cloud.pcd";
    std::cout << "Loading: " << ss.str() << std::endl;
    if(reader.read(ss.str(),*cloud));
       clouds.push_back(cloud);
 //  }
   //Rough alignment utilizing the robot poses
 //  for(size_t i = 0; i < clouds.size (); ++i){
       CloudPtr source = cloud;//clouds[i];

       //Remove Nans
       std::vector<int> indices;
       pcl::removeNaNFromPointCloud(*source, *source, indices);

        CloudPtr cloud_scaled(new Cloud);
        for(size_t j = 0; j < source->size();++j){
           pcl::PointXYZRGB p = source->points[j];
           p.x = p.x * 1000.0f;
           p.y = p.y * 1000.0f;
           p.z = p.z * 1000.0f;
           cloud_scaled->points.push_back(p);
        }

        //Compute normals
        //Estimate surface normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        ne.setInputCloud (cloud_scaled);
        ne.setSearchSurface(cloud_scaled);
        pcl::search::KdTree<PointT>::Ptr search_tree (new pcl::search::KdTree<PointT>);
        ne.setSearchMethod (search_tree);
        ne.setKSearch(10);
        ne.setViewPoint(0,0,1);
        ne.compute(*cloud_normals);
        CloudNormal::Ptr cloud_n (new CloudNormal);
        pcl::concatenateFields<PointT, pcl::Normal, PointNT>(*cloud_scaled, *cloud_normals, *cloud_n);

       Affine3f T_rob = poses[view_id];
       Matrix4f T_align = T_rob.matrix() * he.matrix() * color2ir.matrix();
       pcl::transformPointCloudWithNormals(*cloud_n,*cloud_n,T_align);

       // Remove robot and walls from the scene
       pcl::PassThrough<PointNT> pass;
       pass.setInputCloud(cloud_n);
       pass.setFilterFieldName("y");
       pass.setFilterLimits(-400, 400);
       pass.filter(*cloud_n);
       pass.setInputCloud(cloud_n);
       pass.setFilterFieldName("x");
       pass.setFilterLimits(350, 700);
       pass.filter(*cloud_n);

       pass.setInputCloud(cloud_n);
       pass.setFilterFieldName("z");
       pass.setFilterLimits(-200, 250);
       pass.filter(*cloud_n);

       cloudsInitial.push_back(cloud_n);

       pcl::IterativeClosestPoint<PointTNormal,PointTNormal> icp;
       double res = computeResolution(cloud_n);
       const double voxel_size =  5 * res;
       const float inlier_threshold_icp = 10 * res;


       CloudNormal::Ptr source_ds(new CloudNormal);
       CloudNormal::Ptr target_ds(new CloudNormal);

       //Downsample clouds
       pcl::VoxelGrid<PointTNormal> vg;
       vg.setInputCloud(cloud_n);
       vg.setLeafSize(voxel_size, voxel_size, voxel_size);
       vg.filter(*source_ds);

       vg.setInputCloud(cloud_full_stl);
       vg.filter(*target_ds);


       PCL_INFO("Refining pose using ICP with an inlier threshold of %f...\n", inlier_threshold_icp);
       CloudNormal tmp;
       icp.setInputSource(source_ds);
       icp.setInputTarget(target_ds);
       icp.setMaximumIterations(50);
       icp.setMaxCorrespondenceDistance(inlier_threshold_icp);
       icp.align(tmp);

       if(!icp.hasConverged()) {
            PCL_ERROR("ICP failed!\n");
            return;
        }

        PCL_INFO("Rerunning fine ICP at with an inlier threshold of %f...\n", 0.1 * inlier_threshold_icp);
        icp.setMaximumIterations(25);
        icp.setMaxCorrespondenceDistance(0.1 * inlier_threshold_icp);
        icp.align(tmp, icp.getFinalTransformation());

       if(!icp.hasConverged()) {
           PCL_ERROR("Fine ICP failed!\n");
           return;
       }

        PCL_INFO("Rerunning fine ICP in full resolution with an inlier threshold of %f...\n", 0.05 * inlier_threshold_icp);
        icp.setInputSource(cloud_n);
        icp.setInputTarget(cloud_full_stl);
        icp.setMaximumIterations(3);
        icp.setMaxCorrespondenceDistance(0.05 * inlier_threshold_icp);
        icp.align(tmp, icp.getFinalTransformation());

       if(!icp.hasConverged()) {
           PCL_ERROR("Fine ICP failed!\n");
           return;
       }

       PCL_INFO("Rerunning fine ICP in full resolution with an inlier threshold of %f...\n", 0.01 * inlier_threshold_icp);
       icp.setMaximumIterations(3);
       icp.setMaxCorrespondenceDistance(0.01 * inlier_threshold_icp);
       icp.align(tmp, icp.getFinalTransformation());

      if(!icp.hasConverged()) {
          PCL_ERROR("Fine ICP failed!\n");
          return;
      }

      PCL_INFO("Rerunning fine ICP in full resolution with an inlier threshold of %f...\n", 0.005 * inlier_threshold_icp);
      icp.setMaximumIterations(5);
      icp.setMaxCorrespondenceDistance(0.005 * inlier_threshold_icp);
      icp.align(tmp, icp.getFinalTransformation());

      if(!icp.hasConverged()) {
         PCL_ERROR("Fine ICP failed!\n");
         return;
     }

        pcl::transformPointCloudWithNormals(*cloud_n,*cloud_n,icp.getFinalTransformation());
        //Save transformed model
        pcl::PLYWriter writer;
        ss.str(""); ss << view.absoluteFilePath().toStdString(); ss << "/kinect_world_"; ss << view_id; ss << ".ply";
        std::cout << "Saving: " << ss.str() << std::endl;
        writer.write(ss.str(),*cloud_n,true,false);


        Eigen::Matrix4f final = T_align * icp.getFinalTransformation();
        //TODO: float rms_ = computeRMSError(local_object_aligned_, scene_xx, "nn");
       //Save alignment transform -> world frame = stl_frame. final is the transformation required to align each kinect with the full stl scene used for gt annotation
       ss.str(""); ss << view.absoluteFilePath().toStdString(); ss << "/kinect_to_world_trfm_"; ss << view_id; ss << ".txt";
       std::cout << "Saving alignment trasform: " << ss.str() << std::endl;
       std::ofstream file(ss.str());
       if (file.is_open())
           file << final;

       file.close();

       //Save kinect clouds in stl sensor frame to have the same ground truth
       //First load the stl transformation
        ss.str(""); ss << view.absoluteFilePath().toStdString(); ss << "/stl_to_world_trfm_"; ss << view_id; ss << ".txt";
        Matrix4f T_stl = readMatrix(QString::fromStdString(ss.str()));
       //Transform cloud
       Matrix4f T_inv = T_stl.inverse();
       pcl::transformPointCloudWithNormals(*cloud_n,*cloud_n,T_inv);
       ss.str(""); ss << view.absoluteFilePath().toStdString(); ss << "/kinect_"; ss << view_id; ss << ".ply";
       std::cout << "Transforming kinect cloud back to sensor frame and saving: " << ss.str() << std::endl;
       writer.write(ss.str(),*cloud_n,true,false);

       std::cout << std::endl;

       view_id++;

   }

 /*  pcl::IterativeClosestPoint<PointTNormal,PointTNormal> icp;
   CloudNormal::Ptr target = cloudsInitial[0];
   for (size_t j = 1; j < cloudsInitial.size (); ++j){
           CloudNormal::Ptr source = cloudsInitial[j];

           CloudNormal::Ptr source_ds(new CloudNormal);
           CloudNormal::Ptr target_ds(new CloudNormal);
           CloudNormal tmp;

           assert(!source->empty());
           assert(!target->empty());

           const double voxel_size =  5 * computeResolution(source);

           //Downsample clouds
           pcl::VoxelGrid<PointTNormal> vg;
           vg.setInputCloud(source);
           vg.setLeafSize(voxel_size, voxel_size, voxel_size);
           vg.filter(*source_ds);

           vg.setInputCloud(target);
           vg.filter(*target_ds);

           const float inlier_threshold_icp = 5 * computeResolution(source_ds);

           PCL_INFO("Refining pose using ICP with an inlier threshold of %f...\n", inlier_threshold_icp);
           icp.setInputSource(source_ds);
           icp.setInputTarget(target_ds);
           icp.setMaximumIterations(200);
           //icp.setEuclideanFitnessEpsilon(1e9);
           //icp.setTransformationEpsilon(1e9);
           icp.setMaxCorrespondenceDistance(inlier_threshold_icp);
           icp.align(tmp);


          // PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
            if(!icp.hasConverged()) {
               PCL_ERROR("ICP failed!\n");
           return;
           }

            PCL_INFO("Rerunning ICP with an inlier threshold of %f...\n", 0.5 * inlier_threshold_icp);
            icp.setMaximumIterations(50);
            icp.setMaxCorrespondenceDistance(0.5 * inlier_threshold_icp);
            icp.align(tmp, icp.getFinalTransformation());

           //  PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
            if(!icp.hasConverged()) {
                PCL_ERROR("Fine ICP failed!\n");
                return;
            }

            PCL_INFO("Rerunning fine ICP at with an inlier threshold of %f...\n", 0.1 * inlier_threshold_icp);
        //    icp.setInputSource(source);
        //    icp.setInputTarget(cloud_dst_);
            icp.setMaximumIterations(25);
            icp.setMaxCorrespondenceDistance(0.1 * inlier_threshold_icp);
            icp.align(tmp, icp.getFinalTransformation());

        //    PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
           if(!icp.hasConverged()) {
               PCL_ERROR("Fine ICP failed!\n");
               return;
           }

            PCL_INFO("Rerunning fine ICP in full resolution with an inlier threshold of %f...\n", 0.05 * inlier_threshold_icp);
            icp.setInputSource(source);
            icp.setInputTarget(target);
            icp.setMaximumIterations(25);
            icp.setMaxCorrespondenceDistance(0.05 * inlier_threshold_icp);
            icp.align(tmp, icp.getFinalTransformation());

          //  PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
           if(!icp.hasConverged()) {
               PCL_ERROR("Fine ICP failed!\n");
               return;
           }


           PCL_INFO("Rerunning fine ICP in full resolution with an inlier threshold of %f...\n", 0.01 * inlier_threshold_icp);
           icp.setInputSource(source);
           icp.setInputTarget(target);
           icp.setMaximumIterations(10);
           icp.setMaxCorrespondenceDistance(0.01 * inlier_threshold_icp);
           icp.align(tmp, icp.getFinalTransformation());

         //  PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
          if(!icp.hasConverged()) {
              PCL_ERROR("Fine ICP failed!\n");
              return;
          }

          PCL_INFO("Rerunning fine ICP in full resolution with an inlier threshold of %f...\n", 0.005 * inlier_threshold_icp);
          icp.setInputSource(source);
          icp.setInputTarget(target);
          icp.setMaximumIterations(10);
          icp.setMaxCorrespondenceDistance(0.005 * inlier_threshold_icp);
          icp.align(tmp, icp.getFinalTransformation());

        //  PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
         if(!icp.hasConverged()) {
             PCL_ERROR("Fine ICP failed!\n");
             return;
         }


            Matrix4f transform_ = Matrix4f::Identity();
           transform_ = icp.getFinalTransformation();


           CloudNormal::Ptr cloud_aligned_ (new CloudNormal);
           pcl::transformPointCloud<PointTNormal>(*source, *cloud_aligned_, transform_);
           cloudsICP.push_back(cloud_aligned_);

           pcl::PLYWriter writer;
           ss.str(""); ss << "/home/thso"; ss << "/kinect_align_src"; ss << j; ss << ".ply";
           std::cout << "Saving: " << ss.str() << std::endl;
           writer.write(ss.str(),*cloud_aligned_);


           *target += *cloud_aligned_;

   }

   pcl::PLYWriter writer;
   ss.str(""); ss << base_path.toStdString(); ss << "kinect_full.ply";
   std::cout << "Saving: " << ss.str() << std::endl;
   writer.write(ss.str(),*target,true,false);
*/

  // pcl::console::setVerbosityLevel(pcl::console::L_INFO);
}

void alignSTLScene(QString base_path){

    std::stringstream ss; ss << base_path.toStdString(); ss << "robot_positions.txt";
    std::vector<Affine3f> poses = loadRobotPoses(QString::fromStdString(ss.str()));

    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > cloudsInitial;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > cloudsICP;

    double scan_resolution = 0.0f;

    QDir view_directory(base_path);
    view_directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    QFileInfoList views = view_directory.entryInfoList();

    assert(poses.size() == views.size());

    //Rough alignment utilizing the robot poses
    Affine3f he = Affine3f::Identity();
    he.translation() << 164.1, -130.27, 135.6;
    AngleAxisf yawAngle(float(92.06 * 0.0174532925),Eigen::Vector3f::UnitX());
    AngleAxisf pitchAngle(0.93 * 0.0174532925,Eigen::Vector3f::UnitY());
    AngleAxisf rollAngle(101.95 * 0.0174532925,Eigen::Vector3f::UnitZ());
    Quaternionf f = rollAngle*pitchAngle*yawAngle;
    he.rotate (f);

    //Load Hand eye transform
    /*cv::Mat handeye = loadHandEye("stereo_he_alignment.yml");
    Affine3f he = Affine3f::Identity();
    he.translation() << static_cast<float>(handeye.at<double>(0,3)), static_cast<float>(handeye.at<double>(1,3)), static_cast<float>(handeye.at<double>(2,3));

    Matrix3f rot =  Matrix3f::Identity();
    rot<< static_cast<float>(handeye.at<double>(0,0)), static_cast<float>(handeye.at<double>(0,1)), static_cast<float>(handeye.at<double>(0,2)),
         static_cast<float>(handeye.at<double>(1,0)), static_cast<float>(handeye.at<double>(1,1)), static_cast<float>(handeye.at<double>(1,2)),
         static_cast<float>(handeye.at<double>(2,0)), static_cast<float>(handeye.at<double>(2,1)), static_cast<float>(handeye.at<double>(2,2));

    he.rotate(rot);
*/
    std::cout << "Hand eye transform loaded: \n" << he.matrix() << std::endl;

    unsigned int view_id= 0;
    std::vector<QString> base_paths;
    /// Loops through the found views Pos_xx
    foreach(const QFileInfo &view, views) {
        //Load .ply files
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::PLYReader reader;
        std::stringstream ss; ss << view.absoluteFilePath().toStdString(); ss << "/stl_"; ss << view_id; ss << ".ply";
        std::cout << "Loading: " << ss.str() << std::endl;
        if(reader.read(ss.str(),*cloud) < 0){
            pcl::console::print_error("Could not load point cloud ");// << ss.str());
            return;
        }

        clouds.push_back(cloud);

        //Align cloud to world coordinate system
        CloudNormal::Ptr source = cloud;
        Affine3f T_rob = poses[view_id];
        Matrix4f T_align = T_rob.matrix() * he.matrix();
        pcl::transformPointCloudWithNormals(*source,*source,T_align);
        cloudsInitial.push_back(source);

        base_paths.push_back(view.absoluteFilePath());
        //Save transformed model
        pcl::PLYWriter writer;
        ss.str(""); ss << view.absoluteFilePath().toStdString(); ss << "/stl_world_"; ss << view_id; ss << ".ply";
        std::cout << "Saving aligned cloud: " << ss.str() << std::endl;
        writer.write(ss.str(),*source,true,false);

        //Save alignment transform
        ss.str(""); ss << view.absoluteFilePath().toStdString(); ss << "/stl_to_world_trfm_"; ss << view_id; ss << ".txt";
        std::cout << "Saving alignment trasform: " << ss.str() << std::endl;
        std::ofstream file(ss.str());
        if (file.is_open())
            file << T_align;

        file.close();

        std::cout << std::endl;
        view_id++;
    }


    pcl::IterativeClosestPoint<PointTNormal,PointTNormal> icp;
    CloudNormal::Ptr target = cloudsInitial[0];
    for (size_t j = 2; j < cloudsInitial.size (); ++j){
            CloudNormal::Ptr source = cloudsInitial[j];

            CloudNormal::Ptr source_ds(new CloudNormal);
            CloudNormal::Ptr target_ds(new CloudNormal);
            CloudNormal tmp;

            assert(!source->empty());
            assert(!target->empty());

            const double voxel_size =  5 * computeResolution(source);

            //Downsample clouds
            pcl::VoxelGrid<PointTNormal> vg;
            vg.setInputCloud(source);
            vg.setLeafSize(voxel_size, voxel_size, voxel_size);
            vg.filter(*source_ds);

            vg.setInputCloud(target);
            vg.filter(*target_ds);

            const float inlier_threshold_icp = 5 * computeResolution(source_ds);

            PCL_INFO("Refining pose using ICP with an inlier threshold of %f...\n", inlier_threshold_icp);
            icp.setInputSource(source_ds);
            icp.setInputTarget(target_ds);
            icp.setMaximumIterations(100);
            //icp.setEuclideanFitnessEpsilon(1e9);
            //icp.setTransformationEpsilon(1e9);
            icp.setMaxCorrespondenceDistance(inlier_threshold_icp);
            icp.align(tmp);


           // PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
             if(!icp.hasConverged()) {
                PCL_ERROR("ICP failed!\n");
            return;
            }

             PCL_INFO("Rerunning ICP with an inlier threshold of %f...\n", 0.5 * inlier_threshold_icp);
             icp.setMaximumIterations(50);
             icp.setMaxCorrespondenceDistance(0.5 * inlier_threshold_icp);
             icp.align(tmp, icp.getFinalTransformation());

            //  PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
             if(!icp.hasConverged()) {
                 PCL_ERROR("Fine ICP failed!\n");
                 return;
             }

             PCL_INFO("Rerunning fine ICP at with an inlier threshold of %f...\n", 0.1 * inlier_threshold_icp);
         //    icp.setInputSource(source);
         //    icp.setInputTarget(cloud_dst_);
             icp.setMaximumIterations(25);
             icp.setMaxCorrespondenceDistance(0.1 * inlier_threshold_icp);
             icp.align(tmp, icp.getFinalTransformation());

         //    PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
            if(!icp.hasConverged()) {
                PCL_ERROR("Fine ICP failed!\n");
                return;
            }

             PCL_INFO("Rerunning fine ICP in full resolution with an inlier threshold of %f...\n", 0.05 * inlier_threshold_icp);
             icp.setInputSource(source);
             icp.setInputTarget(target);
             icp.setMaximumIterations(5);
             icp.setMaxCorrespondenceDistance(0.05 * inlier_threshold_icp);
             icp.align(tmp, icp.getFinalTransformation());

           //  PCL_INFO("\tFitness score = %f\n", icp.getFitnessScore());
            if(!icp.hasConverged()) {
                PCL_ERROR("Fine ICP failed!\n");
                return;
            }

             Matrix4f transform_ = Matrix4f::Identity();
            transform_ = icp.getFinalTransformation();


            CloudNormal::Ptr cloud_aligned_ (new CloudNormal);
            pcl::transformPointCloud<PointTNormal>(*source, *cloud_aligned_, transform_);
            cloudsICP.push_back(cloud_aligned_);

        /*    pcl::PLYWriter writer;
            ss.str(""); ss << "/home/thso"; ss << "/stl_align_src"; ss << j; ss << ".ply";
            std::cout << "Saving: " << ss.str() << std::endl;
                        writer.write(ss.str(),*cloud_aligned_,true,false);
                        */

       /*     pcl::PLYWriter writer;
            ss.str(""); ss << "/home/thso"; ss << "/stl_align_src"; ss << j; ss << ".ply";
            std::cout << "Saving: " << ss.str() << std::endl;
            writer.write(ss.str(),*cloud_aligned_,true,false);

            ss.str(""); ss << "/home/thso"; ss << "/stl_align_tar"; ss << j; ss << ".ply";
            std::cout << "Saving: " << ss.str() << std::endl;
            writer.write(ss.str(),*target,true,false);
*/
            *target += *cloud_aligned_;

    }

    pcl::PLYWriter writer;
    ss.str(""); ss << base_path.toStdString(); ss << "full_stl"; ss << ".ply";
    std::cout << "Saving: " << ss.str() << std::endl;
    writer.write(ss.str(),*target,true,false);


  /*  for (size_t j = 2; j < cloudsT.size (); ++j){
        CloudNormal::Ptr source = cloudsT[j];
        CloudNormal::Ptr target = cloudsT[j-1];

        assert(!source->empty());
        assert(!target->empty());

        //Registre each view
        CloudNormal tmp;
        Matrix4f transform_;
        const float inlier_threshold_icp =1.5;

/*        pcl::GeneralizedIterativeClosestPoint6D gicp6d;

        PCL_INFO("Refining pose using Color ICP with an inlier threshold of %f...\n", inlier_threshold_icp);
        gicp6d.setInputSource(source);
        gicp6d.setInputTarget(target);
        gicp6d.setMaximumIterations(200);
        gicp6d.setMaxCorrespondenceDistance(inlier_threshold_icp);
        gicp6d.align(tmp, transform_);

        if(!gicp6d.hasConverged()) {
           PCL_ERROR("Color-ICP failed!\n");
           return;
        }
  */
  /*      ss.str(""); ss << "/home/thso"; ss << "/src_"; ss << j; ss << ".ply";
        pcl::io::savePLYFile(ss.str(),*source);
        ss.str(""); ss << "/home/thso"; ss << "/tar_"; ss << j; ss << ".ply";
        pcl::io::savePLYFile(ss.str(),*target);

        pcl::IterativeClosestPoint<PointTNormal,PointTNormal> icp;
        PCL_INFO("Refining pose using ICP with an inlier threshold of %f...\n", inlier_threshold_icp);

        icp.setInputSource(source);
        icp.setInputTarget(target);
        icp.setMaximumIterations(200);
        //icp.setEuclideanFitnessEpsilon(1e9);
        //icp.setTransformationEpsilon(1e9);
        icp.setMaxCorrespondenceDistance(inlier_threshold_icp);
        icp.align(tmp);

        if(!icp.hasConverged()) {
          PCL_ERROR("ICP failed!\n");
          return;
        }


        PCL_INFO("Rerunning fine Color-ICP with an inlier threshold of %f...\n", 0.5 * inlier_threshold_icp);
        icp.setMaximumIterations(100);
        icp.setMaxCorrespondenceDistance(0.5 * inlier_threshold_icp);
        icp.align(tmp, icp.getFinalTransformation());

        if(!icp.hasConverged()) {
            PCL_ERROR("Fine ICP failed!\n");
            return;
        }

        PCL_INFO("Rerunning ultra-fine Color-ICP at full resolution with an inlier threshold of %f...\n", 0.1 * inlier_threshold_icp);
    //    icp.setInputSource(source);
    //    icp.setInputTarget(cloud_dst_);
        icp.setMaximumIterations(50);
        icp.setMaxCorrespondenceDistance(0.1 * inlier_threshold_icp);
        icp.align(tmp, icp.getFinalTransformation());

        if(!icp.hasConverged()) {
            PCL_ERROR("Ultra-fine ICP failed!\n");
            return;
        }

    transform_ = icp.getFinalTransformation();


        CloudNormal::Ptr cloud_aligned_ (new CloudNormal);
        pcl::transformPointCloud<PointTNormal>(*source, *cloud_aligned_, transform_);

        //Save transformed model
        pcl::PLYWriter writer;
        ss.str(""); ss << "/home/thso"; ss << "/stl_align_"; ss << j; ss << ".ply";
        std::cout << "Saving: " << ss.str() << std::endl;
        writer.write(ss.str(),*cloud_aligned_,true,false);

    }




    // save meshlab aln project file
/*        std::ofstream s(QString("%1/alignment.aln").arg(directory).toLocal8Bit());
    s << pointCloudData.size() << std::endl;
    for(unsigned int i=0; i<pointCloudData.size(); i++){
        QString fileName = QString("pointcloud_%1.ply").arg(i);
        s << fileName.toStdString() << std::endl << "#" << std::endl;
        cv::Mat Tr = cv::Mat::eye(4, 4, CV_32F);
        cv::Mat(pointCloudData[i].R.t()).copyTo(Tr.colRange(0, 3).rowRange(0, 3));
        cv::Mat(-pointCloudData[i].R.t()*pointCloudData[i].T).copyTo(Tr.col(3).rowRange(0, 3));
        for(int j=0; j<Tr.rows; j++){
            for(int k=0; k<Tr.cols; k++){
                s << Tr.at<float>(j,k) << " ";
            }
            s << std::endl;
        }
    }
    s << "0" << std::flush;
    s.close();
*/

}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QCoreApplication::setOrganizationName("DTU");
    QCoreApplication::setOrganizationDomain("dtu.dk");
    QCoreApplication::setApplicationName("DTU_reconstruction");

   // settings = new QSettings("/home/thso/DTU_reconstruction.conf");

    QStringList args = a.arguments();

    QRegExp argsHelp("-h");
    QRegExp argsAll("-all");
    QRegExp argsSingle("-single");
    QRegExp argsStereo("-stereo");
    QRegExp argsStl("-stl");
    QRegExp argsKinect("-kinect");
    QRegExp argsReconstruct("-reconstruct");
    QRegExp argsCalib("-calibrate");
    QRegExp argsAlign("-align");

    QString folder_name;
    QString calib_path;
    bool reconstruct_all = false;
    bool reconstruct_single = false;
    bool reconstruct_stereo = false;
    bool stl = false;
    bool kinect = false;
    bool calibrate = false;
    bool align = false;
    bool reconstruct = false;

    //Test
  /*  unsigned int screenCols = 1024;
    unsigned int screenRows = 800;
    std::vector<cv::Mat> patterns;
    int nGrayBits;
    int nLineShifts = 8;

    int nTotalBits = ceilf(log2f((float)screenCols));

   // determine the necessary Gray code bits and add some robustness
   nGrayBits = nTotalBits - floorf(log2f((float)nLineShifts)) + 2;

   unsigned int N = 2 + 2*nGrayBits + nLineShifts;


   // line shifts
        for(unsigned int p=0; p<nLineShifts; p++){
            cv::Mat pattern(1, screenCols, CV_8UC3, cv::Scalar(0));

       //     for(unsigned int k = 0; k<screenRows; k++){
                for(unsigned int j=p; j<screenCols; j+= nLineShifts){
                   // std::cout << " " << j << " ";//<< std::endl;
                    pattern.at<cv::Vec3b>(0, j) = cv::Vec3b(255, 255, 255);
                }
                   //std::cout << std::endl;
                   //std::cout << std::endl;
                   //std::cout << std::endl;
            //}
std::cout << " " << pattern << std::endl;
            patterns.push_back(pattern);
        }

   for(unsigned int i=0; i<patterns.size(); i++){
       std::stringstream ss; ss << "/home/thso/thoso_test/line_shift/img_"; ss << i; ss << ".png";
       std::cout << "save pattern " << i << std::endl;

       cv::imwrite(ss.str(), patterns[i]);

   }
  std::cout << "FINISH!!!!!!!" << std::endl;
*/



    //Parse command line arguments
    for (int i = 1; i < args.size(); ++i){
        if (argsHelp.indexIn(args.at(i)) != -1){
            help();
            return 0;
        }
        else if (argsAll.indexIn(args.at(i)) != -1 ){
            reconstruct_all = true;
        } else if (argsSingle.indexIn(args.at(i)) != -1 ){
            reconstruct_single = true;
        }else if (argsStereo.indexIn(args.at(i)) != -1 ){
            reconstruct_stereo = true;
        }else if (argsStl.indexIn(args.at(i)) != -1 ){
            stl = true;
        }else if (argsKinect.indexIn(args.at(i)) != -1 ){
            kinect = true;
        }else if (argsReconstruct.indexIn(args.at(i)) != -1 ){
            folder_name = args.at(i+1);
            reconstruct = true;
        }else if (argsAlign.indexIn(args.at(i)) != -1 ){
             folder_name = args.at(i+1);
             align = true;
        }else if (argsCalib.indexIn(args.at(i)) != -1 ){
            calib_path = args.at(i+1);
            calibrate = true;
        }
    }

    /// Check variables
    if(folder_name.isEmpty() && !calibrate){
        std::cout << "Please provide the path to the images!! Help -h" << std::endl;
        return 0;
    }

    if(calib_path.isEmpty() && calibrate){
        std::cout << "Please provide the path to the calibration images!! Help -h" << std::endl;
        return 0;
    }


    ///------------------------ Calibrate the scene ----------------------------------///
     if(calibrate){
       std::vector<SMCalibrationSet> calibration_set;
       loadCalibrationImages(calib_path,calibration_set);
       std::cout << "Loaded " << calibration_set.size() << " calibration sets" << std::endl;
       SMCalibrationWorker calibrationWorker;
       calibrationWorker.performCalibration(calibration_set);

       //TODO: Calibrate Left to kinect camera
       std::cout << "Calibrating Left to kinect " << std::endl;

     }

    std::vector<SceneData> scenes;
/*   if(reconstruct_all){
    //    loadAllData(folder_name, scenes);
     //   reconstructStereo(folder_name);
        std::cout << scenes.size() << " scene(s) are loaded with " <<  scenes.at(0).data.size() + 2 << " views in each" << std::endl;
    }
*/

    if(reconstruct_stereo && reconstruct_single){

        QDir view_directory(folder_name);
        view_directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
        QFileInfoList views = view_directory.entryInfoList();

        unsigned int view_id= 0;
        ViewData view_data(view_id);

        /// Loops through the found views Pos_xx
        foreach(const QFileInfo &view, views) {

         view_data.setViewNr(view_id);
         loadView(view.absoluteFilePath(), view_data);
         reconstructStereo(view_data, view_id);

         view_data.clear();
         view_id++;
        }
    }

    if(stl && reconstruct){
        QDir view_directory(folder_name);
        view_directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
        QFileInfoList views = view_directory.entryInfoList();

        unsigned int view_id= 0;
        ViewData view_data(view_id);
        /// Loops through the found views Pos_xx
        foreach(const QFileInfo &view, views) {

         view_data.setViewNr(view_id);
         loadView(view.absoluteFilePath(), view_data);

          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = reconstructSTL(view_data,view_id);

         std::stringstream ss; ss << view.absoluteFilePath().toStdString(); ss << "/stl_"; ss << view_id; ss << ".ply";
         std::cout << "Save to: " << ss.str() << std::endl;
         pcl::PLYWriter w;
         // Write to ply in binary without camera
         w.write<pcl::PointXYZRGBNormal> (ss.str(), *cloud, true, false);
       //  pcl::io::savePLYFileBinary(ss.str(), *pointCloudPCL);
         std::cout << ".....Done!!" << std::endl;

         view_data.clear();

         view_id++;
        }

    }

      if(kinect && reconstruct){
           std::cout << "Kinect data is already reconstructed.......saving and scaling as .pcd .ply!!!" << std::endl;
          //Load kinect clouds
          QDir view_directory(folder_name);
          view_directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
          QFileInfoList views = view_directory.entryInfoList();

          unsigned int view_id= 0;
          /// Loops through the found views Pos_xx
          foreach(const QFileInfo &view, views) {
           //Load .ply files
           Cloud::Ptr cloud(new Cloud);
           pcl::PCDReader reader;
           std::stringstream ss_pcd; ss_pcd << view.absoluteFilePath().toStdString(); ss_pcd << "/kinect/kinect_cloud.pcd";
           std::cout << "Loading: " << ss_pcd.str() << std::endl;
           if(reader.read(ss_pcd.str(),*cloud) == 0){

               std::vector<int> indices;
               pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

               //Remove Nans
                CloudPtr cloud_scaled(new Cloud);
                for(size_t j = 0; j < cloud->size();++j){
                   pcl::PointXYZRGB p = cloud->points[j];
                   p.x = p.x * 1000.0f;
                   p.y = p.y * 1000.0f;
                   p.z = p.z * 1000.0f;
                   cloud_scaled->points.push_back(p);
                }

               std::stringstream ss;
               ss.str(""); ss << view.absoluteFilePath().toStdString(); ss << "/kinect/kinect_"; ss << view_id; ss <<".ply";
               std::cout << "Saving: " << ss.str() << std::endl;
               pcl::PLYWriter w;
               w.write<PointT> (ss.str(), *cloud_scaled, true, false);
           }


           view_id++;
          }

          return 0;
      }

    //Align STL scene
    if(align && stl)
        alignSTLScene(folder_name);
    if(align && kinect)
        alignKinectScene(folder_name);



   /* else{
        if(!folder_name.isEmpty()){
            std::cout << "Loding scene: " << folder_name.toStdString() << std::endl;
           SceneData scene;
           loadscene(folder_name, scene);
        //   for(int j = 0; j<scene.data.size(); j++)
        //     std::cout << "hej " << scene.data[j]->getKinectCloud()->points.size() << std::endl;
           scenes.push_back(scene);
            // ViewData view(0);
           // loadView(folder_name, view);

            alignKinectScene(scene);
        //reconstructStereo(scenes);
        }
    }
*/
    std::cout << "Loaded " << scenes.size() << " scene(s)!" << std::endl;





    return 0;
}
