#include <QtCore/QCoreApplication>
#include <QDebug>
#include <QStringList>
#include <QRegExp>
#include <QDir>
#include <QSettings>

#include "stereo_match.h"
#include "viewdata.h"
#include "scenedata.h"

#include <pcl/common/bivariate_polynomial.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <vector>

#include "SMCalibrationWorker.h"
#include "SMCalibrationParameters.h"
#include "AlgorithmGrayCode.h"



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
            if(num == 0)
                left_rgb_stereo_path = file_path.toStdString();


            if(num == 20){
                    left_stereo_path = file_path.toStdString();
               //     std::cout << "Loading left from: " << file_path.toStdString() << std::endl;

                 }else if(num < 20)
                    stl_left.push_back(file_path.toStdString());

                     num++;
        }else if(subfolder.baseName().compare("right",Qt::CaseInsensitive) == 0){
                QString file_path = subfolder.absoluteFilePath() + "/" + file;
            //    std::cout << "Loading right from: " << file_path.toStdString() << std::endl;

                    if(num == 0)
                        right_rgb_stereo_path = file_path.toStdString();

                    if(num == 20){
                        right_stereo_path = file_path.toStdString();
                    }else if(num < 20)
                        stl_right.push_back(file_path.toStdString());

                        num++;
             }
        }
     }

     single_view.loadStereoTexture(left_stereo_path,right_stereo_path);
     single_view.loadStereoRGB(left_rgb_stereo_path,right_rgb_stereo_path);
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

    QSettings settings;
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
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reconstructSTL(ViewData &view, unsigned int view_id){

    std::cout << "Reconstruct view nr: " << view.getViewNr() << ".....";

    SMCalibrationParameters calibration;// = settings.value("calibration/parameters").value<SMCalibrationParameters>();
    calibration.importFromXML("Calibration.xml");
   // calibration.print();

    AlgorithmGrayCode* gc = new AlgorithmGrayCode(1024, 800); //1024
    std::vector<cv::Point3f> Q;
    std::vector<cv::Vec3b> color;

    SMFrameSequence frames = view.getSTLFramesequence();

    gc->get3DPoints(calibration,frames.frames0, frames.frames1,Q,color);

    std::cout << "Q: " << Q.size() << std::endl;

    // Convert point cloud to PCL format
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudPCL(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    pointCloudPCL->width = Q.size();
    pointCloudPCL->height = 1;
    pointCloudPCL->is_dense = true;
    pointCloudPCL->points.resize(Q.size());

   for(unsigned int i=0; i<Q.size(); i++){
      pcl::PointXYZRGBNormal point;
      point.x = Q[i].x; point.y = Q[i].y; point.z = Q[i].z;
      point.r = color[i][0]; point.g = color[i][1]; point.b = color[i][2];
      pointCloudPCL->points[i] = point;
   }

/*   std::stringstream ss; ss << "/home/thso/stl_"; ss << view_id; ss << ".ply";
   pcl::PLYWriter w;
   // Write to ply in binary without camera
   w.write<pcl::PointXYZRGBNormal> (ss.str(), *pointCloudPCL, true, false);
 //  pcl::io::savePLYFileBinary(ss.str(), *pointCloudPCL);
   std::cout << ".....Done!!" << std::endl;
*/

  delete gc;

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

void alignKinectScene(SceneData &scene){

    CloudPtr cloud(new Cloud);

    std::vector<Affine3d> T;

    Affine3d transform = Affine3d::Identity();
    // pose 1
    transform.translation() << 603.03, -858.81, 280.29;
    transform.rotate (Quaterniond(0.092, 0.701, 0.701, -0.092));
    T.push_back(transform);
    // pose 2
    transform.translation() << 603.02, -413.53, 743.32;
    transform.rotate (Quaterniond(0.327, 0.627, 0.627, -0.326));
    T.push_back(transform);
    // pose 3
    transform.translation() << 1209.44, -605.17, 280.29;
    transform.rotate (Quaterniond(0.121, 0.379, 0.916, -0.05));
    T.push_back(transform);
    // pose 4
    transform.translation() << 894.57, -290.29, 743.32;
    transform.rotate (Quaterniond(0.427, 0.339, 0.819, -0.177));
    T.push_back(transform);
    // pose 5
    transform.translation() << 1358.84, 3.02, 280.29;
    transform.rotate (Quaterniond(0.131, 0.0, 0.991, 0.0));
    T.push_back(transform);
    // pose 6
    transform.translation() << 1358.83, 3.01, 173.19;
    transform.rotate (Quaterniond(0.131, 0.0, 0.991, 0.0));
    T.push_back(transform);
    // pose 7
    transform.translation() << 1013.56, 3.02, 743.32;
    transform.rotate (Quaterniond(0.462, 0.0, 0.887, 0.0));
    T.push_back(transform);
    // pose 8
    transform.translation() << 1205.18, 609.44, 280.29;
    transform.rotate (Quaterniond(0.121, -0.379, 0.916, 0.05));
    T.push_back(transform);
    // pose 9
    transform.translation() << 890.31, 294.57, 743.33;
    transform.rotate (Quaterniond(0.427, -0.339, 0.819, 0.177));
    T.push_back(transform);
    // pose 10
    transform.translation() << 597.0, 858.84, 280.28;
    transform.rotate (Quaterniond(0.092, -0.701, 0.701, 0.092));
    T.push_back(transform);
    // pose 11
    transform.translation() << 597.0, 413.57, 743.33;
    transform.rotate (Quaterniond(0.327, -0.627, 0.627, 0.327));
    T.push_back(transform);


    //Load Hand eye transform
   cv::Mat handeye = loadHandEye("/home/thso/DTUData/calibration/131215/kinect/HandEyeAlignment.yml");
   Affine3d he = Affine3d::Identity();
   he.translation() << handeye.at<double>(0,3), handeye.at<double>(1,3), handeye.at<double>(2,3);

   Matrix3d rot =  Matrix3d::Identity();
   rot<< handeye.at<double>(0,0), handeye.at<double>(0,1), handeye.at<double>(0,2),
                          handeye.at<double>(1,0), handeye.at<double>(1,1), handeye.at<double>(1,2),
                          handeye.at<double>(2,0), handeye.at<double>(2,1), handeye.at<double>(2,2);
   he.rotate(rot);

   std::cout << "Hand eye transform: \n" << he.matrix() << std::endl;

    // Print the transformation
//    printf ("\nMethod #2: using an Affine3f\n");
//    std::cout << transform.matrix() << std::endl;

    Affine3d acc = Affine3d::Identity();
    CloudPtr out (new Cloud);

    for(int i = 1; i< scene.data.size(); i++){
        boost::shared_ptr<ViewData> v = scene.data.at(i);

        Affine3d tr = T[i];
        Affine3d tr_1 = T[i-1];

 std::cout << "T: \n" << tr.translation() << std::endl;
  std::cout << "T-1: \n" << tr_1.translation() << std::endl;
   std::cout << "T-1 - T: \n" << tr_1.translation()-tr.translation() << std::endl;
     //   std::cout << "Tool frame: \n" << tr.matrix() << std::endl;


          acc.translate(tr_1.translation()-tr.translation());
        //  tr.rotate(rot);
          std::cout << "Camera frame: \n" <<  tr.matrix() << std::endl;


      //  std::cout << "Camera frame: \n" <<  tr.matrix() << std::endl;


        pcl::transformPointCloud(*v->getKinectCloud(),*out,acc);
        // *cloud += *out;
        std::stringstream ss; ss << "/home/thso/full_kinect_"; ss << i; ss << ".pcd";
          pcl::io::savePCDFileBinaryCompressed(ss.str(), *out);
    }



}

void alignSTLScene(SceneData &scene){

}

void computeBoundingBox( pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_filtered, Eigen::Vector3f &tra, Eigen::Quaternionf &rot,
    pcl::PointXYZ &min_pt,pcl::PointXYZ &max_pt){
   // Placeholder for the 3x3 covariance matrix at each surface patch
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f xyz_centroid;

  pcl::console::print_highlight ("Computing centroid...\n");
  pcl::compute3DCentroid(*cloud_filtered,xyz_centroid);

  pcl::console::print_highlight ("Computing bounding box...\n");
  Eigen::Matrix3f covariance;
  // Compute the 3x3 covariance matrix
  pcl::computeCovarianceMatrixNormalized(*cloud_filtered, xyz_centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3,3>(0,0) = eigDx.transpose();
  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * xyz_centroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ> cPoints;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::copyPointCloud(*cloud_filtered, cloud_xyz);
  pcl::transformPointCloud(cloud_xyz, cPoints, p2w);

  // std::cout << "centroid -> x: " << xyz_centroid[0] << " y: " << xyz_centroid[1] << " z: " <<  xyz_centroid[2] << std::endl;
 // PointT min_pt;
 // PointT max_pt;


  pcl::getMinMax3D(cPoints, min_pt, max_pt);
 // std::cout << "min_pt: \n " << min_pt << std::endl;
 // std::cout << "max_pt: \n " << max_pt << std::endl;
  Eigen::Vector3f mean_diag;
  mean_diag[0] = 0.5f*(max_pt.x + min_pt.x);
  mean_diag[1] = 0.5f*(max_pt.y + min_pt.y);
  mean_diag[2] = 0.5f*(max_pt.z + min_pt.z);

  // final transform
  //const Eigen::Quaternionf
  rot = Eigen::Quaternionf(eigDx);
  tra = eigDx*mean_diag + xyz_centroid.head<3>();
  std::cout << "tra-> x: " << tra[0] << " y: " << tra[1] << " z: " << tra[2] << std::endl;


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
    QRegExp argsFolder("-folder");
    QRegExp argsCalib("-calibrate");
    QRegExp argsAlign("-align");

    QString folder_name;
    QString calib_path;
    bool reconstruct_all = false;
    bool reconstruct_single = false;
    bool reconstruct_stereo = false;
    bool reconstruct_stl = false;
    bool calibrate = false;
    bool align = false;

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
            reconstruct_stl = true;
        }else if (argsFolder.indexIn(args.at(i)) != -1 ){
            folder_name = args.at(i+1);
        }else if (argsAlign.indexIn(args.at(i)) != -1 ){
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

    if(reconstruct_stl && reconstruct_single){
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

    //Align STL scene
    if(align){
        std::stringstream ss; ss << folder_name.toStdString(); ss << "robot_positions.txt";
        std::vector<Affine3f> poses = loadRobotPoses(QString::fromStdString(ss.str()));

        std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > clouds;
        std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > cloudsT;

        QDir view_directory(folder_name);
        view_directory.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
        QFileInfoList views = view_directory.entryInfoList();

         if(poses.size() != views.size()){
             return 0;
         }

        unsigned int view_id= 0;
        /// Loops through the found views Pos_xx
        foreach(const QFileInfo &view, views) {
         //Load .ply files
         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
         pcl::PLYReader reader;
         std::stringstream ss; ss << view.absoluteFilePath().toStdString(); ss << "/stl_"; ss << view_id; ss << ".ply";
         std::cout << "Loading: " << ss.str() << std::endl;
         if(reader.read(ss.str(),*cloud));
            clouds.push_back(cloud);

         view_id++;
        }


        Affine3f he = Affine3f::Identity();
        he.translation() << 164.1, -130.27, 135.6;
        AngleAxisf yawAngle(float(92.06 * 0.0174532925),Eigen::Vector3f::UnitX());
        AngleAxisf pitchAngle(0.93 * 0.0174532925,Eigen::Vector3f::UnitY());
        AngleAxisf rollAngle(101.95 * 0.0174532925,Eigen::Vector3f::UnitZ());
        Quaternionf f = rollAngle*pitchAngle*yawAngle;
        he.rotate (f);


         for (size_t i = 1; i < clouds.size (); ++i){
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source = clouds[i-1];
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target = clouds[i];

            Affine3f T_rob = poses[i-1];
            Matrix4f T_align = T_rob.matrix() * he.matrix();

          /*  std::cout << "----------------------------------------------------\n" << std::endl;
            std::cout << "HE: \n" << he.matrix() << std::endl;
            std::cout << "Robot pose: \n" << T_rob.matrix() << std::endl;
            std::cout << "T_align: \n" <<  T_align << std::endl;
            std::cout << "----------------------------------------------------\n" << std::endl;
           */
            pcl::transformPointCloud(*source,*source,T_align);

            //Registre each view

            //Save transformed model
            pcl::PLYWriter writer;
            ss.str(""); ss << "/home/thso"; ss << "/stl_world_"; ss << i; ss << ".ply";
            std::cout << "Saving: " << ss.str() << std::endl;
            writer.write(ss.str(),*source);

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
