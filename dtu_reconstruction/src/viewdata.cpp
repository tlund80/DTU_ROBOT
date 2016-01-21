#include "viewdata.h"
#include <pcl/io/pcd_io.h>

ViewData::ViewData(unsigned int view_number) : view_number_(view_number){
    kinect_cloud_.reset(new Cloud);
}

bool ViewData:: loadKinectcloud(std::string path){

    if (pcl::io::loadPCDFile<PointT> (path, *kinect_cloud_) == -1) {
        std::cerr << "Couldn't read PCD file " << path << std::endl;
        return false;
       }else
        return true;

}

bool ViewData::loadKinectRGB(std::string path){
    assert(!path.empty());

    kinect_rgb_ = cv::imread(path,cv::IMREAD_UNCHANGED);

    if( kinect_rgb_.empty() ){
      std::cerr <<  "Could not open or find the kinect rgb image for view nr: " << view_number_ << std::endl ;
      return false;
    }else
        return true;

}

bool ViewData::loadKinectDepth(std::string path){
     assert(!path.empty());

    kinect_depth_ = cv::imread(path,cv::IMREAD_UNCHANGED);

    if( kinect_depth_.empty() ){
      std::cerr <<  "Could not open or find the kinect depth image for view nr: " << view_number_ << std::endl ;
      return false;
    }else
        return true;

}

bool ViewData::loadStereoTexture(std::string path_left, std::string path_right){

    assert(!path_left.empty());
    assert(!path_right.empty());

    stereo_left_texture_ = cv::imread(path_left,cv::IMREAD_UNCHANGED);
    stereo_right_texture_ = cv::imread(path_right,cv::IMREAD_UNCHANGED);

    if( stereo_left_texture_.empty() ){
      std::cerr <<  "Could not open or find the left stereo image for view nr: " << view_number_ << std::endl ;
      return false;
    }
    if( stereo_right_texture_.empty() ){
      std::cerr <<  "Could not open or find the right stereo image for view nr: " << view_number_ << std::endl ;
      return false;
    }

    return true;
}

bool ViewData::loadStereoRGB(std::string path_left, std::string path_right){

    assert(!path_left.empty());
    assert(!path_right.empty());

    stereo_left_rgb_ = cv::imread(path_left,cv::IMREAD_UNCHANGED);
    stereo_right_rgb_ = cv::imread(path_right,cv::IMREAD_UNCHANGED);

    if( stereo_left_rgb_.empty() ){
      std::cerr <<  "Could not open or find the left stereo image for view nr: " << view_number_ << std::endl ;
      return false;
    }
    if( stereo_right_rgb_.empty() ){
      std::cerr <<  "Could not open or find the right stereo image for view nr: " << view_number_ << std::endl ;
      return false;
    }

    return true;
}

bool ViewData::loadStructuredLight(std::vector<std::string> left_path, std::vector<std::string> right_path){

    assert(left_path.size() > 0);
    assert(right_path.size() > 0);

    stl.id = view_number_;
    stl.codec = "GrayCode";
    stl.reconstructed = false;

    cv::Mat tmp_left, tmp_right;
    for(unsigned int i = 0; i<left_path.size(); i++){
  //      std::cout << "Loading left structured light images from -> " << left_path[i] << std::endl;
  //       std::cout << "Loading right structured light images from -> " << right_path[i] << std::endl;
        tmp_left = cv::imread(left_path[i],cv::IMREAD_UNCHANGED);
        tmp_right = cv::imread(right_path[i],cv::IMREAD_UNCHANGED);

        if( tmp_left.empty() ){
            std::cerr <<  "Could not open or find the left structured light image nr: " << i << " for view nr: " << view_number_ << std::endl ;
          return false;
        }
        if( tmp_right.empty() ){
          std::cerr <<  "Could not open or find the right structured light image nr: " << i << " for view nr: " << view_number_ << std::endl ;
          return false;
        }
        stl.frames0.push_back(tmp_left);
        stl.frames1.push_back(tmp_right);

    }

    return true;
}

