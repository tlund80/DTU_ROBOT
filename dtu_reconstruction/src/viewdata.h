#ifndef VIEWDATA_H
#define VIEWDATA_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <opencv2/opencv.hpp>
#include "SMTypes.h"


class ViewData
{
public:

    ViewData(unsigned int view_number);

    bool loadKinectcloud(std::string path);
    bool loadKinectRGB(std::string path);
    bool loadKinectDepth(std::string path);

    bool loadStereoTexture(std::string path_left, std::string path_right);
    bool loadStereoRGB(std::string path_left, std::string path_right);
    bool loadStructuredLight(std::vector<std::string> left_path, std::vector<std::string> right_path);

    SMFrameSequence getSTLFramesequence(void){return stl;}
    CloudPtr getKinectCloud(void){return kinect_cloud_;}
    cv::Mat getKinectRGB(void){return kinect_rgb_;}
    cv::Mat getkinectDepth(void){return kinect_depth_;}
    std::pair<cv::Mat, cv::Mat> getStereoTexturePair(void){return std::pair<cv::Mat, cv::Mat>(stereo_left_texture_, stereo_right_texture_);}
    std::pair<cv::Mat, cv::Mat> getStereoRGBPair(void){return std::pair<cv::Mat, cv::Mat>(stereo_left_rgb_, stereo_right_rgb_);}

    inline void setViewNr(unsigned int view_number){ view_number_ = view_number;}
    inline unsigned int getViewNr(void){ return view_number_;}

    inline void clear(){
        stl.frames0.clear();
        stl.frames1.clear();
        kinect_cloud_->clear();
        kinect_rgb_.release();
        kinect_depth_.release();
        stereo_left_rgb_.release();
        stereo_right_rgb_.release();
        stereo_left_texture_.release();
        stereo_right_texture_.release();
    }

private:

    unsigned int view_number_;

    CloudPtr kinect_cloud_;
    cv::Mat kinect_rgb_;
    cv::Mat kinect_depth_;

    cv::Mat stereo_left_rgb_;
    cv::Mat stereo_right_rgb_;

    cv::Mat stereo_left_texture_;
    cv::Mat stereo_right_texture_;

    SMFrameSequence stl;

};

#endif // VIEWDATA_H
