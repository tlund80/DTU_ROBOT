#ifndef SMTYPES_H
#define SMTYPES_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QString>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointT> Cloud;
typedef pcl::PointXYZRGBNormal PointTNormal;
typedef pcl::PointCloud<PointTNormal> CloudNormal;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;


struct SMColorCalibrationData {
    // Input data
    int id;
    cv::Mat image;
    int ulx;
    int uly;
    int urx;
    int ury;
    int llx;
    int lly;
    int lrx;
    int lry;
    int whiteRegionX;
    int whiteRegionY;
    int blackRegionX;
    int blackRegionY;

    //Output
    cv::Mat normalized_image;
    cv::Mat calibrated_image;
    cv::Mat calibration_matrix;
    cv::Mat colorCenters;

    SMColorCalibrationData():
        ulx(0), uly(0), urx(0), ury(0), llx(0), lly(0), lrx(0), lry(0),
        whiteRegionX(0), whiteRegionY(0), blackRegionX(0), blackRegionY(0), colorCenters(3, 24, CV_64F) {}
};

struct SMCalibrationSet {
    // frames. if one channel it is BG Bayer. if three it is color.
    cv::Mat frame0;
    cv::Mat frame1;
    int id;
    float rotationAngle;
    bool checked;
    cv::Mat frame0Result;
    cv::Mat frame1Result;
    SMColorCalibrationData colorCalibration0;
    SMColorCalibrationData colorCalibration1;

    SMCalibrationSet(): id(-1), rotationAngle(0), checked(false){}
};

struct SMFrameSequence {
    // raw bayer frames
    std::vector<cv::Mat> frames0;
    std::vector<cv::Mat> frames1;
    int id;
    QString codec;
    float rotationAngle;
    bool reconstructed;
    SMFrameSequence(): id(-1), codec(""), rotationAngle(0), reconstructed(false){}
};


struct SMPointCloud {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloud;
    float rotationAngle;
    int id;
    cv::Matx33f R;
    cv::Vec3f T;
    SMPointCloud(): pointCloud(), rotationAngle(-1), id(-1){}
};

#endif // SMTYPES_H
