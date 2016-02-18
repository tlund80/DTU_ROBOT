#ifndef CVTOOLS_H
#define CVTOOLS_H

#include <opencv2/opencv.hpp>

namespace cvtools{
    void cvtColorBGRToBayerBG(const cv::Mat &imBGR, cv::Mat &imBayerBG);
    void removeIncorrectMatches(const cv::Mat F, const std::vector<cv::Point2f> &q0, const std::vector<cv::Point2f> &q1, const float maxD,
                                    std::vector<cv::Point2f> q0Correct, std::vector<cv::Point2f> q1Correct);
    void rshift(cv::Mat& I, unsigned int shift);
    void drawChessboardCorners(cv::InputOutputArray _image, cv::Size patternSize, cv::InputArray _corners, bool patternWasFound, int line_width=1);
    void cvDrawChessboardCorners(CvArr* _image, CvSize pattern_size, CvPoint2D32f* corners, int count, int found, int line_width=1);
    cv::Mat modulo(const cv::Mat &mat, float n);
    void matToPoints3f(const cv::Mat &mat, std::vector<cv::Point3f> &points);
    void convertMatFromHomogeneous(cv::Mat &src, cv::Mat &dst);
    void handEyeCalibrationTsai(const std::vector<cv::Matx33f> R, const std::vector<cv::Vec3f> t, const std::vector<cv::Matx33f> R_mark, const std::vector<cv::Vec3f> t_mark, cv::Matx33f &Omega, cv::Vec3f &tau);
    void rotationAxisCalibration(const std::vector< std::vector<cv::Point3f> > Qcam, const std::vector<cv::Point3f> Qobj, cv::Vec3f &axis, cv::Vec3f &point);
    void fitSixDofData(const std::vector<cv::Matx33f> R, const std::vector<cv::Vec3f> t, const std::vector<cv::Matx33f> R_mark, const std::vector<cv::Vec3f> t_mark, cv::Matx33f &Omega, cv::Vec3f &tau);
    void initDistortMap(const cv::Matx33f cameraMatrix, const cv::Vec<float, 5> distCoeffs, const cv::Size size, cv::Mat &map1, cv::Mat &map2);
    cv::Mat diamondDownsample(cv::Mat &pattern);
    void imshow(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
    void imagesc(const char* windowName, cv::Mat im);
    cv::Mat histimage(cv::Mat histogram);
    void hist(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
    void writeMat( cv::Mat const& mat, const char* filename, const char* varName = "A", bool bgr2rgb = true);
}

#endif // CVTOOLS_H
