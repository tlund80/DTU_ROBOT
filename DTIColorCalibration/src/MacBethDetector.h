#ifndef MACBETHDETECTOR_H
#define MACBETHDETECTOR_H

//standard headers
#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <numeric>
#include <functional>
#include <signal.h>
#include <cmath>

//include OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#define MACBETH_WIDTH   6
#define MACBETH_HEIGHT  4
#define MACBETH_SQUARES MACBETH_WIDTH * MACBETH_HEIGHT

#define MAX_CONTOUR_APPROX  7


class MacBethDetector
{  
public:
    MacBethDetector();
    ~MacBethDetector();

    void findColorCenters(cv::Mat input_image, cv::Point upperleft, cv::Point lowerright);


private:
    cv::Rect find_quad(cv::Mat contours, int min_size);
    std::vector<cv::RotatedRect> findColorSquares(cv::Mat input_image, int method = 0);
    void drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2);
    double board_orentation(std::vector<cv::RotatedRect> squares, cv::Mat img = cv::Mat());
    void median_filter(std::vector<cv::RotatedRect>& vec);
    void histogram_filtering(std::vector<cv::RotatedRect>& rect);
    bool colorSort(std::vector<cv::RotatedRect> &rect,  cv::Mat &img);
    void sortAndID(std::vector<cv::RotatedRect> rect,  cv::Mat img = cv::Mat());

};

//The RGB ground truth values of the 24 colors
static double ArrayTrueRGBValues[72] = {115., 194.,  98.,  87., 133., 103., 214.,  80., 193.,  94., 157., 224.,  56.,  70., 175., 231., 187.,   8., 243., 200., 160., 122., 85., 52.,
                                 82., 150., 122., 108., 128., 189., 126.,  91.,  90.,  60., 188., 163.,  61., 148.,  54., 199.,  86., 133., 243., 200., 160., 122., 85., 52.,
                                 68., 130., 157.,  67., 177., 170.,  44., 166.,  99., 108.,  64.,  46., 150.,  73.,  60.,  31., 149., 161., 242., 200., 160., 121., 85., 52.
                                 };


#endif // MACBETHDETECTOR_H
