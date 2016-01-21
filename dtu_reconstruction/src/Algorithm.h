#ifndef CODEC_H
#define CODEC_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "SMCalibrationParameters.h"

// Base class for all Algorithms
class Algorithm {
    public:
        Algorithm(unsigned int _screenCols, unsigned int _screenRows) : N(0), screenCols(_screenCols), screenRows(_screenRows){}
    virtual ~Algorithm(){}
        unsigned int getNPatterns(){return N;}
        int getscreenCols(){return screenCols;}
        int getscreenRows(){return screenRows;}
        // Encoding
        virtual cv::Mat getEncodingPattern(unsigned int depth) = 0;
        // Matching
        virtual void get3DPoints(SMCalibrationParameters calibration, const std::vector<cv::Mat>& frames0, const std::vector<cv::Mat>& frames1, std::vector<cv::Point3f>& Q, std::vector<cv::Vec3b>& color) = 0;
    protected:
        unsigned int N;
        unsigned int screenCols, screenRows;
};


#endif // CODEC_H
