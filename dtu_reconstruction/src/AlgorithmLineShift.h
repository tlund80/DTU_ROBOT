#ifndef AlgorithmLineShift_H
#define AlgorithmLineShift_H

#include "Algorithm.h"

class AlgorithmLineShift : public Algorithm {
    public:
        AlgorithmLineShift(unsigned int _screenCols, unsigned int _screenRows);
    virtual ~AlgorithmLineShift(){}
        unsigned int getNPatterns(){return N;}
        int getscreenCols(){return screenCols;}
        int getscreenRows(){return screenRows;}
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
        // Matching
        void get3DPoints(SMCalibrationParameters calibration, const std::vector<cv::Mat>& frames0, const std::vector<cv::Mat>& frames1, std::vector<cv::Point3f>& Q, std::vector<cv::Vec3b>& color);
    protected:
        std::vector<cv::Mat> patterns;
        int nGrayBits;
};

#endif // AlgorithmLineShift_H
