#ifndef MACBETHCOLORCALIBRATE_H
#define MACBETHCOLORCALIBRATE_H

//include OpenCV2
#include <opencv2/opencv.hpp>
#include "SMTypes.h"

class MacBethColorCalibrate
{
public:
    MacBethColorCalibrate();

    ~MacBethColorCalibrate();

    void calibrate(SMColorCalibrationData &data);
    void colorCalibrate(cv::Mat input, cv::Mat output, cv::Mat calibration);
    void printError(void);

//private:
    void getColorCenters(SMColorCalibrationData &data);
    cv::Mat getColorsFromRig(const cv::Mat &Image,  cv::Mat colorCenters);
    cv::Mat computeNormalization(cv::Mat image, int whiteRegionX, int whiteRegionY, int blackRegionX, int blackRegionY, int l_margin = 10, int h_margin = 10);
    void colorNormalize(cv::Mat image, cv::Mat &normalizedColorImage, double scaling, double translation);
    double checkBoundary(double input);
    cv::Mat computeColorCalibration(cv::Mat &inputImage, int colorspace);
    void getError(cv::Mat inputImage, int colorspace, int ColumnError, int camLorR);


private:
    //Global Parameter
    SMColorCalibrationData cc_data;
    int bigger;
    int smaller;
    double TestError[2][24][3];
    double TestRigColors[2][24][3][3];
    const int LEFT_IMAGE_ID;
    const int RIGHT_IMAGE_ID;

    int allConnerL[8];
    int allConnerR[8];
    int WBCenterL[4];
    int WBCenterR[4];

    int counterL1;
    int counterL2;
    int counterR1;
    int counterR2;

    int lower_margin;
    int higher_margin;

};

#endif // MACBETHCOLORCALIBRATE_H
