#ifndef MACBETHCOLORCALIBRATE_H
#define MACBETHCOLORCALIBRATE_H

//include OpenCV
#include <opencv2/opencv.hpp>

class MacBethColorCalibrate
{
public:
    MacBethColorCalibrate();

    ~MacBethColorCalibrate();

    void calibrate(cv::Mat input);
    void colorCalibrate(cv::Mat input, cv::Mat output, cv::Mat calibration);
    void printError(void);

//private:
    void getColorCenters(cv::Mat& colorCenters, const int& ulx, const int& uly, const int& urx, const int& ury, const int& llx, const int& lly, const int& lrx, const int& lry);
    cv::Mat getColorsFromRig(const cv::Mat &Image,  cv::Mat colorCenters);
    cv::Mat computeNormalization(cv::Mat image, int whiteRegionX, int whiteRegionY, int blackRegionX, int blackRegionY, int l_margin = 10, int h_margin = 10);
    void colorNormalize(cv::Mat image, cv::Mat &normalizedColorImage, double scaling, double translation);
    double checkBoundary(double input);
    cv::Mat computeColorCalibration(cv::Mat &inputImage, int colorspace, int ulx, int uly, int urx, int ury, int llx, int lly, int lrx, int lry);
    void getError(cv::Mat inputImage, int colorspace, int ulx, int uly, int urx, int ury, int llx, int lly, int lrx, int lry, int ColumnError, int camLorR);


private:
      //Global Parameter
    int bigger;
    int smaller;
    double TestError[2][24][3];
    double TestRigColors[2][24][3][3];
    const int LEFT_IMAGE_ID = 1;
    const int RIGHT_IMAGE_ID = 2;

    int allConnerL[8];
    int allConnerR[8];
    int WBCenterL[4];
    int WBCenterR[4];


    int counterL1 = 0;
    int counterL2 = 0;
    int counterR1 = 0;
    int counterR2 = 0;

    int ulxL, urxL, llxL, lrxL, ulyL, uryL, llyL, lryL;
    int ulxR, urxR, llxR, lrxR, ulyR, uryR, llyR, lryR;
    int whiteRegionXL, whiteRegionYL, blackRegionXL,  blackRegionYL;
    int whiteRegionXR, whiteRegionYR, blackRegionXR,  blackRegionYR;

};

#endif // MACBETHCOLORCALIBRATE_H
