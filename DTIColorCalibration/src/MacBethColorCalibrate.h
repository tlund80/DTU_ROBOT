#ifndef MACBETHCOLORCALIBRATE_H
#define MACBETHCOLORCALIBRATE_H

//include OpenCV
#include <opencv2/opencv.hpp>

class MacBethColorCalibrate
{
public:
    MacBethColorCalibrate();

    ~MacBethColorCalibrate();

    void colorCalibrate(cv::Mat input, cv::Mat output, cv::Mat calibration);
    void printError(void);
    void save(std::string path, int camLorR);

    void getColorCenters(cv::Mat& colorCenters, const int& ulx, const int& uly, const int& urx, const int& ury, const int& llx, const int& lly, const int& lrx, const int& lry);

    void colorNormalize(cv::Mat image, cv::Mat &normalizedColorImage, cv::Point2i whiteRegion,  cv::Point2i blackRegion, int l_margin, int h_margin);
    double checkBoundary(double input);
    cv::Mat computeColorCalibration(cv::Mat &inputImage, int colorspace, int ulx, int uly, int urx, int ury, int llx, int lly, int lrx, int lry, int camLorR);
    void getError(cv::Mat inputImage, int colorspace, int ulx, int uly, int urx, int ury, int llx, int lly, int lrx, int lry, int ColumnError, int camLorR);

private:
   cv::Mat getColorsFromRig(const cv::Mat &Image,  cv::Mat colorCenters);
   cv::Mat computeNormalization(cv::Mat image, cv::Point2i whiteRegion, cv::Point2i blackRegion, int l_margin, int h_margin);

private:
      //Global Parameter
    int bigger;
    int smaller;
    // two images; 24 colors; 3 channels
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

    int ulxL, urxL, llxL, lrxL, ulyL, uryL, llyL, lryL;
    int ulxR, urxR, llxR, lrxR, ulyR, uryR, llyR, lryR;
    int whiteRegionXL, whiteRegionYL, blackRegionXL,  blackRegionYL;
    int whiteRegionXR, whiteRegionYR, blackRegionXR,  blackRegionYR;

    cv::Mat calibration_matrix[2];

};

#endif // MACBETHCOLORCALIBRATE_H
