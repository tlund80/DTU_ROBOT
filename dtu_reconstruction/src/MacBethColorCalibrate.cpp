#include "MacBethColorCalibrate.h"

#include <iomanip>

MacBethColorCalibrate::MacBethColorCalibrate() : counterL1(0),
                                                 counterL2(0),
                                                 counterR1(0),
                                                 counterR2(0),
                                                 LEFT_IMAGE_ID(1),
                                                 RIGHT_IMAGE_ID(2),
                                                 lower_margin(10),
                                                 higher_margin(10)
{
    bigger = 0;
    smaller = 0;

    for(int i = 0; i < 8; i++)
            allConnerL[i] = 0;
    for(int i = 0; i < 8; i++)
            allConnerR[i] = 0;
    for(int i = 0; i < 4; i++)
            WBCenterL[i] = 0;
    for(int i = 0; i < 4; i++)
            WBCenterR[i] = 0;

}

MacBethColorCalibrate::~MacBethColorCalibrate()
{

}

void MacBethColorCalibrate::getColorCenters(SMColorCalibrationData& data) {
   double templateCornersArray[] = {5.0 - 1, 239.0 - 1, 1.0, 5.0 - 1, 5.0 - 1, 1.0, 325.0 - 1, 5.0 - 1, 1.0, 325.0 - 1, 239.0 - 1, 1.0};
   //double templateCornersArray[] = {5.0, 239.0, 1.0, 5.0, 5.0, 1.0, 325.0, 5.0, 1.0, 325.0, 239.0, 1.0};
   cv::Mat templateCornersT = cv::Mat(4, 3, CV_64F, &templateCornersArray);

   cv::Mat templateCorners = cv::Mat(3, 4, CV_64F);
   cv::transpose(templateCornersT, templateCorners);

   cv::Mat templateColorCenters = cv::Mat(24, 3, CV_64F);
   cv::Mat templateColorCentersT = cv::Mat(3, 24, CV_64F);

   int numberOfColumns = 6;
   int margin_x = 25;
   int margin_y = 19;
   int spacing_x = 54;
   int spacing_y = 64;


   for (int j = 0; j < 24; j++) {
      templateColorCenters.at<double>(j,0) = double((j % numberOfColumns) * spacing_x + margin_x);
      templateColorCenters.at<double>(j,1) = double(floor(j / numberOfColumns) * spacing_y + margin_y);
      templateColorCenters.at<double>(j,2) = 1.0f;
   }

   cv::transpose(templateColorCenters, templateColorCentersT);

   double cornersArray[] = {double(data.llx), double(data.lly), 1.0, double(data.ulx), double(data.uly), 1.0, double(data.urx), double(data.ury), 1.0, double(data.lrx), double(data.lry), 1.0};


   cv::Mat cornersT = cv::Mat(4, 3, CV_64F, &cornersArray);
   cv::Mat corners = cv::Mat(3, 4, CV_64F);
   cv::transpose(cornersT, corners);

   //The transformation matrix, between the template color calibration rig, and
   //the rig's pose in the pictures (defined by the 4 corner points) is found
   //transMatrix = corners * (templateCorners'*inv(templateCorners*templateCorners'));
   cv::Mat temp1 = cv::Mat(3, 3, CV_64F);
   cv::gemm( templateCorners, templateCornersT, 1, cv::Mat(), 0, temp1);

   cv::Mat temp2 = cv::Mat(3, 3, CV_64F);
   cv::invert(temp1, temp2);

   cv::Mat temp3 = cv::Mat(4, 3, CV_64F);
   cv::gemm( templateCornersT, temp2, 1, cv::Mat(), 0, temp3);

   cv::Mat transMatrix = cv::Mat(3, 3, CV_64F);
   cv::gemm( corners, temp3, 1, cv::Mat(), 0, transMatrix);

   //colorCenters = transMatrix*templateColorCenters;
  // cv::Mat colorCenters = cv::Mat(3, 24, CV_64F);
   cv::gemm(transMatrix, templateColorCentersT, 1, cv::Mat(), 0, data.colorCenters);
}

cv::Mat  MacBethColorCalibrate::getColorsFromRig(const cv::Mat &Image,  cv::Mat colorCenters) {

   cv::Mat rigColors(24, 3, CV_64F);
   double intervalMargin = 10.;
   cv::Mat vis_img;
   vis_img = Image.clone();

     // cv::Mat colorCentersT = cv::Mat(3, 24, CV_64F);
    //  cv::transpose(colorCenters, colorCentersT);
//   std::cout << colorCentersT << std::endl;

   for (int i = 0; i < 24; i++) {
      rigColors.at<double>(i,0) = 0;
      rigColors.at<double>(i,1) = 0;
      rigColors.at<double>(i,2) = 0;
   }



   for (int i = 0; i < 24; i++) {

     int x1 = int(round(colorCenters.at<double>(0,i) - intervalMargin));
     int y1 = int(round(colorCenters.at<double>(1,i) - intervalMargin));
     int x2 = int(round(colorCenters.at<double>(0,i) + intervalMargin));
     int y2 = int(round(colorCenters.at<double>(1,i) + intervalMargin));

    // std::cout << "Upperleft = (" << x1 << "," << y1 << ") " << "Lowerright = (" << x2 << "," << y2 << ")" << std::endl;
     cv::Rect myROI( x1, y1, x2-x1, y2-y1);
     //Get the color of each rectangle
     cv::Mat region = Image(myROI);
     cv::Scalar mean_color = cv::mean(region);
     rigColors.at<double>(i,0) = mean_color.val[2];
     rigColors.at<double>(i,1) = mean_color.val[1];
     rigColors.at<double>(i,2) = mean_color.val[0];


     cv::rectangle(vis_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 127, 255), 1);
   }

   //   cv::namedWindow("picked regions", 1 );
   //    cv::imshow("picked regions", vis_img);
   //    cv::waitKey(0);

   return rigColors;
}

cv::Mat  MacBethColorCalibrate::computeNormalization(cv::Mat image, int whiteRegionX, int whiteRegionY, int blackRegionX, int blackRegionY, int l_margin, int h_margin) {
   cv::Mat greyImage(cv::Size(image.cols, image.rows), IPL_DEPTH_64F);
   uchar *cdata;
   double *gdata;
   gdata = (double *)greyImage.data;
   cdata = (uchar *)image.data;

   for (int r = 0; r < image.rows; r++) {
      for (int c = 0; c < image.cols; c++) {
         gdata[r*image.cols+c] = ((double)cdata[(r*image.cols+c)*3]) / 3.0 + ((double)cdata[(r*image.cols+c)*3+1]) / 3.0 + ((double)cdata[(r*image.cols+c)*3+2]) / 3.0;
      }
   }

   //Find values of black and white regions
   double whiteValue = 0;
   int counter = 0;

   for (int r = whiteRegionY - 5; r <= whiteRegionY + 5; r++) {
      for (int c = whiteRegionX - 5; c <= whiteRegionX + 5; c++) {
         whiteValue = whiteValue + (double) gdata[r*image.cols+c];
         counter++;
      }
   }

   whiteValue = whiteValue / counter;

   double blackValue = 0;
   counter = 0;
   for (int r = blackRegionY - 5; r <= blackRegionY + 5; r++) {
      for (int c = blackRegionX - 5; c <= blackRegionX + 5; c++) {
         blackValue = blackValue + (double) gdata[r*image.cols+c];
         counter++;
      }
   }
   blackValue = blackValue / counter;

   double currentRange = whiteValue - blackValue;
   int desiredRange = 256 - (l_margin + h_margin);
   double scaling = desiredRange / currentRange;
   double translation = l_margin - blackValue;

   cv::Mat ScalANDTrans(2, 1, CV_64F);
   ScalANDTrans.at<double>(0) = scaling;
   ScalANDTrans.at<double>(1) = translation;

   //cvReleaseImage(&greyImage);

   return ScalANDTrans;
}

void MacBethColorCalibrate::colorNormalize(cv::Mat image, cv::Mat &normalizedColorImage, double scaling, double translation) {
    image.convertTo(normalizedColorImage,image.type(),scaling,translation);
    //std::cout << normalizedColorImage.rows << " " << normalizedColorImage.cols << std::endl;
    //  CvConvertScale(image, normalizedColorImage, scaling, translation);
    // cvNormalize(image, normalizedColorImage, scaling, translation, CV_L2, NULL);
}


double MacBethColorCalibrate::checkBoundary(double input) {

   if (input > 255.) {
      bigger++;
      return 255.;
   }
   if (input < 0.) {
      smaller++;
      return 0.;
   }

   return input;
}

void MacBethColorCalibrate::calibrate(SMColorCalibrationData& data){

    cv::Mat ScalANDTrans;
        cv::Mat normalizedColorImage;
        cv::Mat resizednormalizedColorImage;
        cv::Mat resizedcalibratedImage;
        cv::Mat calibrationMatrix;
        cv::Mat calibratedImage;


        cc_data = data;
        std::cout << "************************ Color Calibration Data ************************ " << std::endl;
        std::cout << "The following data are applied:" << std::endl;
        std::cout << "\t Coordinate of upper left cornor of MacBeth target = (" << cc_data.ulx << "," << cc_data.uly << ")" << std::endl;
        std::cout << "\t Coordinate of upper right cornor of MacBeth target = (" << cc_data.urx << "," << cc_data.ury << ")" << std::endl;
        std::cout << "\t Coordinate of lower left cornor of MacBeth target = (" << cc_data.llx << "," << cc_data.lly << ")" << std::endl;
        std::cout << "\t Coordinate of lower right cornor of MacBeth target = (" << cc_data.lrx << "," << cc_data.lry << ")" << std::endl;
        std::cout << std::endl;
        std::cout << "\t Center coordinate of the white square of MacBeth target = (" << cc_data.whiteRegionX << "," << cc_data.whiteRegionY << ")" << std::endl;
        std::cout << "\t Center coordinate of the black square of MacBeth target = (" << cc_data.blackRegionX << "," << cc_data.blackRegionY << ")" << std::endl;
        std::cout << "********************************* End ********************************** " << std::endl;

        bool test = false;
        if(test){
           //Check points
           cv::Mat tempForVisualization;
           data.image.copyTo(tempForVisualization);

           //Draw the click point
           cv::circle(tempForVisualization, cv::Point(cc_data.ulx, cc_data.uly), 5, cv::Scalar(0, 255, 255), 2);
           cv::circle(tempForVisualization, cv::Point(cc_data.urx, cc_data.ury), 5, cv::Scalar(0, 255, 255), 2);
           cv::circle(tempForVisualization, cv::Point(cc_data.llx, cc_data.lly), 5, cv::Scalar(0, 255, 255), 2);
           cv::circle(tempForVisualization, cv::Point(cc_data.lrx, cc_data.lry), 5, cv::Scalar(0, 255, 255), 2);

           cv::circle(tempForVisualization, cv::Point(cc_data.whiteRegionX, cc_data.whiteRegionY), 5, cv::Scalar(0, 0, 255), 2);
           cv::circle(tempForVisualization, cv::Point(cc_data.blackRegionX, cc_data.blackRegionY), 5, cv::Scalar(0, 0, 255), 2);

           cv::namedWindow("Clicked Points", 1 );
           cv::imshow("Clicked Points", tempForVisualization);

       }
   //     std::cout << "ulx: " << cc_data.ulx << " uly: " << cc_data.uly << " urx: " << cc_data.urx << " ury: " << cc_data.ury
   //               << " llx: " << cc_data.llx << " lly: " <<  cc_data.lly << " lrx: " << cc_data.lrx << " lry: " <<  cc_data.lry << std::endl;
       getError(cc_data.image, 0, 0, 0);


   //    std::cout << "ulx: " << cc_data.ulx << " uly: " << cc_data.uly << " urx: " << cc_data.urx << " ury: " << cc_data.ury
   //              << " llx: " << cc_data.llx << " lly: " <<  cc_data.lly << " lrx: " << cc_data.lrx << " lry: " <<  cc_data.lry << std::endl;

       ScalANDTrans = computeNormalization(cc_data.image, cc_data.whiteRegionX, cc_data.whiteRegionY, cc_data.blackRegionX, cc_data.blackRegionY, lower_margin, higher_margin);

       colorNormalize(cc_data.image, normalizedColorImage, ScalANDTrans.at<double>(0), ScalANDTrans.at<double>(1));

       if(test){
           cv::resize( normalizedColorImage, resizednormalizedColorImage, normalizedColorImage.size());
           cv::namedWindow("Normalized Image", 1 );
           cv::imshow("Normalized Image", resizednormalizedColorImage);
       }
       getError(normalizedColorImage, 0, 0, 0);
       calibrationMatrix = computeColorCalibration(normalizedColorImage, 0);

       std::cout << "************************ Color Calibration Result ************************ " << std::endl;
       std::cout << "Calibration Matrix:" << std::endl;
       std::cout << calibrationMatrix << std::endl;
       std::cout << "*********************************** End ********************************** " << std::endl;

       calibratedImage = cv::Mat(normalizedColorImage.size(),normalizedColorImage.type());
       colorCalibrate(normalizedColorImage, calibratedImage, calibrationMatrix);

       if(test){
           cv::resize(calibratedImage, resizedcalibratedImage, normalizedColorImage.size() );
           cv::namedWindow("Calibrated Image", 1);
           cv::imshow("Calibrated Image", resizedcalibratedImage);
           cv::waitKey(0);
       }

       getError(calibratedImage, 0, 2, 0);

       //Save the data in struct
       normalizedColorImage.copyTo(data.normalized_image);
       calibratedImage.copyTo(data.calibrated_image);
       calibrationMatrix.copyTo(data.calibration_matrix);
       cc_data.colorCenters.copyTo(data.colorCenters);

}
/**
  * Apply color calibration to cv::Mat datastructure
 */
void MacBethColorCalibrate::colorCalibrate(cv::Mat input, cv::Mat output, cv::Mat calibration) {

   if (input.channels()  != output.channels() || input.channels() != calibration.rows)
      std::cout << "colorCalibrate(): There seems to be a problem with dimensions on input data." << std::endl;

   cv::Mat RGB_vector(1, 3, CV_64F);
   cv::Mat RGB_calibrated(1, 3, CV_64F);

   for (int i = 0; i < input.rows; ++i){
  //  cv::Vec3b* pixel = input.ptr<cv::Vec3b>(i); // point to first pixel in row
    for (int j = 0; j < input.cols; ++j){
        // read R, G and B values from image, apply calibration matrix and store results
        // calibration matrix is loaded from matlab, so R and B needs to switch place before madoubletrix multiplication
        cv::Vec3b pixel = input.at<cv::Vec3b>(i,j); // point to first pixel in row
        RGB_vector.at<double>(0) = double(pixel.val[0]);
         RGB_vector.at<double>(1) = double(pixel.val[1]);
         RGB_vector.at<double>(2) = double(pixel.val[2]);

         cv::gemm(RGB_vector, calibration, 1, cv::Mat(), 0, RGB_calibrated);

         output.at<cv::Vec3b>(i,j)[0] = (uchar) checkBoundary(RGB_calibrated.at<double>(0,0));
         output.at<cv::Vec3b>(i,j)[1] = (uchar) checkBoundary(RGB_calibrated.at<double>(0,1));
         output.at<cv::Vec3b>(i,j)[2] = (uchar) checkBoundary(RGB_calibrated.at<double>(0,2));
          }
      }
}


cv::Mat MacBethColorCalibrate::computeColorCalibration(cv::Mat& inputImage, int colorspace) {

   if (colorspace != 0 && colorspace != 1 && colorspace != 2)
      std::cout << "Error. Colorspace must be either 0 (for RGB), 1 (for XYZ) or 2 (CIELAB) " << std::endl;

   //The RGB ground truth values of the 24 colors
   double ArrayTrueRGBValues[72] = {115., 194.,  98.,  87., 133., 103., 214.,  80., 193.,  94., 157., 224.,  56.,  70., 175., 231., 187.,   8., 243., 200., 160., 122., 85., 52.,
                                    82., 150., 122., 108., 128., 189., 126.,  91.,  90.,  60., 188., 163.,  61., 148.,  54., 199.,  86., 133., 243., 200., 160., 122., 85., 52.,
                                    68., 130., 157.,  67., 177., 170.,  44., 166.,  99., 108.,  64.,  46., 150.,  73.,  60.,  31., 149., 161., 242., 200., 160., 121., 85., 52.
                                    };

   //The XYZ ground truth values of the 24 colors
 /*  double ArraytrueXYZValues[72] = {0.3673, 0.6482, 0.4636, 0.3573, 0.5469, 0.5807, 0.5828, 0.3940, 0.5349, 0.3289, 0.5922, 0.6559,
                                    0.2936, 0.3918, 0.4221, 0.7098, 0.5560, 0.3297, 0.9522, 0.7843, 0.6275, 0.4777, 0.3333, 0.2039,
                                    0.3451, 0.6193, 0.4683, 0.3944, 0.5200, 0.6641, 0.5443, 0.3689, 0.4414, 0.2772, 0.6763, 0.6570,
                                    0.2586, 0.4941, 0.3144, 0.7595, 0.4393, 0.4252, 0.9527, 0.7843, 0.6275, 0.4781, 0.3333, 0.2039,
                                    0.2760, 0.5228, 0.5966, 0.2817, 0.6700, 0.6702, 0.2196, 0.6128, 0.3909, 0.4019, 0.3107, 0.2430,
                                    0.5433, 0.3183, 0.2407, 0.2076, 0.5599, 0.6087, 0.9495, 0.7843, 0.6275, 0.4750, 0.3333, 0.2039
                                   };

   //The LAB ground truth values of the 24 colors
   double ArraytrueLABValues[72] = {65.3676,  82.8746,  74.0822,  69.0696,  77.2808,  85.2038, 78.7120, 67.1955, 72.3214,  59.6381,  85.8214, 84.8420,
                                    58.0593,  75.7065,  62.8754,  89.8377,  72.1830,  71.2309, 98.1398, 90.9764, 83.3081,  74.7080,  64.4299, 52.2777,
                                    7.3567,   6.5411,  -1.2970, -11.8913,   6.8289, -19.0831,  9.4137,  7.9426, 25.1873,  19.1059, -19.0066, -0.2318,
                                    14.3613, -29.4116,  35.1000, -10.1857,  31.0540, -30.5820, -0.0794,      0.,      0.,  -0.1258,       0.,      0.,
                                    10.0802,   9.3515, -13.0485,  15.5587, -14.1794,  -0.5312, 42.6350, -26.4364, 6.0408, -17.1894,  40.0955, 49.0573,
                                    -35.5258,  21.5644,  11.5775,  64.0507, -12.8032, -19.0996,  0.2164,       0.,     0.,   0.3431,       0.,      0.
                                   };
*/
   cv::Mat calibrationMatrix(3, 3, CV_64F);
   cv::Mat trueRGBValues = cv::Mat(3, 24, CV_64F, ArrayTrueRGBValues);

   cv::Mat trueRGBValuesT(24, 3, CV_64F);
 //  cv::Mat inverstMatrix(3, 24, CV_64F);
   cv::Mat rigColorsT(3, 24, CV_64F);

 //  cv::Mat colorCenters = cv::Mat(3, 24, CV_64F);
   //getColorCenters(colorCenters, ulx, uly, urx, ury, llx, lly, lrx, lry);
   getColorCenters(cc_data);

   cv::Mat rigColors(24, 3, CV_64F);
   rigColors = getColorsFromRig(inputImage, cc_data.colorCenters);      //The 24 rig colors are extracted from the picture

   cv::transpose(rigColors, rigColorsT);
   cv::transpose(trueRGBValues, trueRGBValuesT);

   //std::cout << "rigColors: \n" << rigColors << std::endl;
   //std::cout << "trueRGBValues: \n" <<trueRGBValuesT << std::endl;

   cv::solve(rigColors, trueRGBValuesT, calibrationMatrix, cv::DECOMP_SVD);

 //  std::cout << "calibrationMatrix:\n " << calibrationMatrix << std::endl;

   //Since the calibration matrix in OpenCV has difference value position with matlab. so we switch the position
/*   cv::Mat temp(2, 3, CV_64F);
   //for(int i=0; i<2;i++)
   temp.data[0] = calibrationMatrix.data[0];
   temp.data[1] = calibrationMatrix.data[1];
   temp.data[2] = calibrationMatrix.data[2];
   temp.data[3] = calibrationMatrix.data[6];
   temp.data[4] = calibrationMatrix.data[7];
   temp.data[5] = calibrationMatrix.data[8];

   calibrationMatrix.data[0] = temp.data[3];
   calibrationMatrix.data[1] = temp.data[4];
   calibrationMatrix.data[2] = temp.data[5];
   calibrationMatrix.data[6] = temp.data[0];
   calibrationMatrix.data[7] = temp.data[1];
   calibrationMatrix.data[8] = temp.data[2];
*/
   /*
   std::cout << std::endl;
   std::cout << "Check compute Calibration Matrix after Switch: " << std::endl;
   for(int r = 0; r<3; r++){
        for(int c=0; c<3; c++){
        std::cout << " " << calibrationMatrix->data.db[r*3+c];
        }
        std::cout << std::endl;
   }
   */

   return calibrationMatrix;
}


void MacBethColorCalibrate::getError(cv::Mat inputImage, int colorspace, int ColumnError, int camLorR) {

   if (colorspace != 0 && colorspace != 1 && colorspace != 2)
      std::cout << "Error. Colorspace must be either 0 (for RGB), 1 (for XYZ) or 2 (CIELAB) " << std::endl;

   //The RGB ground truth values of the 24 colors
   double trueRGBValues[3][24] = {115, 194,  98,  87, 133, 103, 214,  80, 193,  94, 157, 224,  56,  70, 175, 231, 187,   8, 243, 200, 160, 122, 85, 52,
                                  82, 150, 122, 108, 128, 189, 126,  91,  90,  60, 188, 163,  61, 148,  54, 199,  86, 133, 243, 200, 160, 122, 85, 52,
                                  68, 130, 157,  67, 177, 170,  44, 166,  99, 108,  64,  46, 150,  73,  60,  31, 149, 161, 242, 200, 160, 121, 85, 52
                                 };

   //The XYZ ground truth values of the 24 colors
   double trueXYZValues[3][24] = {0.3673, 0.6482, 0.4636, 0.3573, 0.5469, 0.5807, 0.5828, 0.3940, 0.5349, 0.3289, 0.5922, 0.6559,
                                  0.2936, 0.3918, 0.4221, 0.7098, 0.5560, 0.3297, 0.9522, 0.7843, 0.6275, 0.4777, 0.3333, 0.2039,
                                  0.3451, 0.6193, 0.4683, 0.3944, 0.5200, 0.6641, 0.5443, 0.3689, 0.4414, 0.2772, 0.6763, 0.6570,
                                  0.2586, 0.4941, 0.3144, 0.7595, 0.4393, 0.4252, 0.9527, 0.7843, 0.6275, 0.4781, 0.3333, 0.2039,
                                  0.2760, 0.5228, 0.5966, 0.2817, 0.6700, 0.6702, 0.2196, 0.6128, 0.3909, 0.4019, 0.3107, 0.2430,
                                  0.5433, 0.3183, 0.2407, 0.2076, 0.5599, 0.6087, 0.9495, 0.7843, 0.6275, 0.4750, 0.3333, 0.2039
                                 };

   //The LAB ground truth values of the 24 colors
   double trueLABValues[3][24] = {65.3676,  82.8746,  74.0822,  69.0696,  77.2808,  85.2038, 78.7120, 67.1955, 72.3214,  59.6381,  85.8214, 84.8420,
                                  58.0593,  75.7065,  62.8754,  89.8377,  72.1830,  71.2309, 98.1398, 90.9764, 83.3081,  74.7080,  64.4299, 52.2777,
                                  7.3567,   6.5411,  -1.2970, -11.8913,   6.8289, -19.0831,  9.4137,  7.9426, 25.1873,  19.1059, -19.0066, -0.2318,
                                  14.3613, -29.4116,  35.1000, -10.1857,  31.0540, -30.5820, -0.0794,      0.,      0.,  -0.1258,       0.,      0.,
                                  10.0802,   9.3515, -13.0485,  15.5587, -14.1794,  -0.5312, 42.6350, -26.4364, 6.0408, -17.1894,  40.0955, 49.0573,
                                  -35.5258,  21.5644,  11.5775,  64.0507, -12.8032, -19.0996,  0.2164,       0.,     0.,   0.3431,       0.,      0.
                                 };

   //Calculate the coordinates of the 24 colors (centers of the squares), based on the four corner points
   cv::Mat colorCenters = cv::Mat(3, 24, CV_64F);
  // getColorCenters(colorCenters, ulx, uly, urx, ury, llx, lly, lrx, lry);
   getColorCenters(cc_data);
   cv::Mat rigColors = getColorsFromRig(inputImage, cc_data.colorCenters);
   cv::Mat colorErrors(1, 24, CV_64F);

   if (colorspace == 0)
      for (int i = 0; i < 24; i++) {
         colorErrors.at<double>(i) = double (sqrt(double (pow((trueRGBValues[0][i] - rigColors.at<double>(i,0)), 2))
                                                +
                                                double (pow((trueRGBValues[1][i] - rigColors.at<double>(i,1)), 2))
                                                +
                                                double (pow((trueRGBValues[2][i] - rigColors.at<double>(i,2)), 2))));
      }

   if (colorspace == 1)
      for (int i = 0; i < 24; i++) {
         colorErrors.at<double>(i) = double (sqrt(double (pow((trueXYZValues[0][i] - rigColors.at<double>(i,0)), 2))
                                                +
                                                double (pow((trueXYZValues[1][i] - rigColors.at<double>(i,1)), 2))
                                                +
                                                double (pow((trueXYZValues[2][i] - rigColors.at<double>(i,2)), 2))));

         std::cout << " color Errors " << i << " :" << colorErrors.at<double>(i) << std::endl;
      }

   if (colorspace == 2)
      for (int i = 0; i < 24; i++) {
         colorErrors.at<double>(i) = double (sqrt(double (pow((trueLABValues[0][i] - rigColors.at<double>(i,0)), 2))
                                                +
                                                double (pow((trueLABValues[1][i] - rigColors.at<double>(i,1)), 2))
                                                +
                                                double (pow((trueLABValues[2][i] - rigColors.at<double>(i,2)), 2))));

         std::cout << " color Errors " << i << " :" << colorErrors.at<double>(i) << std::endl;
      }


   for (int i = 0; i < 24; i++)
      TestError[camLorR][i][ColumnError] = colorErrors.at<double>(i);

   for (int i = 0; i < 24; i++) {
      TestRigColors[camLorR][i][ColumnError][0] = rigColors.at<double>(i,0);
      TestRigColors[camLorR][i][ColumnError][1] = rigColors.at<double>(i,1);
      TestRigColors[camLorR][i][ColumnError][2] = rigColors.at<double>(i,2);
   }

}


void MacBethColorCalibrate::printError(void){

    //Display the BGR value of each color
    std::cout << std::endl;

    std::cout << std::setw(10) << "Blue-O" << std::setw(10) << "Blue-C" << std::setw(10) << "Red-O" << std::setw(10) << "Red-C" << std::setw(10) << "Green-O" << std::setw(10) << "Green-C";

    std::cout << std::setw(20) << "Blue-O" << std::setw(10) << "Blue-C" << std::setw(10) << "Red-O" << std::setw(10) << "Red-C" << std::setw(10) << "Green-O" << std::setw(10) << "Green-C";

    std::cout << std::endl;
    for (int i = 0; i < 24; i++) {
       if (i < 9)
          std::cout << std::setw(2) << i + 1 << std::setw(5);
       else
          std::cout << i + 1 << std::setw(5);

       for (int camLorR = 0; camLorR < 2; camLorR++) {
          for (int k = 0; k < 3; k++) {
             std::cout << std::setw(10) << TestRigColors[camLorR][i][0][k] << std::setw(10) << TestRigColors[camLorR][i][2][k] << std::setw(10);
          }
          std::cout << "  ||   ";
       }
       std::cout << std::endl;
    }

    std::cout << "***************************************************************************************************************************************" << std::endl;

     for(int j=0; j<24; j++){
         if (j < 9)
            std::cout << std::setw(2) << j + 1 << std::setw(5);
         else
            std::cout << j + 1 << std::setw(5);

        for(int i=0; i<2; i++){
          for(int k=0; k<3; k++){
              std::cout<< std::setw(10) << TestError[i][j][k]<< std::setw(10);
          }
          std::cout<<"  ||   ";
          }
          std::cout<<std::endl;
     }
     std::cout << "***************************************************************************************************************************************" << std::endl;

     double TotalColorErrors[2][3];

     for (int i = 0; i < 2; i++)
         for (int j = 0; j < 3; j++)
             TotalColorErrors[i][j] = 0;

     for (int i = 0; i < 2; i++)
         for (int j = 0; j < 3; j++)
             for (int k = 0; k < 24; k++)
                TotalColorErrors[i][j] = TotalColorErrors[i][j] + TestError[i][k][j];


          std::cout << "Average Color Errors Left  O : " << TotalColorErrors[0][0] / 24 << " N : " << TotalColorErrors[0][1] / 24 << " C : " << TotalColorErrors[0][2] / 24 << std::endl;

          std::cout << "Average Color Errors Right O : " << TotalColorErrors[1][0] / 24 << " N : " << TotalColorErrors[1][1] / 24 << " C : " << TotalColorErrors[1][2] / 24 << std::endl;



}
