#include <QCoreApplication>

// standard headers
#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <numeric>
#include <functional>
#include <signal.h>
//#include <sys/times.h>
#include <iomanip>
//#include <stdio.h>

//include OpenCV
#include <opencv2/opencv.hpp>

#include "MacBethColorCalibrate.h"
#include "MacBethDetector.h"


#define MACBETH_WIDTH   6
#define MACBETH_HEIGHT  4
#define MACBETH_SQUARES MACBETH_WIDTH * MACBETH_HEIGHT

#define MAX_CONTOUR_APPROX  7


//Global Parameter
const int LEFT_IMAGE_ID = 1;
const int RIGHT_IMAGE_ID = 2;

int allConnerL[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int allConnerR[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int WBCenterL[4] = {0, 0, 0, 0};
int WBCenterR[4] = {0, 0, 0, 0};

int counterL1 = 0;
int counterL2 = 0;
int counterR1 = 0;
int counterR2 = 0;

int ulxL, urxL, llxL, lrxL, ulyL, uryL, llyL, lryL;
int ulxR, urxR, llxR, lrxR, ulyR, uryR, llyR, lryR;
int whiteRegionXL, whiteRegionYL, blackRegionXL,  blackRegionYL;
int whiteRegionXR, whiteRegionYR, blackRegionXR,  blackRegionYR;


void mouseEvent(int event, int x, int y, int flags, void *param) {

  // std::cout << std::endl;
  // std::cout << " Click at The 4 Conners:: Start at TopLeft -> TopRight -> LowerLeft -> LowerRight  : ";

   if ( *(int*)param == LEFT_IMAGE_ID )      //*(int*)param = get the number which we know the value is interger
      switch (event) {
         case CV_EVENT_MOUSEMOVE:
            //std::cout << " Mouse Position = (" << x << ", " << y << " )";
            break;

         case CV_EVENT_LBUTTONDOWN:
        //    std::cout << " X Position = " << x << " Y Position = " << y << std::endl;
            if (counterL1 <= 3) {
               allConnerL[counterL1*2] = x;
               allConnerL[counterL1*2+1] = y;
            }

            if (counterL1 == 4 || counterL1 == 5) {
               WBCenterL[counterL2*2] = x;
               WBCenterL[counterL2*2+1] = y;
               counterL2++;
            }
            counterL1++;
         //   std::cout << "counterL1 : " << counterL1 << std::endl;
         //   std::cout << "counterL2 : " << counterL2 << std::endl;
            break;
      }

   if ( *(int*)param == RIGHT_IMAGE_ID )        //*(int*)param = get the number which we know the value is interger
      switch (event) {
         case CV_EVENT_MOUSEMOVE:
        //    std::cout << " Mouse Position = (" << x << ", " << y << " )";
            break;

         case CV_EVENT_LBUTTONDOWN:
        //    std::cout << " X Position = " << x << " Y Position = " << y << std::endl;
            if (counterR1 <= 3) {
               allConnerR[counterR1*2] = x;
               allConnerR[counterR1*2+1] = y;
            }

            if (counterR1 == 4 || counterR1 == 5) {
               WBCenterR[counterR2*2] = x;
               WBCenterR[counterR2*2+1] = y;
               counterR2++;
            }
            counterR1++;
          //  std::cout << "counter1R : " << counterR1 << std::endl;
          //  std::cout << "counter2R : " << counterR2 << std::endl;
            break;
      }

   if (counterL1 == 4) {
/*      std::cout << " Upper-LeftConner  X = " << allConnerL[0] << " Upper-LeftConner  Y = " << allConnerL[1] << std::endl;
      std::cout << " Upper-RightConner X = " << allConnerL[2] << " Upper-RightConner Y = " << allConnerL[3] << std::endl;
      std::cout << " Lower-LeftConner  X = " << allConnerL[4] << " Lower-LeftConner  Y = " << allConnerL[5] << std::endl;
      std::cout << " Lower-RightConner X = " << allConnerL[6] << " Lower-RightConner Y = " << allConnerL[7] << std::endl;

      std::cout << std::endl;
      std::cout << " Click at The White and Black Colors Center " << std::endl;
      */
   }

   if (counterL2 == 2) {
  /*    std::cout << " Upper-LeftConner  X = " << allConnerL[0] << " Upper-LeftConner  Y = " << allConnerL[1] << std::endl;
      std::cout << " Upper-RightConner X = " << allConnerL[2] << " Upper-RightConner Y = " << allConnerL[3] << std::endl;
      std::cout << " Lower-LeftConner  X = " << allConnerL[4] << " Lower-LeftConner  Y = " << allConnerL[5] << std::endl;
      std::cout << " Lower-RightConner X = " << allConnerL[6] << " Lower-RightConner Y = " << allConnerL[7] << std::endl;
      std::cout << " WhiteCenter X = " << WBCenterL[0] << " WhiteCenter Y = " << WBCenterL[1] << std::endl;
      std::cout << " BlackConner X = " << WBCenterL[2] << " BlackConner Y = " << WBCenterL[3] << std::endl;
      std::cout << " ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ " << std::endl;
      std::cout << " Press Any Key To Close The Image, Then into Calibration Process " << std::endl;
      std::cout << " ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ " << std::endl;
      */
   }

   ulxL = allConnerL[0] - 1, urxL = allConnerL[2] - 1, llxL = allConnerL[4] - 1, lrxL = allConnerL[6] - 1;
   ulyL = allConnerL[1] - 1, uryL = allConnerL[3] - 1, llyL = allConnerL[5] - 1, lryL = allConnerL[7] - 1;

   whiteRegionXL = WBCenterL[0] - 1, blackRegionXL = WBCenterL[2] - 1;
   whiteRegionYL = WBCenterL[1] - 1, blackRegionYL = WBCenterL[3] - 1;

   if (counterR1 == 4) {
 /*     std::cout << " Upper-LeftConner  X = " << allConnerR[0] << " Upper-LeftConner  Y = " << allConnerR[1] << std::endl;
      std::cout << " Upper-RightConner X = " << allConnerR[2] << " Upper-RightConner Y = " << allConnerR[3] << std::endl;
      std::cout << " Lower-LeftConner  X = " << allConnerR[4] << " Lower-LeftConner  Y = " << allConnerR[5] << std::endl;
      std::cout << " Lower-RightConner X = " << allConnerR[6] << " Lower-RightConner Y = " << allConnerR[7] << std::endl;

      std::cout << std::endl;
      std::cout << " Click at The White and Black Colors Center " << std::endl;
      */
   }

   if (counterR2 == 2) {
 /*     std::cout << " Upper-LeftConner  X = " << allConnerR[0] << " Upper-LeftConner  Y = " << allConnerR[1] << std::endl;
      std::cout << " Upper-RightConner X = " << allConnerR[2] << " Upper-RightConner Y = " << allConnerR[3] << std::endl;
      std::cout << " Lower-LeftConner  X = " << allConnerR[4] << " Lower-LeftConner  Y = " << allConnerR[5] << std::endl;
      std::cout << " Lower-RightConner X = " << allConnerR[6] << " Lower-RightConner Y = " << allConnerR[7] << std::endl;
      std::cout << " WhiteCenter X = " << WBCenterR[0] << " WhiteCenter Y = " << WBCenterR[1] << std::endl;
      std::cout << " BlackConner X = " << WBCenterR[2] << " BlackConner Y = " << WBCenterR[3] << std::endl;
      std::cout << " ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ " << std::endl;
      std::cout << " Press Any Key To Close The Image, Then into Calibration Process " << std::endl;
      std::cout << " ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ " << std::endl;
      */
   }
   ulxR = allConnerR[0] - 1, urxR = allConnerR[2] - 1, llxR = allConnerR[4] - 1, lrxR = allConnerR[6] - 1;
   ulyR = allConnerR[1] - 1, uryR = allConnerR[3] - 1, llyR = allConnerR[5] - 1, lryR = allConnerR[7] - 1;

   whiteRegionXR = WBCenterR[0] - 1, blackRegionXR = WBCenterR[2] - 1;
   whiteRegionYR = WBCenterR[1] - 1, blackRegionYR = WBCenterR[3] - 1;

   if (counterL2 > 1) {
      cvSetMouseCallback( "Left Image", NULL, NULL );   //Stop mouse Event
  //    printf("End mouse handling\n");
   }


   if (counterR2 > 1) {
      cvSetMouseCallback( "Right Image", NULL, NULL );  //Stop mouse Event
  //    printf("End mouse handling\n");
   }
}

void drawOnImage(cv::Mat &leftColorImage, cv::Mat &rightColorImage,  cv::Mat colorCentersL, cv::Mat colorCentersR){
    cv::Mat tempForVisualizationLeft, tempForVisualizationRight;
    tempForVisualizationLeft = leftColorImage.clone();
    tempForVisualizationRight = rightColorImage.clone();

    //Draw the click point
    cv::circle(tempForVisualizationLeft, cv::Point(ulxL, ulyL), 5, cv::Scalar(0, 255, 255), 2);
    cv::circle(tempForVisualizationLeft, cv::Point(urxL, uryL), 5, cv::Scalar(0, 255, 255), 2);
    cv::circle(tempForVisualizationLeft, cv::Point(llxL, llyL), 5, cv::Scalar(0, 255, 255), 2);
    cv::circle(tempForVisualizationLeft, cv::Point(lrxL, lryL), 5, cv::Scalar(0, 255, 255), 2);

    cv::circle(tempForVisualizationLeft, cv::Point(whiteRegionXL, whiteRegionYL), 5, cv::Scalar(0, 0, 255), 2);
    cv::circle(tempForVisualizationLeft, cv::Point(blackRegionXL, blackRegionYL), 5, cv::Scalar(0, 0, 255), 2);

    cv::circle(tempForVisualizationRight, cv::Point(ulxR, ulyR), 5, cv::Scalar(0, 255, 255), 2);
    cv::circle(tempForVisualizationRight, cv::Point(urxR, uryR), 5, cv::Scalar(0, 255, 255), 2);
    cv::circle(tempForVisualizationRight, cv::Point(llxR, llyR), 5, cv::Scalar(0, 255, 255), 2);
    cv::circle(tempForVisualizationRight, cv::Point(lrxR, lryR), 5, cv::Scalar(0, 255, 255), 2);

    cv::circle(tempForVisualizationRight, cv::Point(whiteRegionXR, whiteRegionYR), 5, cv::Scalar(0, 0, 255), 2);
    cv::circle(tempForVisualizationRight, cv::Point(blackRegionXR, blackRegionYR), 5, cv::Scalar(0, 0, 255), 2);

    //Draw the rectangle at the center in each color
    for (int i = 0; i < 24; i++) {
        double x = colorCentersL.at<double>(i);
        double y = colorCentersL.at<double>(i+24);

       // std::cout<<"x = "<<x<<",   y = "<<y<<std::endl;
        //cvCircle(tempForVcvShowImageisualizationLeft, cvPoint(x,y), 10, cvScalar(0,255,0),2);
        cv::rectangle(tempForVisualizationLeft, cv::Point(x - 7, y - 7), cv::Point(x + 7, y + 7), cv::Scalar(0, 127, 255), 1);
    }

    for (int i = 0; i < 24; i++) {
        double x = colorCentersR.at<double>(i);
        double y = colorCentersR.at<double>(i+24);

        //std::cout<<"x = "<<x<<",   y = "<<y<<std::endl;
        //cvCircle(tempForVisualizationRight, cvPoint(x,y), 10, cvScalar(0,255,0),2);
        cv::rectangle(tempForVisualizationRight, cv::Point(x - 7, y - 7), cv::Point(x + 7, y + 7), cv::Scalar(0, 127, 255), 1);
    }

    cv::setMouseCallback( "Left Image", &mouseEvent, (void*)&LEFT_IMAGE_ID);
    cv::setMouseCallback( "Right Image", &mouseEvent, (void*)&RIGHT_IMAGE_ID);
    cv::imshow( "Left Image", tempForVisualizationLeft );                   // Show our image inside it.
    cv::imshow( "Right Image", tempForVisualizationRight );                   // Show our image inside it.

}

void handleSignal(int) {
   std::cout << "Killed by signal!" << std::endl;
   exit(-1);
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    signal( SIGINT, handleSignal );
    signal( SIGABRT, handleSignal );
    signal( SIGSEGV, handleSignal );

    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Basic image color calibration using a MacBeth Color checker" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;

    if(argc != 3){
        std::cout << "-----------------------------------------------------------" << std::endl;
        std::cout << "\tUsages: ./DTIColorCalibration left_image right_image"      << std::endl;
        std::cout << "-----------------------------------------------------------" << std::endl;
        return 0;
    }

    cv::Mat leftColorImage, rightColorImage;
    try{
        leftColorImage = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
        rightColorImage = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);   // Read the file

    }catch(cv::Exception &e){

        std::cerr << e.what() << std::endl;
    }

    if(! leftColorImage.data || !rightColorImage.data)                              // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    bool running = true;

    std::cout << "Press 'q' or ESC to quit." << std::endl << std::endl;

    std::cout << " Click at The 4 Conners:: Start at TopLeft -> TopRight -> LowerLeft -> LowerRight " << std::endl;


    int lower_margin = 0;//10
    int higher_margin = 0; //10

    cv::Mat colorCentersL = cv::Mat(3, 24, CV_64F);
    cv::Mat colorCentersR = cv::Mat(3, 24, CV_64F);
    cv::Mat normalizedColorImageL, normalizedColorImageR;
    cv::Mat resizednormalizedColorImageL, resizednormalizedColorImageR;
    cv::Mat calibrationL, calibrationR;
    cv::Mat resizedcalibratedImageL, resizedcalibratedImageR;

    MacBethColorCalibrate mac;
    MacBethDetector md;


    while(running){
        //Get the centers of each color rectangle
         mac.getColorCenters(colorCentersL, ulxL, ulyL, urxL, uryL, llxL, llyL, lrxL, lryL);
         mac.getColorCenters(colorCentersR, ulxR, ulyR, urxR, uryR, llxR, llyR, lrxR, lryR);

        //Draw
        drawOnImage(leftColorImage, rightColorImage, colorCentersL, colorCentersR);

        //If point clicking finished....compute...
        if (counterL2 > 1 && counterR2 > 1) {
             //Get error befor normalizing image

           std::cout << "Errors before Left: \n" << std::endl;
            mac.getError(leftColorImage, 0, ulxL, ulyL, urxL, uryL, llxL, llyL, lrxL, lryL, 0, 0);
             std::cout << "Errors before Right: \n" << std::endl;
           mac.getError(rightColorImage, 0, ulxR, ulyR, urxR, uryR, llxR, llyR, lrxR, lryR, 0, 1);

            //Normalize image
            mac.colorNormalize(leftColorImage, normalizedColorImageL, cv::Point2i(whiteRegionXL, whiteRegionYL), cv::Point2i(blackRegionXL, blackRegionYL),lower_margin,higher_margin);
            mac.colorNormalize(rightColorImage, normalizedColorImageR,cv::Point2i(whiteRegionXR, whiteRegionYR), cv::Point2i(blackRegionXR, blackRegionYR),lower_margin,higher_margin);

            //Get error
            std::cout << "Left Errors after normalizing: \n" << std::endl;
            mac.getError(normalizedColorImageL, 0, ulxL, ulyL, urxL, uryL, llxL, llyL, lrxL, lryL, 1, 0);
            std::cout << "Right Errors after normalizing: \n" << std::endl;
            mac.getError(normalizedColorImageR, 0, ulxR, ulyR, urxR, uryR, llxR, llyR, lrxR, lryR, 1, 1);

         /*   cv::resize( normalizedColorImageL, resizednormalizedColorImageL, normalizedColorImageL.size());
            cv::namedWindow("Normalized Left Image", 1 );
            cv::imshow("Normalized Left Image", resizednormalizedColorImageL);

            cv::resize( normalizedColorImageR, resizednormalizedColorImageR, normalizedColorImageR.size());
            cv::namedWindow("Normalized Right Image", 1 );
            cv::imshow("Normalized Right Image", resizednormalizedColorImageR);
*/
            calibrationL = mac.computeColorCalibration(normalizedColorImageL, 0, ulxL, ulyL, urxL, uryL, llxL, llyL, lrxL, lryL,0);
            calibrationR = mac.computeColorCalibration(normalizedColorImageR, 0, ulxR, ulyR, urxR, uryR, llxR, llyR, lrxR, lryR,1);

            cv::Mat calibratedImageL(normalizedColorImageL.size(),normalizedColorImageL.type());
            cv::Mat calibratedImageR(normalizedColorImageR.size(), normalizedColorImageR.type());
            mac.colorCalibrate(normalizedColorImageL, calibratedImageL, calibrationL);
            mac.colorCalibrate(normalizedColorImageR, calibratedImageR, calibrationR);

            cv::resize(calibratedImageL, resizedcalibratedImageL, normalizedColorImageL.size() );
            cv::namedWindow("Calibrated Left Image", 1);
            cv::imshow("Calibrated Left Image", resizedcalibratedImageL);

            cv::resize(calibratedImageR, resizedcalibratedImageR, normalizedColorImageR.size() );
            cv::namedWindow("Calibrated Right Image", 1);
            cv::imshow("Calibrated Right Image", resizedcalibratedImageR);

            //Get error after calibration
             std::cout << "Left errors after calibration: \n" << std::endl;
            mac.getError(calibratedImageL, 0, ulxL, ulyL, urxL, uryL, llxL, llyL, lrxL, lryL, 2, 0);
             std::cout << "Right errors after calibration: \n" << std::endl;
            mac.getError(calibratedImageR, 0, ulxR, ulyR, urxR, uryR, llxR, llyR, lrxR, lryR, 2, 1);

            //Print error
     //       mac.printError();

            //Save calibration
            cv::imwrite("calibrated_left.png", calibratedImageL);
            cv::imwrite("calibrated_right.png", calibratedImageR);
            mac.save("colormatrixL.xml", 0);
            mac.save("colormatrixR.xml", 1);

            running = false;


         }




        int keyPress = cvWaitKey(5) & 255;
        switch ( keyPress ) {
            case 113: //q
            case 81: // Q
            case 27: // ESC
                running = false;
            break;
        default:
            break;
        }

    }

    return a.exec();
}
