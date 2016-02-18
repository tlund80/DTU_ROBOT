// Copyright (c) 2013, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "stereo_match.h"
//#include "../core/macros.h"

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
//#include "opencv2/contrib/contrib.hpp"

//PCL
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>

#include "SMTypes.h"

using namespace cv;

CloudPtr StereoMatch::compute(Mat &leftimg, Mat &rightimg,  Mat &img_colored){//(std::string leftImg, std::string rightImg) {

    CloudPtr out (new Cloud);

    int color_mode = _algorithm == STEREO_BM ? 0 : -1;
  //  std::cout << "Color_Mode " << color_mode << std::endl;
  //  Mat leftimg = imread(leftImg, color_mode);
    //COVIS_ASSERT(!leftimg.empty());
    assert(!leftimg.empty());
  //  Mat rightimg = imread(rightImg, color_mode);
    assert(!rightimg.empty());
    //COVIS_ASSERT(!rightimg.empty());
  //  Mat img_colored = imread(leftImg, -1);

    //check parameters
    if (_maxdisp % 16 != 0 || _maxdisp < 0) {
        pcl::console::print_error("Max disparity is < 0 or not divisible by 16!\n");
        return out;
    }
   /* if (_mindisp < 0) {
        pcl::console::print_error("Min disparity is < 0!\n");
        return out;
    }
    */
    if (_blocksize % 2 != 1 || _blocksize < 0) {
        pcl::console::print_error("Blocksize is not positive odd number!\n");
        return out;
    }

    Mat disp, disp8;
    Size img_size = leftimg.size();

    if(leftimg.channels() > 1)
        cv::cvtColor(leftimg, leftimg, CV_BGR2GRAY);

    if(rightimg.channels() > 1)
        cv::cvtColor(rightimg, rightimg, CV_BGR2GRAY);


    Rect roi1, roi2;

    //Calibration output
    Mat Q;
    Mat_<double> R, T, R1, P1, R2, P2;
    // reading intrinsic parameters
    //FileStorage fs(intrinsic, CV_STORAGE_READ);
    //std::cerr << "Problem with intrinsic file" << std::endl;//COVIS_ASSERT_MSG(fs.isOpened(), "Problem with intrinsic file");

    Mat_<double> M1, D1, M2, D2;
    M1 = (cv::Mat_<double>(3,3) << _calibParams.K0(0,0), _calibParams.K0(0,1), _calibParams.K0(0,2),
                                   _calibParams.K0(1,0), _calibParams.K0(1,1),  _calibParams.K0(1,2),
                                   _calibParams.K0(2,0), _calibParams.K0(2,1), _calibParams.K0(2,2));
    D1 = (cv::Mat_<double>(1,5) << _calibParams.k0(0), _calibParams.k0(1), _calibParams.k0(2), _calibParams.k0(3), _calibParams.k0(4),  _calibParams.k0(5));

    M2 = (cv::Mat_<double>(3,3) << _calibParams.K1(0,0), _calibParams.K1(0,1), _calibParams.K1(0,2),
                                   _calibParams.K1(1,0), _calibParams.K1(1,1),  _calibParams.K1(1,2),
                                   _calibParams.K1(2,0), _calibParams.K1(2,1), _calibParams.K1(2,2));
    D2 = (cv::Mat_<double>(1,5) << _calibParams.k1(0), _calibParams.k1(1), _calibParams.k1(2), _calibParams.k1(3), _calibParams.k1(4),  _calibParams.k1(5));

    R = (cv::Mat_<double>(3,3) << _calibParams.R1(0,0), _calibParams.R1(0,1), _calibParams.R1(0,2),
                                   _calibParams.R1(1,0), _calibParams.R1(1,1),  _calibParams.R1(1,2),
                                   _calibParams.R1(2,0), _calibParams.R1(2,1), _calibParams.R1(2,2));

    T = (cv::Mat_<double>(3,1) << _calibParams.T1(0), _calibParams.T1(1), _calibParams.T1(2));

    //fs["M1"] >> M1;
    //fs["D1"] >> D1;
    //fs["M2"] >> M2;
    //fs["D2"] >> D2;

  //  fs.open(extrinsic, CV_STORAGE_READ);
  //  std::cerr << "Problem with extrinsic file" << std::endl;
   // COVIS_ASSERT_MSG(fs.isOpened(), "Problem with extrinsic file");

    //fs["R"] >> R;
    //fs["T"] >> T;

  //  std::cout << "M1" << M1 << std::endl;
  //  std::cout << "D1" << D1 << std::endl;
  //std::cout << "M2" << M2 << std::endl;
  //    std::cout << "D2" << D2 << std::endl;

    if (!_rectified) {
        stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, 0, -1, img_size, &roi1, &roi2 );
        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r, img_cr;
        remap(leftimg, leftimg, map11, map12, INTER_CUBIC);
        remap(rightimg, rightimg, map21, map22, INTER_CUBIC);
        remap(img_colored, img_colored, map11, map12, INTER_CUBIC);

        if(_show_rectification){
            test_rectification(leftimg,rightimg);
        }
        cv::imwrite("/home/thso/left_rec.png", leftimg);
        cv::imwrite("/home/thso/right_rec.png", rightimg);

    }

    _maxdisp = _maxdisp > 0 ? _maxdisp : ((img_size.width/8) + 15) & -16;

    switch(_algorithm){
        case STEREO_BM: {
       // Ptr<StereoBM> sbm = StereoBM::create( ndisparities, SADWindowSize );
            cv::StereoBM bm;
            bm.state->roi1 = roi1;
            bm.state->roi2 = roi2;
            bm.state->preFilterCap = 31;
            bm.state->SADWindowSize = _blocksize > 0 ? _blocksize : 9;
            bm.state->minDisparity = _mindisp;
            bm.state->numberOfDisparities = _maxdisp;
            bm.state->textureThreshold = 10;//10
            bm.state->speckleWindowSize = 100; //100
            bm.state->speckleRange = 32;
            bm.state->disp12MaxDiff = -1;
            bm.state->uniquenessRatio = 15;

            if (_rectified) {
                bm.state->uniquenessRatio = 10;
                bm.state->disp12MaxDiff = -1;
            }
            bm(leftimg, rightimg, disp);
 cv::imwrite("/home/thso/disp.png",disp);
            disp.convertTo(disp8, CV_8U, 255 / (_maxdisp * 16.));
             cv::imwrite("/home/thso/disp8.png",disp8);
            break;
        }
        
        case STEREO_SGBM: {
            StereoSGBM sgbm;
            sgbm.preFilterCap = 31; //63
            sgbm.SADWindowSize = _blocksize > 0 ? _blocksize : 3;

            int cn = leftimg.channels();

            if (_rectified) {
                sgbm.P1 = 64*cn*sgbm.SADWindowSize*sgbm.SADWindowSize; //128
                sgbm.P2 = 128*cn*sgbm.SADWindowSize*sgbm.SADWindowSize; //256
                sgbm.disp12MaxDiff = -1;
                sgbm.fullDP = false;
            } else {
             //   sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
             //   sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
                sgbm.P1 = 64*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
                sgbm.P2 = 128*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
                sgbm.disp12MaxDiff = -1;
                sgbm.fullDP = false;//_algorithm == STEREO_HH;
            }
            sgbm.minDisparity = _mindisp;
            sgbm.numberOfDisparities = _maxdisp;
            sgbm.uniquenessRatio = 0;
            sgbm.speckleWindowSize = 0;
            sgbm.speckleRange = 0;


            sgbm(leftimg, rightimg, disp);
            disp.convertTo(disp8, CV_8U, 255 / (_maxdisp * 16.));
            cv::imwrite("/home/thso/disp.png",disp);
            break;
        }

        case STEREO_VAR: {
            StereoVar var;
            var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
            var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
            var.nIt = 25;
            var.minDisp = -_maxdisp;
            if (_rectified) {
                var.maxDisp = _mindisp;
            } else {
                var.maxDisp = 0;
            }
            var.poly_n = 3;
            var.poly_sigma = 0.0;
            var.fi = 15.0f;
            var.lambda = 0.03f;
            var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
            var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
            var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;

            var(leftimg, rightimg, disp);
            disp.convertTo(disp8, CV_8U);
            break;
        }
    }

    float f, B, cx, cy, cx2, dcx;
    int temp, maxdisp;
    // Get parameters for reconstruction
    if (_rectified){
        f = M1(0,0); // Focal length
        B = T(0,0); // Baseline in the x direction
        cx = M1(0,2); // Center x coordinate
        cy = M1(1,2); // Center y coordinate
        cx2 = M2(0,2); // Center x coordinate of right image
        dcx = cx-cx2; // Difference in center x coordinates
        temp = disp.at<int16_t>(0,0);
    } else {
        f = P1.at<double>(0,0); // Focal length
        B = P2.at<double>(0,3)/f; // Baseline in the x direction
        cx = P1.at<double>(0,2); // Center x coordinate
        cy = P1.at<double>(1,2); // Center y coordinate

        cx2 = P2.at<double>(0,2); // Center x coordinate of right image
        dcx = cx-cx2; // Difference in center x coordinates
        temp = disp.at<int16_t>(0,0);
        maxdisp = 0;
    }

    for(int y = 0; y < disp.rows; ++y) {
        for(int x = 0; x<disp.cols; ++x) {
            if(temp > disp.at<int16_t>(y,x))
                temp = disp.at<int16_t>(y,x);
            if(!_rectified){
                if(maxdisp < disp.at<int16_t>(y,x))
                    maxdisp = disp.at<int16_t>(y,x);
            }
        }
    }
    out->height = disp.cols;
    out->width = disp.rows;


    out->points.resize(out->height * out->width);
    Mat_<Vec3f> xyz(disp.rows, disp.cols, Vec3f(0,0,0)); // Resulting point cloud, initialized to zero
    for(int y = 0; y < disp.rows; ++y) {
        for(int x = 0; x < disp.cols; ++x) {
            pcl::PointXYZRGBA point;

            // Avoid invalid disparities
            if(disp.at<int16_t>(y,x) == temp) continue;
            if(disp.at<int16_t>(y,x) == 0) continue;
            if (!_rectified) {
                if(disp.at<int16_t>(y,x) == maxdisp) continue;
            }

            float d = float(disp.at<int16_t>(y,x)) / 16.0f; // Disparity
            if( _algorithm == STEREO_VAR ) d = float(disp.at<int16_t>(y,x))/16.0f/16.0f/16.0f;
            float W = B/(-d+dcx); // Weighting

            point.x = (float(x)-cx) * W;
            point.y = (float(y)-cy) * W;
            point.z = f * W;
            //skip 0 points
            if (point.x== 0 && point.y == 0 && point.z == 0) continue;
            // disregard points farther then 2m
            const double max_z = 2e3;
            if (fabs(point.y - max_z) < FLT_EPSILON || fabs(point.y) > max_z) continue;
            //scale position from mm to m
        //    point.x = 0.001*point.x;
        //    point.y = 0.001*point.y;
        //    point.z = 0.001*point.z;

           //add color
            Vec3b bgr = img_colored.at<Vec3b>(y,x);
            point.b = bgr[0];
            point.g = bgr[1];
            point.r = bgr[2];

            out->at(y, x) = point;
        }

    }
    //output
    if (_verbose) {
        pcl::console::print_warn("Executing Stereo match with following parameters: \n");
        std::string a = "bm";
        if (_algorithm == 1) a = "sgbm";
        else if (_algorithm == 2) a = "var";
        pcl::console::print_value("Algorithm: %s \n", a.c_str());
        pcl::console::print_value("Max disparity: %d\n", _maxdisp);
        pcl::console::print_value("Min disparity: %d\n", _mindisp);
        pcl::console::print_value("Block size: %d\n", _blocksize);
        pcl::console::print_warn("Results: \n");
        std::cout<<"R1 matrix:\n "<<R1<<std::endl<<std::endl;
        std::cout<<"R2 matrix:\n "<<R2<<std::endl<<std::endl;
        std::cout<<"P1 matrix:\n "<<P1<<std::endl<<std::endl;
        std::cout<<"P2 matrix:\n "<<P2<<std::endl<<std::endl;
        std::cout<<"P1 (0 3): "<<P1.at<double>(0,2)<<std::endl<<std::endl;
        std::cout<<"Q reproject matrix:\n "<<Q<<std::endl;
        std::cout<<"Q (0 3): "<<Q.at<double>(0,3)<<std::endl;
    }

    return out;
}


void StereoMatch::test_rectification(cv::Mat& left, cv::Mat& right)
{
     if(left.channels() == 1){
        cv::cvtColor(left, left, CV_GRAY2BGR );
        cv::cvtColor(right, right, CV_GRAY2BGR );
     }

     cv::Size sz1 = left.size();
     cv::Size sz2 = right.size();

     cv::Mat im3(sz1.height, sz1.width+sz2.width, left.type());

     cv::Mat left1(im3, cv::Rect(0, 0, sz1.width, sz1.height));
     left.copyTo(left1);
     cv::Mat right1(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
     right.copyTo(right1);

     //Add horizontal lines to the screen
     for( int j = 0; j < im3.rows; j += 50 ){
         cv::line(im3,cv::Point(0,j),cv::Point(im3.cols,j),cv::Scalar(0,200,0),1,CV_AA);
         }

    cv::namedWindow("Epipolar lines",CV_WINDOW_NORMAL|CV_WINDOW_FREERATIO);
    cv::imshow("Epipolar lines", im3);
    cv::resizeWindow("Epipolar lines", im3.cols/2, im3.rows/2);
    //cv::moveWindow("approximateZeroFilter",WPOS3X, WPOS3Y);



    while(true){
         int keyPressed = waitKey(0);
         if(keyPressed == 1048603){
             std::cout << "break" << std::endl;
             cv::destroyWindow("Epipolar lines");
             break;
         }
    }
}
