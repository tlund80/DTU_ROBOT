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

#ifndef COVIS_CALIB_STEREO_MATCH_H
#define COVIS_CALIB_STEREO_MATCH_H

//OpenCV
#include "opencv2/imgproc/imgproc.hpp"

//PCL
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "SMCalibrationParameters.h"

enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };


        /**
         * @class StereoMatch
         * @ingroup calib
         * @brief Computes point cloud out of stereo image pair
         *
         * @author
         */
        class StereoMatch {
            public:

                //Empty constructor
                StereoMatch() : _algorithm(STEREO_BM), _maxdisp(512), _mindisp(0), _blocksize(7),
                    _rectified (false), _verbose(false), _show_rectification(false){}


                /**
                 * Computes point cloud out of not rectified stereo image pair
                 * @param leftImg path to the left rgb image
                 * @param rightImg path to the right rgb image
                 * @param intrinsic path to the intrinsic camera parameter file
                 * @param extrinsic path to the extrinsic camera parameter file
                 */
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr compute(cv::Mat &leftimg, cv::Mat &rightimg,  cv::Mat &img_colored);


                /**
                 * Set used algorithm for stereo matching
                 * @param algorithm used algorithm for stereo matching
                 */
                inline void setAlgorithm(int algorithm) {
                    this->_algorithm = algorithm;
                }
                /**
                 * Set maximum disparity for stereo matching
                 * @param maxdisp max disparity, must be a positive integer divisible by 16
                 */
                inline void setMaxDisparity(int maxdisp) {
                    this->_maxdisp = maxdisp;
                }
                /**
                 * Set minimum disparity for stereo matching
                 * @param mindisp min disparity
                 */
                inline void setMinDisparity(int mindisp) {
                    this->_mindisp = mindisp;
                }
                /**
                 * Set blocksize
                 * @param blocksize the block size must be a positive odd number
                 */
                inline void setBlockSize(int blocksize) {
                    this->_blocksize = blocksize;
                }
                /**
                 * Set if images are in common image frame (rectification)
                 * @param rectified are images in common plane? true/false
                 */
                inline void setRectified(bool rectified) {
                    this->_rectified = rectified;
                }
                /**
                 * Set verbose
                 * @param verbose enable console output
                 */
                inline void setVerbose(bool verbose) {
                    this->_verbose = verbose;
                }

                /**
                 * Set calibration params
                 * @param verbose enable console output
                 */
                inline void setCalibrationParams(SMCalibrationParameters params) {
                    this->_calibParams = params;
                }

                inline bool show_rectification(bool show){
                    this->_show_rectification = show;
                }


            private:

                int _algorithm;
                int _maxdisp;
                int _mindisp;
                int _blocksize;
                bool _rectified;
                bool _verbose;
                bool _show_rectification;
                SMCalibrationParameters _calibParams;

                void test_rectification(cv::Mat& left, cv::Mat& right);

        };

        /**
         * @ingroup calib
         *
         * Computes point cloud out of not rectified stereo image pair
         *
         * @param leftImg path to the left rgb image
         * @param rightImg path to the right rgb image
         * @param intrinsic path to the intrinsic camera parameter file
         * @param extrinsic path to the extrinsic camera parameter file
         * @param algorithm used algorithm
         * @param maxdisp maximum disparity, must be a positive integer divisible by 16
         * @param mindisp minimum disparity
         * @param blocksize set the black size, the block size must be a positive odd number
         * @param rectified set to true if the input images are already rectified
         * @param verbose enable verbose output
         *
         * @return PointXYZRGBA point cloud
         */

  /*      inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr makeStereoMatch(std::string leftImg, std::string rightImg,
                std::string intrinsic, std::string extrinsic,
                int algorithm = STEREO_SGBM, int maxdisp = 512, int mindisp = 0, int blocksize = 7,
                bool rectified = false, bool verbose = false) {

            StereoMatch sm;
            sm.setAlgorithm(algorithm);
            sm.setMaxDisparity(maxdisp);
            sm.setMinDisparity(mindisp);
            sm.setBlockSize(blocksize);
            sm.setVerbose(verbose);
            sm.setRectified(rectified);

            return sm.compute(leftImg, rightImg, intrinsic, extrinsic);
        }
*/
#endif /* COVIS_CALIB_STEREO_MATCH_H */
