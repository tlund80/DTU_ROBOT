/*
 * Calibration.cpp
 *
 *  Created on: Dec 6, 2011
 *      Author: THSO/RAHA
 */

#include "../../include/cal_ti/Calibration.hpp"
#include <string.h>

namespace dti {

Calibration::Calibration() {
	_maxScale = 2;
	_rms = -1;
}

Calibration::~Calibration() {

}

int Calibration::checkForCorners(const cv::Mat& img)
{
	IplImage ipl_img = img;
	return cvCheckChessboard(&ipl_img,  _boardSize);
}

bool Calibration::detectCorners(const std::string& imgPath, std::vector<cv::Point2f>& corners)
{
	cv::Mat img = cv::imread(imgPath, 0);
	return detectCorners(img, corners);
}

void Calibration::addToBadImgs(int imgId)
{
	_badImages.push_back(imgId);
}

bool Calibration::readIntrinsic(const std::string& path, cv::Mat* intrinsic, cv::Mat *distortion, const std::string& intrinsicMatName, const std::string& distortionMatName)
{
	using namespace cv;
	bool res = false;
	if(!(path == ""))
	{
	  try{
		cv::FileStorage fs(path, FileStorage::READ);
		if(!fs.isOpened())
			fs.open(path, FileStorage::READ);
		if(fs.isOpened())
		{
			fs[intrinsicMatName.c_str()]  >> *intrinsic;
			fs[distortionMatName.c_str()] >> *distortion;
			res = true;
		}
	  }catch(cv::Exception &e)
	  {
	    std::cout << "Exception occured in Calibration::readIntrinsic()" << std::endl;
        Q_EMIT consoleSignal("Camera Calibration format Not Suported!!!!\n", dti::VerboseLevel::ERROR);
	    return false;
	  }
	}
	if(res == false)
	{
		*intrinsic = Mat::zeros(3,3,CV_64F);
		*distortion = Mat::zeros(1,8,CV_64F);
	}
	return res;
}


bool Calibration::detectCorners(const cv::Mat& img, std::vector<cv::Point2f>& corners)
{
	using namespace cv;
    bool found = false;
    int cornerFlags = calcCornerFlags();
    int terminationFlags = calcCornerTerminationFlags();
    int terminationIte = getCornerTerminationIte();
	double terminationEpsilon = getCornerTerminationEpsilon();

	CalBoardTypes boardType = _sharedData->getBoardType();

	try{
		for( int scale = 1; scale <= _maxScale; scale++ )
		{
			Mat timg;
			if( scale == 1 ) timg = img;
			else cv::resize(img, timg, cv::Size(), scale, scale);

			switch(boardType)
			{
        		case CHECKERBOARD:
        			found = findChessboardCorners(timg, _boardSize, corners, cornerFlags);
        			if(found)
        				cornerSubPix(timg, corners, Size(11,11),Size(-1,-1), TermCriteria( terminationFlags, terminationIte, terminationEpsilon));

        			break;
        		case SYMMETRIC_CIRCLES_GRID:
        			found = findCirclesGrid(timg, _boardSize, corners);

        			break;
        		case ASYMMETRIC_CIRCLES_GRID:
        			found = findCirclesGrid(timg, _boardSize, corners, CALIB_CB_ASYMMETRIC_GRID );
        			break;
			}

			if( found )
			{
				if( scale > 1 )
				{
					Mat cornersMat(corners);
					cornersMat *= 1./scale;
				}
				break;
			}
		}
	}catch(cv::Exception &h){
        Q_EMIT consoleSignal("OpenCV Exception in detectCorners()" + QString::fromStdString(h.what()) + "\n", dti::VerboseLevel::ERROR);

	}

    return found;
}

void Calibration::printValues(std::vector<std::vector<cv::Point2f> >& val, const QString& name)
{
    Q_EMIT consoleSignal("" + name + "\n", dti::VerboseLevel::INFO);
	//emit consoleSignal("" + name + "\n");
	std::vector<cv::Point2f>& ref = val[0];
	for (int i = 0; i < (int) val.size(); i++) {
		ref = val[i];
		for (int j = 0; j < (int) ref.size(); j++) {
            Q_EMIT consoleSignal("  " + QString::number(ref[j].x) + ", "  + QString::number(ref[j].y) +"\t", dti::VerboseLevel::INFO);
		}
        Q_EMIT consoleSignal("\n", dti::VerboseLevel::INFO);
	}
}

void Calibration::printValues(std::vector<std::vector<cv::Point3f> >& val, const QString& name)
{
    Q_EMIT consoleSignal("" + name + "\n", dti::VerboseLevel::INFO);
	std::vector<cv::Point3f>& ref = val[0];
	for (int i = 0; i < (int) val.size(); i++) {
		ref = val[i];
		for (int j = 0; j < (int) ref.size(); j++) {
            Q_EMIT consoleSignal("  " + QString::number(ref[j].x) + ", "  + QString::number(ref[j].y) + QString::number(ref[j].z) + "\t", dti::VerboseLevel::INFO);
		}
        Q_EMIT consoleSignal("\n", dti::VerboseLevel::INFO);
	}
}

int Calibration::calcCalibFlags()
{
	int flags = 0;
	flags += _calFlags._useIntrinsicGuess    	* cv::CALIB_USE_INTRINSIC_GUESS;
	flags += _calFlags._fixAspectRatio        	* cv::CALIB_FIX_ASPECT_RATIO;
	flags += _calFlags._fixPrincipalPoint     	* cv::CALIB_FIX_PRINCIPAL_POINT;
	flags += _calFlags._zeroTangentDistortion	* cv::CALIB_ZERO_TANGENT_DIST;
	flags += _calFlags._fixFocalLength        	* cv::CALIB_FIX_FOCAL_LENGTH;
	flags += _calFlags._fix_K1                	* cv::CALIB_FIX_K1;
	flags += _calFlags._fix_K2               	* cv::CALIB_FIX_K2;
	flags += _calFlags._fix_K3                	* cv::CALIB_FIX_K3;
	flags += _calFlags._fix_K4                	* cv::CALIB_FIX_K4;
	flags += _calFlags._fix_K5                	* cv::CALIB_FIX_K5;
	flags += _calFlags._fix_K6                	* cv::CALIB_FIX_K6;
	flags += _calFlags._useRationalModel      	* cv::CALIB_RATIONAL_MODEL;
    Q_EMIT consoleSignal("Calib flag sum: " + QString::number(flags) + "\n", dti::VerboseLevel::INFO);
	return flags;
}

int Calibration::calcCornerTerminationFlags()
{
	int flags = 0;
	flags += (_cornerFlags._stopOnEpsilon > 0 ? CV_TERMCRIT_EPS : 0);
	flags += (_cornerFlags._stopOnIterations > 0 ? CV_TERMCRIT_ITER : 0);
	return flags;
}

int Calibration::calcCornerFlags()
{
	// call update flags before..
	int flags = 0;
	flags += _cornerFlags._filterQuads			* cv::CALIB_CB_FILTER_QUADS;
	flags += _cornerFlags._doPreCornerCheck		* cv::CALIB_CB_FAST_CHECK;
	flags += _cornerFlags._normalizeImage		* cv::CALIB_CB_NORMALIZE_IMAGE;
	flags += _cornerFlags._useAdaptiveThres		* cv::CALIB_CB_ADAPTIVE_THRESH;
//	Q_EMIT consoleSignal("Corner flag sum: " + QString::number(flags) + "\n");
	return flags;

}

bool Calibration::doPreCornerCheck(){
	return _cornerFlags._doPreCornerCheck;
}

cv::Mat Calibration::drawCorners(const cv::Mat& img, const std::vector<cv::Point2f>& corners, bool completePatternFound)
{
	using namespace cv;
	Mat cimg, cimg1;
	cvtColor(img, cimg, CV_GRAY2BGR);
	drawChessboardCorners(cimg, getBoardSize(), (corners), completePatternFound);
	return cimg;
//	double sf = 640. / MAX(img.rows, img.cols);
//	resize(cimg, cimg1, Size(), sf, sf);
//	if (k == 0) {
//		setImgLCorner(&cimg1);
//	} else if (k == 1) {
//		setImgRCorner(&cimg1);
//	}
}


cv::Mat Calibration::draw3dAxis(cv::Mat &Image, const std::vector<cv::Point3f>& objectPoints, cv::Mat rvec, cv::Mat tvec,const cv::Mat cameraMatrix, const cv::Mat distortion)
{

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints( objectPoints, rvec,tvec, cameraMatrix,distortion, imagePoints);

    //draw lines of different colours
    cv::line(Image,imagePoints[0],imagePoints[1],cv::Scalar(0,0,255,255),1,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[2],cv::Scalar(0,255,0,255),1,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[3],cv::Scalar(255,0,0,255),1,CV_AA);
    putText(Image,"x", imagePoints[1],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255,255),2);
    putText(Image,"y", imagePoints[2],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0,255),2);
    putText(Image,"z", imagePoints[3],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0,255),2);

    return Image;
}

void Calibration::narrowStereoCalib()
{

}

} /* namespace dti */
