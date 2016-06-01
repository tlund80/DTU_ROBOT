/*
 * StereoCalibration.cpp
 *
 *  Created on: Dec 6, 2011
 *      Author: RAHA
 */

#include "../../include/cal_ti/StereoCalibration.hpp"
#include <assert.h>
#include <exception>
#include <sstream>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace dti {

StereoCalibration::StereoCalibration(SharedData *sharedData) {
	setSharedData(sharedData);
	init();
}

StereoCalibration::~StereoCalibration() {

}

void StereoCalibration::init()
{
	// init vars
	_rectificationMapValid = false;
	updateCalibrationFlags();
	clearPoints();
	resetCalirationData();
}

void StereoCalibration::setIntrinsicAndDistortion(int CAM, cv::Mat intrinsic, cv::Mat distortion)
{
	switch (CAM) {
		case LEFT:
			intrinsic.copyTo(_camMatInit[LEFT]);
			distortion.copyTo(_distortionInit[LEFT]);
		break;
		case RIGHT:
			intrinsic.copyTo(_camMatInit[RIGHT]);
			distortion.copyTo(_distortionInit[RIGHT]);
		break;
		default:
            Q_EMIT consoleSignal("invalid CAM parameter in 'StereoCalibration::setIntrinsicAndDistortion(int CAM, cv::Mat intrinsic, cv::Mat distortion)'\n", dti::VerboseLevel::ERROR);
			return;
			break;
	}
}

void StereoCalibration::setIntrinsic(cv::Mat left, cv::Mat right)
{
	left.copyTo(_camMatInit[LEFT]);
	right.copyTo(_camMatInit[RIGHT]);
}

void StereoCalibration::setImageSizeStereo(cv::Size sizeLeft,cv::Size sizeRight)
{
	_imageSize  = sizeLeft; // Left
	_imageSizeR = sizeRight;// Right
}

void StereoCalibration::setDistortion(cv::Mat left, cv::Mat right)
{
	left.copyTo(_distortionInit[LEFT]);
	right.copyTo(_distortionInit[RIGHT]);
}

void StereoCalibration::clearPoints()
{
	// clear points
	_imagePoints[LEFT].clear();
	_imagePoints[RIGHT].clear();
	_objectPoints.clear();
	_goodImages.clear();
	_badImages.clear();
	_validPairs  = 0;
}
void StereoCalibration::resetCalirationData()
{
	std::cout << "resetting Calib data!!" << std::endl;
	_E.zeros(cv::Size(3,3),CV_64F);
	_F.zeros(cv::Size(3,3),CV_64F);
	_P1.zeros(cv::Size(4,3),CV_64F);
	_P2.zeros(cv::Size(4,3),CV_64F);
	_R1.zeros(cv::Size(3,3),CV_64F);
	_R2.zeros(cv::Size(3,3),CV_64F);
	_Q.zeros(cv::Size(4,4),CV_64F);
	_R.zeros(cv::Size(3,3),CV_64F);
	_T.zeros(cv::Size(1,3),CV_64F);
}
void StereoCalibration::resizePoints(int newSize )
{
	_imagePoints[LEFT].resize(newSize);
	_imagePoints[RIGHT].resize(newSize);
	_objectPoints.resize(newSize);
}

int StereoCalibration::addImagePointPairs(std::vector<cv::Point2f> cornersL, std::vector<cv::Point2f> cornersR, int imgId)
{

	_imagePoints[LEFT][_validPairs]  = cornersL;
	_imagePoints[RIGHT][_validPairs] = cornersR;

	switch(_sharedData->getBoardType())
	{
	case dti::CHECKERBOARD:
	case dti::SYMMETRIC_CIRCLES_GRID:
		for(int j = 0; j < _boardSize.height; j++ )
		{
			for( int k = 0; k < _boardSize.width; k++ )
			{
				_objectPoints[_validPairs].push_back(cv::Point3f(j * getSquareSize(), k * getSquareSize(), 0)); //todo needs to be tested.
			}
		}
		break;

	case dti::ASYMMETRIC_CIRCLES_GRID:
		for(int j = 0; j < _boardSize.height; j++ )
		{
			for( int k = 0; k < _boardSize.width; k++ )
			{
				_objectPoints[_validPairs].push_back(cv::Point3f((2*k + j % 2) * getSquareSize(), j * getSquareSize(), 0));
			}
		}
		break;
	}

	_goodImages.push_back(imgId);

	_validPairs++;
	return _validPairs;
}

//int calibrate(int imgOffsetIndex)
//{
//	using namespace cv;
//	int fileNum = _fileList[LEFT].size();
//	int validPairs = 0;
//	updateCalibrationFlags();
//
////	if(_fileList[LEFT].size() != _fileList[RIGHT].size()) {
////		Q_EMIT consoleSignal("The amount of left and right files does not match!\n --> They MUST be the same <--\n");
////		return -1;
////	}
//
//
//
//	// Detect the valid images in the set
//	for(int curImgID = 0 + imgOffsetIndex; curImgID< fileNum; curImgID++) {
//		if(shouldStop())
//			return cleanUpAndStop(-1);
//		std::vector<Point2f>& cornersL = _imagePoints[LEFT][curImgID];
//		std::vector<Point2f>& cornersR = _imagePoints[RIGHT][curImgID];
//
//		Q_EMIT consoleSignal("File to read: " + _fileList[LEFT][curImgID] + "REMEMBER that i've forced gray read\n");
//		Q_EMIT consoleSignal("File to read: " + _fileList[RIGHT][curImgID] + "REMEMBER that i've forced gray read\n");
//
//		_imgSrc[LEFT] 	=	imread(_fileList[LEFT][curImgID].toStdString(), 0);
//		_imgSrc[RIGHT] 	=	imread(_fileList[RIGHT][curImgID].toStdString(), 0);
//
//		bool foundL = true, foundR = true;
//		foundL = detectCorners(_imgSrc[LEFT], cornersL);
//		foundR = detectCorners(_imgSrc[RIGHT], cornersR);
//
//		// if not found ask for user input!
//		// by using eg. a Qt::BlockingQueuedConnection
//		cv::Mat roiL, roiR;
//		cv::Rect roiRect;
//		if(!foundL || !foundR)
//		{
//			if(!foundL){
//				manuelMarkerRoiSelection(_imgSrc[LEFT]);
//				while (!_sharedData->hasFreshAnnotation()) {
//
//
//					/*nop*/
//				}
//				roiRect = _sharedData->getROI();
//				if(roiRect.area() <= 0)
//					continue; // continue the for loop for all imgs
//				roiL = cv::Mat(_imgSrc[LEFT], roiRect);
//				foundL = detectCorners(roiL, cornersL);
////				if(foundL)
//			}
//
//			if(!foundR){
//				manuelMarkerRoiSelection(_imgSrc[RIGHT]);
//				while(!_sharedData->hasFreshAnnotation()) { /*nop*/ }
//				roiRect = _sharedData->getROI();
//				roiR = cv::Mat(_imgSrc[RIGHT], roiRect);
//				foundR = detectCorners(roiR, cornersR);
////				if(foundR)
//			}
//		}
//
//
//		if(foundL && foundR) {
//			_goodImages.push_back(curImgID);
//			validPairs++;
//			if(_cornerFlags._displayCorners)
//			{
//				_imgCorners[LEFT] 	= drawCorners(_imgSrc[LEFT], cornersL, foundL);
//				_imgCorners[RIGHT] 	= drawCorners(_imgSrc[RIGHT], cornersR, foundR);
//			}
//		} else
//		{
//			cvtColor(_imgSrc[LEFT], _imgCorners[LEFT], CV_GRAY2BGR);
//			cvtColor(_imgSrc[RIGHT], _imgCorners[RIGHT], CV_GRAY2BGR);
////			_imgCorners[LEFT] 	= _imgSrc[LEFT];
////			_imgCorners[RIGHT] 	= _imgSrc[RIGHT];
//			_badImages.push_back(curImgID);
//		}
//
//		if(_cornerFlags._displayCorners)
//			displayCorners();
//	}
//

//
//	//
//	Q_EMIT consoleSignal("BEFORE resize");
//    printValues(_imagePoints[LEFT], "LEFT");
//    printValues(_imagePoints[RIGHT], "RIGHT");
////    printValues(_objectPoints, "OBJS");
//
//    _imagePoints[LEFT].resize(validPairs);
//    _imagePoints[RIGHT].resize(validPairs);
//    _objectPoints.clear();
//    _objectPoints.resize(validPairs);
//
////    Q_EMIT consoleSignal("SquareSize: " + QString::number(getSquareSize()) + "\n");
////    Q_EMIT consoleSignal("BoardSize: h" + QString::number(_boardSize.height) + ", w: " + QString::number(_boardSize.width) + "\n");
////    Q_EMIT consoleSignal("_objectPoints:\n");
//    for(int i = 0; i < validPairs; i++ )
//    {
//        for(int j = 0; j < _boardSize.height; j++ )
//            for( int k = 0; k < _boardSize.width; k++ ){
//                _objectPoints[i].push_back(Point3f(j*getSquareSize(), k*getSquareSize(), 0));
////                emit consoleSignal("  " + QString::number(j*getSquareSize()) + ", " + QString::number( k*getSquareSize()) + ",0\n");
//            }
//    }
////    printValues(_imagePoints[LEFT], "LEFT");
////    printValues(_imagePoints[RIGHT], "RIGHT");
////    printValues(_objectPoints, "OBJS");
//}

double StereoCalibration::calibrate()
{
	using namespace cv;
    Q_EMIT consoleSignal("Running stereo calibration ...\n", dti::VerboseLevel::INFO);

    if(_validPairs < 2){
        Q_EMIT consoleSignal(" -> To few valid imagepairs available\n", dti::VerboseLevel::WARN);
		return -1;
	}

    resizePoints(_validPairs);

    // all vars should be class vars, hence be preloaded from elsewhere.
    Mat cameraMatrix[2];
    Mat distCoeffs[2];
    cameraMatrix[LEFT]  = Mat::eye(3, 3, CV_64F);
    cameraMatrix[RIGHT] = Mat::eye(3, 3, CV_64F);
    distCoeffs[LEFT]  = Mat::zeros(1,6,CV_64F);
    distCoeffs[RIGHT]  = Mat::zeros(1,6,CV_64F);
    if(_calFlags._useIntrinsicGuess)
    {
    	std::cout  << "Using intrinsic guess" << std::endl;
    	_sharedData->getIntrinsicGuessMatL(&cameraMatrix[LEFT]);
    	_sharedData->getIntrinsicGuessMatR(&cameraMatrix[RIGHT]);
    	_sharedData->getDistortionGuessMatL(&distCoeffs[LEFT]);
    	_sharedData->getDistortionGuessMatR(&distCoeffs[RIGHT]);
//    	_camMatInit[LEFT].copyTo(cameraMatrix[LEFT]);
//    	_camMatInit[RIGHT].copyTo(cameraMatrix[RIGHT]);
    	_distortionInit[LEFT].copyTo(distCoeffs[LEFT]);
    	_distortionInit[RIGHT].copyTo(distCoeffs[RIGHT]);
//    	imgSize = _imageSize;
    }

	Mat R, T, E, F;

	int calibrationFlags = calcStereoFlags();
	int termCriteriaFlags = calcStereoTerminationFlags();

#ifdef CV_VERSION < 3
     _rms = stereoCalibrate(
            _objectPoints,
            _imagePoints[LEFT], _imagePoints[RIGHT],
            cameraMatrix[LEFT], distCoeffs[LEFT],
            cameraMatrix[RIGHT], distCoeffs[RIGHT],
            _imageSize,
            R, T, E, F,
            TermCriteria(termCriteriaFlags, getStereoTermIterations(), getStereoTermEpsilon()),
            calibrationFlags
            );
#else
    _rms = stereoCalibrate(
            _objectPoints,
            _imagePoints[LEFT], _imagePoints[RIGHT],
            cameraMatrix[LEFT], distCoeffs[LEFT],
            cameraMatrix[RIGHT], distCoeffs[RIGHT],
            _imageSize,
            R, T, E, F,
            calibrationFlags,
            TermCriteria(termCriteriaFlags, getStereoTermIterations(), getStereoTermEpsilon())
            );


#endif
	cameraMatrix[LEFT].copyTo(_camMatRes[LEFT]);
	cameraMatrix[RIGHT].copyTo(_camMatRes[RIGHT]);

	R.copyTo(_R);
	T.copyTo(_T);
	E.copyTo(_E);
	F.copyTo(_F);

	if(_calFlags._useIntrinsicGuess){
		_distortionInit[LEFT].copyTo(_distRes[LEFT]);
		_distortionInit[RIGHT].copyTo(_distRes[RIGHT]);
	}else{
		distCoeffs[LEFT].copyTo(_distRes[LEFT]);
		distCoeffs[RIGHT].copyTo(_distRes[RIGHT]);
	}
    Q_EMIT consoleSignal("done with RMS error = " + QString::number(_rms) + "\n", dti::VerboseLevel::INFO);

	return _rms;
}

int StereoCalibration::rectifyImagePair(cv::Mat imgSrc[2], cv::Mat imgRes[2])
{
	using namespace cv;
	Mat rimg, cimg;


	if(_rectificationMapValid == false){
        Q_EMIT consoleSignal("Must initialize the rectification before rectifying!\n", dti::VerboseLevel::WARN);
		return -1;
	}

	// OpenCV can handle left-right
	// or up-down camera arrangements
	/** BUT THAT DOESN'T MATTER IN THIS CASE!! **/

//	bool isVerticalStereo = std::fabs(_P2.at<double>(1, 3)) > fabs(_P2.at<double>(0, 3));
//	double sf;
//	int w, h;
//	if( !isVerticalStereo )
//	{
//		sf = 600./MAX(_imageSize.width, _imageSize.height);
//		w = cvRound(_imageSize.width*sf);
//		h = cvRound(_imageSize.height*sf);
//	}
//	else
//	{
//		sf = 300./MAX(_imageSize.width, _imageSize.height);
//		w = cvRound(_imageSize.width*sf);
//		h = cvRound(_imageSize.height*sf);
//	}

	// resize images, compare imgSize with map size, if not equal -> resize by padding.
	cv::Mat imgResized = cv::Mat(_rectificationMap[LEFT][0].size(), imgSrc[LEFT].type(), cv::Scalar(0));
	cv::Mat imgSub;
	if(imgSrc[LEFT].size().width 	!= _rectificationMap[LEFT][0].size().width ||
		imgSrc[LEFT].size().height  != _rectificationMap[LEFT][0].size().height)
	{
		cv::Rect roi = cv::Rect(0,0,imgSrc[LEFT].size().width, imgSrc[LEFT].size().height);
		cv::Mat imgSub(imgResized, roi);
		imgSrc[LEFT].copyTo(imgSub);

		_validRoi[LEFT].width  = _validRoi[LEFT].width  *  (double) imgResized.size().width / imgSrc[LEFT].size().width;
		_validRoi[LEFT].height = _validRoi[LEFT].height *  (double) imgResized.size().height / imgSrc[LEFT].size().height;
	}
	else
		imgResized = imgSrc[LEFT];
	cv::remap(imgResized, _imgRectified[LEFT], _rectificationMap[LEFT][0], _rectificationMap[LEFT][1], CV_INTER_LINEAR);


	imgResized = cv::Mat(_rectificationMap[RIGHT][0].size(), imgSrc[RIGHT].type(), cv::Scalar(0));
	if(imgSrc[RIGHT].size().width 	!= _rectificationMap[RIGHT][0].size().width ||
		imgSrc[RIGHT].size().height  != _rectificationMap[RIGHT][0].size().height)
	{
		cv::Rect roi = cv::Rect(0,0,imgSrc[RIGHT].size().width, imgSrc[RIGHT].size().height);
		cv::Mat imgSub(imgResized, roi);

		imgSrc[RIGHT].copyTo(imgSub);

		_validRoi[RIGHT].width  = _validRoi[RIGHT].width    * (double) imgResized.size().width / imgSrc[RIGHT].size().width;
		_validRoi[RIGHT].height = _validRoi[RIGHT].height   * (double) imgResized.size().height/ imgSrc[RIGHT].size().height;

	}
	else
		imgResized = imgSrc[RIGHT];
	cv::remap(imgResized, _imgRectified[RIGHT], _rectificationMap[RIGHT][0], _rectificationMap[RIGHT][1], CV_INTER_LINEAR);



	Rect vroi;
	if( _rectFlags._rectAlg == dti::RE_Bougets && _rectFlags._displayValidRect)
	{
		rectangle(_imgRectified[LEFT], _validRoi[LEFT], Scalar(0,0,255), 3, 8);
		rectangle(_imgRectified[RIGHT], _validRoi[RIGHT], Scalar(0,0,255), 3, 8);
	}

	cv::cvtColor(_imgRectified[LEFT], imgRes[LEFT], CV_BGR2RGB);
	cv::cvtColor(_imgRectified[RIGHT], imgRes[RIGHT], CV_BGR2RGB);
//	_imgRectified[LEFT].copyTo(imgRes[LEFT]);
//	_imgRectified[RIGHT].copyTo(imgRes[RIGHT]);

	return 1;
}

void StereoCalibration::rectify()
{
    Q_EMIT consoleSignal("RECTIFY IMPLEMENT\n", dti::VerboseLevel::DEBUG);
	if(!_rectificationMapValid) {
		initRectification();
		printRectificationRes();
	}
//	doRectified();
}

bool StereoCalibration::isRectificationReady()
{
	return _rectificationMapValid;
}

void StereoCalibration::initRectification()
{
	int rectifyFlag = calcRectficationFlags();
	cv::Size newImgSize;

	newImgSize.height 	= std::max(_imageSize.height, _imageSizeR.height);
	newImgSize.width 	= std::max(_imageSize.width, _imageSizeR.width);

	try{

		cv::stereoRectify(_camMatRes[LEFT], _distRes[LEFT], _camMatRes[RIGHT],
						  _distRes[RIGHT], _imageSize, _R, _T,
						  _R1, _R2, _P1, _P2, _Q,
						  rectifyFlag,  _rectFlags._alpha,
						  newImgSize, &_validRoi[LEFT], &_validRoi[RIGHT]);

	}catch(cv::Exception &e){
		std::cout << "In initRectification() " << e.what() << std::endl;
	}

	// IF BY BOUGUET'S METHOD
	if (_rectFlags._rectAlg == dti::RE_Bougets)
	{
		std::cout << "BOUGUET'S METHOD" << std::endl;
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
	// use intrinsic parameters of each camera, but
	// compute the rectification transformation directly
	// from the fundamental matrix
	{
		std::cout << "HARTLEY'S METHOD" << std::endl;
		std::vector<cv::Point2f> allimgpt[2];
		for (int i = 0; i < _validPairs; i++) {
			std::copy(_imagePoints[LEFT][i].begin(), 	_imagePoints[LEFT][i].end(),  back_inserter(allimgpt[LEFT]));
			std::copy(_imagePoints[RIGHT][i].begin(), 	_imagePoints[RIGHT][i].end(), back_inserter(allimgpt[RIGHT]));
		}


		int fundamentalMatAlg = getRectFAlg();
		_F = findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), fundamentalMatAlg, _rectFlags._param1, _rectFlags._param2);
		cv::Mat H1, H2;
		stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), _F, _imageSize, H1, H2, 3);

		_R1 = _camMatRes[LEFT].inv() * H1 * _camMatRes[LEFT];
		_R2 = _camMatRes[RIGHT].inv() * H2 * _camMatRes[RIGHT];
		_P1 = _camMatRes[LEFT];
		_P2 = _camMatRes[RIGHT];
	}

    //Precompute maps for cv::remap()
	try {
		initUndistortRectifyMap(_camMatRes[LEFT], _distRes[LEFT], _R1, _P1,
				newImgSize, CV_16SC2, _rectificationMap[LEFT][0],
				_rectificationMap[LEFT][1]);
		initUndistortRectifyMap(_camMatRes[RIGHT], _distRes[RIGHT], _R2, _P2,
				newImgSize, CV_16SC2, _rectificationMap[RIGHT][0],
				_rectificationMap[RIGHT][1]);

		_rectificationMapValid = true;
	} catch (std::exception& ex) {
        Q_EMIT consoleSignal("initRectification failed!!\n" + QString(ex.what()),dti::VerboseLevel::ERROR);
		_rectificationMapValid = false;
	}catch (cv::Exception& ex) {
        Q_EMIT consoleSignal("initRectification failed!!\n" + QString(ex.what()),dti::VerboseLevel::ERROR);
		_rectificationMapValid = false;
	}

	std::cout << "end" << std::endl;

}

int StereoCalibration::getRectFAlg() {

	int res = 0;
	switch (_rectFlags._fundamentalAlg) {
	case dti::RE_FM_Points_7:
		res = cv::FM_7POINT;
	break;

	case dti::RE_FM_Points_8:
		res = cv::FM_8POINT;
	break;

	case dti::RE_FM_LeastMedian:
		res = cv::FM_LMEDS;
	break;

	case dti::RE_FM_RANSAC:
		res = cv::FM_RANSAC;
	break;

	default:
		break;
	}

	return res;
}

std::string StereoCalibration::rectificationAsString()
{
	std::stringstream ss;
	ss << " --> RECTIFICATION RESULTS <-- \n";

	ss << "R1 : Output 3x3 rectification transform (rotation matrix) for the first camera.\n";
	ss << _R1 << "\n\n";

	ss << "R2 : Output 3x3 rectification transform (rotation matrix) for the second camera.\n";
	ss << _R2 << "\n\n";

	ss << "P1 : Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera.\n";
	ss << _P1 << "\n\n";

	ss << "P2 : Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera.\n";
	ss << _P2 << "\n\n";

	ss << "Q : Output 4x4 disparity-to-depth mapping matrix ( see reprojectImageTo3D() ).\n";
	ss << _Q << "\n\n";

	ss << "Flags val: " << calcRectficationFlags() << "\n\n";
	ss << "Alpha value: " << _rectFlags._alpha << "\n\n";

	 return ss.str();
}

void StereoCalibration::printRectificationRes()
{
	std::string results = rectificationAsString();

    Q_EMIT consoleSignal(QString(results.c_str()),dti::VerboseLevel::INFO);
}

void StereoCalibration::printResults()
{
	printCalibResults();
	if(isRectificationReady())
		printRectificationRes();
}

void StereoCalibration::storeResults(std::string path)
{
	storeCalibResults(path);
	if(_rectificationMapValid)
	{
		//storeROSCalibResults();
		storeRectificationResults(path);
	}
}

void StereoCalibration::storeRectificationResults(std::string path)
{
	if (_rms < 0) {
        Q_EMIT consoleSignal("Calibration must be performed before storing the results!!", dti::VerboseLevel::WARN);
		return;
	}

	std::cout << "Storing result" << std::endl;
	std::string storeFileName = "StereoRectificationResults.yml";
	std::string storePath = path + "\\" + storeFileName;

	cv::FileStorage fs(storePath, CV_STORAGE_WRITE);

	if (!fs.isOpened())
		fs.open(storePath, CV_STORAGE_WRITE);
	if (fs.isOpened()) {
//		fs << " --> CALIBRATION RESULTS <-- \n";

		fs << "R1" 	<< _R1;
		fs << "R2"	<< _R2;

		fs << "P1"	<< _P1;
		fs << "P2" 	<< _P2;

		fs << "Q"	<< _Q;

		fs << "Flags" << calcRectficationFlags();

		fs << "Alpha" << _rectFlags._alpha;

		fs.release();
	} else
        Q_EMIT consoleSignal("Error: can not save the intrinsic parameters\n", dti::VerboseLevel::ERROR);
}

QString StereoCalibration::getFileName(int imgId, int CAM)
{
	return _fileList[CAM][imgId];
}

//int StereoCalibration::doRectified(cv::Mat imgRectifiedOut[2], int i)
//{
//
//}

void StereoCalibration::storeCalibResults(std::string path)
{
	if (_rms < 0) {
        Q_EMIT consoleSignal("Calibration must be performed before storing the results!!",dti::VerboseLevel::WARN);
		return;
	}

	std::string storeFileNameL 	= "StereoCalibrationResults_Left.yml";
	std::string storeFileNameR 	= "StereoCalibrationResults_Right.yml";
	std::string storeFileNameEx = "StereoCalibrationResults_Extrinsics.yml";
	std::string storePathL  	= path + storeFileNameL;
	std::string storePathR 		= path + storeFileNameR;
	std::string storePathEx 	= path + storeFileNameEx;

	cv::FileStorage fsL(storePathL, CV_STORAGE_WRITE);
	cv::FileStorage fsR(storePathR, CV_STORAGE_WRITE);
	cv::FileStorage fsEx(storePathEx, CV_STORAGE_WRITE);

	if (!fsL.isOpened())
		fsL.open(storePathL, CV_STORAGE_WRITE);
	if (fsL.isOpened()) {
//		fs << " --> CALIBRATION RESULTS <-- \n";

		fsL << "image_count" 			<< _validPairs;
		fsL << "image_size"				<< _imageSize;
		fsL << "square_size"			<< _squareSize;

		fsL << "pattern_size" 			<< _boardSize;
		fsL << CAMERA_MATRIX_NAME	 	<< _camMatRes[LEFT];
		fsL << DISTORTION_COEFF_NAME 	<< _distRes[LEFT];

		fsL.release();
	} else
        Q_EMIT consoleSignal("Error: can not save the intrinsic parameters\n", dti::VerboseLevel::ERROR);


	if (!fsR.isOpened())
		fsR.open(storePathR, CV_STORAGE_WRITE);
	if (fsR.isOpened()) {
		//fsR << " --> CALIBRATION RESULTS <-- \n";

		fsR << "image_count" 			<< _validPairs;
		fsR << "image_size"				<< _imageSize;
		fsR << "square_size"			<< _squareSize;

		fsR << "pattern_size" 			<< _boardSize;

		fsR << CAMERA_MATRIX_NAME 		<< _camMatRes[RIGHT];
		fsR << DISTORTION_COEFF_NAME  	<< _distRes[RIGHT];

		fsR.release();
	} else
        Q_EMIT consoleSignal("Error: can not save the intrinsic parameters\n",dti::VerboseLevel::ERROR);


	if (!fsEx.isOpened())
		fsEx.open(storePathR, CV_STORAGE_WRITE);
	if (fsEx.isOpened()) {
		//fsEx << " --> CALIBRATION RESULTS <-- \n";

		fsEx << "image_count" 			<< _validPairs;
		fsEx << "image_size"				<< _imageSize;
		fsEx << "square_size"			<< _squareSize;

		fsEx << "pattern_size" 			<< _boardSize;

		fsEx << "Rotation_matrix" 		<< _R;
		fsEx << "Translation_vector" 	<< _T;
		fsEx << "Essential_matrix" 		<< _E;
		fsEx << "Fundamental_matrix" 	<< _F;
		fsEx.release();
	} else
        Q_EMIT consoleSignal("Error: can not save the intrinsic parameters\n",dti::VerboseLevel::ERROR);
}

void StereoCalibration::storeROSCalibResults(std::string cameraNameL, std::string cameraNameR, int cameraCount)
{
	if (_rms < 0) {
        Q_EMIT consoleSignal("Calibration must be performed before storing the results!!\n",dti::VerboseLevel::WARN);
		return;
	}

	std::string homePath(getenv("HOME"));
	std::string storeFileNameL 	= cameraNameL + ".yaml";// "StereoCalibrationResults_Left.yml";
	std::string rosPathL = homePath + "/.ros/camera_info/" + storeFileNameL;

	std::string storeFileNameR 	= cameraNameR + ".yaml";//"StereoCalibrationResults_Right.yml";
	std::string rosPathR = homePath + "/.ros/camera_info/" + storeFileNameR;

	std::ofstream calibFileL;
	std::ofstream calibFileR;
	//Saving Right camera data
	if(cameraCount > 1){
		calibFileR.open(rosPathR.c_str());

		calibFileR << std::setprecision(15);
		calibFileR << "image_width: " << _imageSize.width << std::endl;
		calibFileR << "image_height: " << _imageSize.height << std::endl;
		calibFileR << "camera_name: " << cameraNameR << std::endl;
		calibFileR << CAMERA_MATRIX_NAME << ":" << std::endl;
		calibFileR << "  rows: " << _camMatRes[RIGHT].rows << std::endl;
		calibFileR << "  cols: " << _camMatRes[RIGHT].cols << std::endl;
		calibFileR << "  data: [";
			for(int iRow = 0; iRow < _camMatRes[RIGHT].rows; iRow++)
			{
				for(int iCol = 0; iCol < _camMatRes[RIGHT].cols; iCol++)
				{
					calibFileR << _camMatRes[RIGHT].at<double>(iRow,iCol);
					if(!(iRow == _camMatRes[RIGHT].rows-1 && iCol == _camMatRes[RIGHT].cols-1))
						calibFileR << ", ";
				}
			}
			calibFileR << "]" << std::endl;
			calibFileR << "distortion_model: plumb_bob" << std::endl; //todo other models?
			calibFileR << DISTORTION_COEFF_NAME << ":" << std::endl;
			calibFileR << "  rows: " << _distRes[RIGHT].rows << std::endl;
			calibFileR << "  cols: " << _distRes[RIGHT].cols << std::endl;
			calibFileR << "  data: [";
			for(int iRow = 0; iRow < _distRes[RIGHT].rows; iRow++)
			{
				for(int iCol = 0; iCol < _distRes[RIGHT].cols; iCol++)
				{
					calibFileR << _distRes[RIGHT].at<double>(iRow,iCol);
					if(!(iRow == _distRes[RIGHT].rows-1 && iCol == _distRes[RIGHT].cols-1))
						calibFileR << ", ";
				}
			}
			calibFileR << "]" << std::endl;

			calibFileR << "rectification_matrix" << ":" << std::endl;
			calibFileR << "  rows: " << _R1.rows << std::endl;
			calibFileR << "  cols: " << _R1.cols << std::endl;
			calibFileR << "  data: [";
			for(int iRow = 0; iRow < _R1.rows; iRow++)
			{
				for(int iCol = 0; iCol < _R1.cols; iCol++)
				{
					calibFileR << _R1.at<double>(iRow,iCol);
					if(!(iRow == _R1.rows-1 && iCol == _R1.cols-1))
						calibFileR << ", ";
				}
			}
			calibFileR << "]" << std::endl;

			calibFileR << "projection_matrix" << ":" << std::endl;
			calibFileR << "  rows: " << _P1.rows << std::endl;
			calibFileR << "  cols: " << _P1.cols << std::endl;
			calibFileR << "  data: [";
			for(int iRow = 0; iRow < _P1.rows; iRow++)
			{
				for(int iCol = 0; iCol < _P1.cols; iCol++)
				{
					calibFileR << _P1.at<double>(iRow,iCol);
					if(!(iRow == _P1.rows-1 && iCol == _P1.cols-1))
						calibFileR << ", ";
				}
			}
			calibFileR << "]" << std::endl;

			calibFileR.close();

            Q_EMIT consoleSignal("Calibration data committed to ROS. (Path: " + QString::fromStdString(rosPathR) + ")\n",dti::VerboseLevel::INFO);

	}

	//Saving Left camera / mono camera data
	calibFileL.open(rosPathL.c_str());

	calibFileL << std::setprecision(15);
	calibFileL << "image_width: " << _imageSize.width << std::endl;
	calibFileL << "image_height: " << _imageSize.height << std::endl;
	calibFileL << "camera_name: " << cameraNameL << std::endl;
	calibFileL << CAMERA_MATRIX_NAME << ":" << std::endl;
	calibFileL << "  rows: " << _camMatRes[LEFT].rows << std::endl;
	calibFileL << "  cols: " << _camMatRes[LEFT].cols << std::endl;
	calibFileL << "  data: [";
	for(int iRow = 0; iRow < _camMatRes[LEFT].rows; iRow++)
	{
		for(int iCol = 0; iCol < _camMatRes[LEFT].cols; iCol++)
		{
			calibFileL << _camMatRes[LEFT].at<double>(iRow,iCol);
			if(!(iRow == _camMatRes[LEFT].rows-1 && iCol == _camMatRes[LEFT].cols-1))
				calibFileL << ", ";
		}
	}
	calibFileL << "]" << std::endl;
	calibFileL << "distortion_model: plumb_bob" << std::endl; //todo other models?
	calibFileL << DISTORTION_COEFF_NAME << ":" << std::endl;
	calibFileL << "  rows: " << _distRes[LEFT].rows << std::endl;
	calibFileL << "  cols: " << _distRes[LEFT].cols << std::endl;
	calibFileL << "  data: [";
	for(int iRow = 0; iRow < _distRes[LEFT].rows; iRow++)
	{
		for(int iCol = 0; iCol < _distRes[LEFT].cols; iCol++)
		{
			calibFileL << _distRes[LEFT].at<double>(iRow,iCol);
			if(!(iRow == _distRes[LEFT].rows-1 && iCol == _distRes[LEFT].cols-1))
				calibFileL << ", ";
		}
	}
	calibFileL << "]" << std::endl;

	calibFileL << "rectification_matrix" << ":" << std::endl;
	calibFileL << "  rows: " << _R1.rows << std::endl;
	calibFileL << "  cols: " << _R1.cols << std::endl;
	calibFileL << "  data: [";
	for(int iRow = 0; iRow < _R1.rows; iRow++)
	{
		for(int iCol = 0; iCol < _R1.cols; iCol++)
		{
			calibFileL << _R1.at<double>(iRow,iCol);
			if(!(iRow == _R1.rows-1 && iCol == _R1.cols-1))
				calibFileL << ", ";
		}
	}
	calibFileL << "]" << std::endl;

	calibFileL << "projection_matrix" << ":" << std::endl;
	calibFileL << "  rows: " << _P1.rows << std::endl;
	calibFileL << "  cols: " << _P1.cols << std::endl;
	calibFileL << "  data: [";
	for(int iRow = 0; iRow < _P1.rows; iRow++)
	{
		for(int iCol = 0; iCol < _P1.cols; iCol++)
		{
			calibFileL << _P1.at<double>(iRow,iCol);
			if(!(iRow == _P1.rows-1 && iCol == _P1.cols-1))
				calibFileL << ", ";
		}
	}
	calibFileL << "]" << std::endl;

	calibFileL.close();

    Q_EMIT consoleSignal("Calibration data committed to ROS. (Path: " + QString::fromStdString(rosPathL) + ")\n",dti::VerboseLevel::INFO);
}

std::string StereoCalibration::resultsAsString()
{
	std::stringstream ss;
	ss << " --> (Stereo) CALIBRATION RESULTS <-- \n";

	ss << "image_count: " 			<< _validPairs 			<< "\n\n";
	ss << "image_size: [" 			<< _imageSize.height 	<< ", " << _imageSize.width 	<< "]\n\n";
	ss << "square_size: "			<< _squareSize			<< "\n\n";
	ss << "pattern_size: [" 		<< _boardSize.height 	<< ", " << _boardSize.width	<< "]\n\n";

	ss << "Camera matrix left:\n"  	<< _camMatRes[LEFT] 	<< "\n\n";
	ss << "Distortion left:\n"  	<< _distRes[LEFT]		<< "\n\n";

	ss << "Camera matrix right:\n" 	<< _camMatRes[RIGHT] 	<< "\n\n";
	ss << "Distortion right:\n"  	<< _distRes[RIGHT]	 	<< "\n\n";

	ss << "Rotation matrix:\n" 		<< _R 					<< "\n\n";
	ss << "Translation vector:\n" 	<< _T 					<< "\n\n";
	ss << "Essential Matrix:\n" 	<< _E 					<< "\n\n";
	ss << "Fundamental Matrix:\n" 	<< _F 					<< "\n\n";

	 return ss.str();
}

void StereoCalibration::printCalibResults()
 {
	if (_rms < 0) {
        Q_EMIT consoleSignal("Calibration must be performed before printing the results!!",dti::VerboseLevel::WARN);
		return;
	}
	std::string results = resultsAsString();

    Q_EMIT consoleSignal(QString(results.c_str()), dti::VerboseLevel::INFO);
}

int StereoCalibration::getStereoTermEpsilon()
{
	return _stereoFlags._stopOnEpsilon;
}

int StereoCalibration::getStereoTermIterations()
{
	return _stereoFlags._stopOnIterations;
}

int StereoCalibration::calcStereoTerminationFlags()
{
	int flags = 0;
	flags += (_stereoFlags._stopOnEpsilon > 0 ? CV_TERMCRIT_EPS : 0);
	flags += (_stereoFlags._stopOnIterations > 0 ? CV_TERMCRIT_ITER : 0);
	return flags;
}

int StereoCalibration::calcStereoFlags()
{
    Q_EMIT consoleSignal("Calc calib flags..\n",dti::VerboseLevel::DEBUG);
	int Calflags = calcCalibFlags();
    Q_EMIT consoleSignal("Calc stereo flags..\n", dti::VerboseLevel::DEBUG);
	int flags = _stereoFlags._fixIntrinsicStereo 	* cv::CALIB_FIX_INTRINSIC;
	flags += _stereoFlags._sameFocalLengthStereo 	* cv::CALIB_SAME_FOCAL_LENGTH;
    Q_EMIT consoleSignal("Stereo Calib flag sum (cal + stereo) : " + QString::number(Calflags) + " + " + QString::number(flags) + "\n", dti::VerboseLevel::DEBUG);
	return flags+Calflags;
}

int StereoCalibration::calcRectficationFlags()
{
	return (int) _rectFlags._zeroRectifyDisparity * cv::CALIB_ZERO_DISPARITY;
}

void StereoCalibration::setFilesLists(QStringList filesL, QStringList filesR)
{
	_fileList[LEFT].clear();
	_fileList[RIGHT].clear();

	if(_fileList[LEFT].size() != _fileList[RIGHT].size())
            Q_EMIT consoleSignal("The amount of left and right files does not match!\n --> They MUST be the same <--\n", dti::VerboseLevel::WARN);

	_fileList[LEFT].append(filesL);
	_fileList[RIGHT].append(filesR);
}

//bool StereoCalibration::shouldStop()
//{
//	QMutexLocker locker(&_mutexRunning);
//	return _sharedData->shouldStop();
//}

void StereoCalibration::updateCalibrationFlags(CornerDetectionFlags cornerFlags, CalibrationFlags calFlags, StereoCalibFlags stereoFlags, RectificationFlags rectFlags)
{
	_cornerFlags 	= cornerFlags;
	_calFlags 		= calFlags;
	_stereoFlags	= stereoFlags;
	_rectFlags		= rectFlags;
}

void StereoCalibration::updateCalibrationFlags()
{
	_cornerFlags 	= _sharedData->getCornerFlags();
	_calFlags 		= _sharedData->getCalFlags();
	_stereoFlags	= _sharedData->getStereoFlags();
	_rectFlags		= _sharedData->getRectFlags();
}

int StereoCalibration::cleanUpAndStop(int reason)
{
	// NOP
	return reason;
}

void StereoCalibration::calibrateNarrowStereo(cv::Mat& cameraMatrixL, cv::Mat& DistL,
											  cv::Mat& cameraMatrixR, cv::Mat& DistR,
											  cv::Mat& R, cv::Mat& T, bool print)
{
	_rectificationMapValid = false;
	updateCalibrationFlags();

	cameraMatrixL.copyTo(_camMatRes[LEFT]);
	cameraMatrixR.copyTo(_camMatRes[RIGHT]);
	DistL.copyTo(_distRes[LEFT]);
	DistR.copyTo(_distRes[RIGHT]);
	R.copyTo(_R);
	T.copyTo(_T);

	std::cout << "Computing Fundamental matrix using HARTLEY'S METHOD" << std::endl;
	std::vector<cv::Point2f> allimgpt[2];
	for (int i = 0; i < _validPairs; i++) {
		std::copy(_imagePoints[LEFT][i].begin(), 	_imagePoints[LEFT][i].end(),  back_inserter(allimgpt[LEFT]));
		std::copy(_imagePoints[RIGHT][i].begin(), 	_imagePoints[RIGHT][i].end(), back_inserter(allimgpt[RIGHT]));
	}

	int fundamentalMatAlg = getRectFAlg();
	_F = findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), fundamentalMatAlg, _rectFlags._param1, _rectFlags._param2);

	/* Compute essential matrix */
	cv::Mat E = cameraMatrixR.t() * _F * cameraMatrixL;
	E.copyTo(_E);

	//rectify();

	//Everything is computed manually
	_rms = 1;

	if(print)
		printCalibResults();

}

} /* namespace dti */
