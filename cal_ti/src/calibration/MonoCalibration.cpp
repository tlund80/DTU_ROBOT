/*
 * MonoCalibration.cpp
 *
 *  Created on: Dec 6, 2011
 *      Author: RAHA
 */

#include "../../include/cal_ti/MonoCalibration.hpp"
#include <qstring.h>
#include <qstringlist.h>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace dti {

MonoCalibration::MonoCalibration(SharedData* sharedData)
{
	setSharedData(sharedData);
}

MonoCalibration::~MonoCalibration()
{

}

void MonoCalibration::init()
{
	_undistMapValid = false;
	updateCalibrationFlags();
	clearPoints();
}

double MonoCalibration::calibrate()
{
	using namespace cv;

	_rms = -1.0;
	//emit consoleSignal("Running mono calibration ...\n");
    Q_EMIT consoleSignal("Running mono calibration ...\n", dti::VerboseLevel::INFO);

	if(_validImgs < 2){
        Q_EMIT consoleSignal(" -> To few valid images available\n", dti::VerboseLevel::WARN);
		return -1;
	}

	resizePoints(_validImgs);
	// All vars should be class vars, hence be preloaded from elsewhere.
	Mat cameraMatrix;
	Mat distCoeffs;
	Size imgSize(0,0);
	if(_calFlags._useIntrinsicGuess)
	{
		_camMatInit.copyTo(cameraMatrix);
		_distortionInit.copyTo(distCoeffs);
		imgSize = _imageSize;
	} else
		cameraMatrix = Mat::eye(3, 3, CV_64F);

	int flags = calcCalibFlags();
	_rvecs.clear(); _tvecs.clear();
	_rms = cv::calibrateCamera(_objectPoints, _imagePoints, _imageSize, cameraMatrix, distCoeffs, _rvecs, _tvecs, flags);

	cameraMatrix.copyTo(_camMatRes);
	distCoeffs.copyTo(_distRes);

	return _rms;
}

QString MonoCalibration::getFileName(int imgId)
{
	return _fileList[imgId];
}

bool MonoCalibration::isUndistReady()
{
	return _undistMapValid;
}

void MonoCalibration::storeResults(std::string path)
{
	storeCalibResults(path);
	storeROSCalibResults();
}

void MonoCalibration::storeCalibResults(std::string path)
{
	if (_rms < 0) {
        Q_EMIT consoleSignal("Calibration must be performed before storing the results!!", dti::VerboseLevel::WARN);
		return;
	}
	std::string storeFileName = "MonoCalibrationResults.yml";
	std::string storePath = path + storeFileName;

	cv::FileStorage fs(storePath, CV_STORAGE_WRITE);

	if (!fs.isOpened())
		fs.open(storePath, CV_STORAGE_WRITE);
	if (fs.isOpened()) {

//		fs << " --> CALIBRATION RESULTS <-- \n";

		fs << "image_count" 		<< _validImgs;
		fs << "image_size"			<< _imageSize;
		fs << "square_size"			<< _squareSize;

		fs << "pattern_size" 		<< _boardSize;
		fs << CAMERA_MATRIX_NAME	<< _camMatRes;
		fs << DISTORTION_COEFF_NAME << _distRes;

		fs.release();
	} else
        Q_EMIT consoleSignal("Error: can not save the intrinsic parameters\n", dti::VerboseLevel::ERROR);
}

void MonoCalibration::storeROSCalibResults()
{
	if (_rms < 0) {
        Q_EMIT consoleSignal("Calibration must be performed before storing the results!!", dti::VerboseLevel::WARN);
		return;
	}
	std::string storeFileName = "MonoCalibrationResults.yaml";
	std::string homePath(getenv("HOME"));
	std::string rosPath = homePath + "/.ros/camera_info/" + storeFileName;

	std::ofstream outputFile;
	outputFile.open(rosPath.c_str());

	outputFile << std::setprecision(16);
	outputFile << "image_width: " << _imageSize.width << std::endl;
	outputFile << "image_height: " << _imageSize.height << std::endl;
	outputFile << "camera_name: " << "Name of camera" << std::endl;
	outputFile << CAMERA_MATRIX_NAME << ":" << std::endl;
	outputFile << "  rows: " << _camMatRes.rows << std::endl;
	outputFile << "  cols: " << _camMatRes.cols << std::endl;
	outputFile << "  data: [";
	for(int iRow = 0; iRow < _camMatRes.rows; iRow++)
	{
		for(int iCol = 0; iCol < _camMatRes.cols; iCol++)
		{
			outputFile << _camMatRes.at<double>(iRow,iCol);
			if(!(iRow == _camMatRes.rows-1 && iCol == _camMatRes.cols-1))
				outputFile << ", ";
		}
	}
	outputFile << "]" << std::endl;
	outputFile << "distortion_model: plumb_bob" << std::endl; //todo other models?
	outputFile << DISTORTION_COEFF_NAME << ":" << std::endl;
	outputFile << "  rows: " << _distRes.rows << std::endl;
	outputFile << "  cols: " << _distRes.cols << std::endl;
	outputFile << "  data: [";
	for(int iRow = 0; iRow < _distRes.rows; iRow++)
	{
		for(int iCol = 0; iCol < _distRes.cols; iCol++)
		{
			outputFile << _distRes.at<double>(iRow,iCol);
			if(!(iRow == _distRes.rows-1 && iCol == _distRes.cols-1))
				outputFile << ", ";
		}
	}
	outputFile << "]" << std::endl;

	outputFile.close();
}

void MonoCalibration::clearPoints()
{
	// clear points
	_imagePoints.clear();
	_objectPoints.clear();
	_goodImages.clear();
	_badImages.clear();
	_validImgs = 0;
}

void MonoCalibration::resizePoints(int newSize)
{
	_imagePoints.resize(newSize);
	_objectPoints.resize(newSize);
}

void MonoCalibration::printCalibResults()
{
	std::string str = calibResultsAsString();
    Q_EMIT consoleSignal(QString(str.c_str()), dti::VerboseLevel::INFO);
}

std::string MonoCalibration::calibResultsAsString()
{
	std::stringstream ss;
	ss << " --> (MONO) CALIBRATION RESULTS <-- \n";

	ss << "reprojection error: "    << _rms					<< "\n\n";
	ss << "image_count: " 			<< _validImgs			<< "\n\n";
	ss << "image_size: [" 			<< _imageSize.height 	<< ", " << _imageSize.width 	<< "]\n\n";
	ss << "square_size: "			<< _squareSize			<< "\n\n";
	ss << "Pattern size: [" 		<< _boardSize.height 	<< ", " << _boardSize.width	<< "]\n\n";

	ss << "CAMERA_MATRIX_NAME:\n"  	 << _camMatRes			<< "\n\n";
	ss << "DISTORTION_COEFF_NAME:\n" << _distRes			<< "\n\n";

	 return ss.str();
}

int MonoCalibration::undistortImage(const cv::Mat& imgSrc, cv::Mat& imgUndist)
{
	using namespace cv;
	Mat rimg, cimg;

	if(_undistMapValid == false){
        Q_EMIT consoleSignal("Must initialize the rectification before rectifying!\n", dti::VerboseLevel::WARN);
		return -1;
	}

	cv::Mat imgResized = cv::Mat(_undistMap[0].size(), imgSrc.type(), cv::Scalar(0));
	cv::Mat imgSub;
	if(imgSrc.size().width 	!= _undistMap[0].size().width ||
		imgSrc.size().height  != _undistMap[0].size().height)
	{
		cv::Rect roi = cv::Rect(0,0,imgSrc.size().width, imgSrc.size().height);
		cv::Mat imgSub(imgResized, roi);
		imgSrc.copyTo(imgSub);

		_validRoi.width  = _validRoi.width  *  (double) imgResized.size().width / imgSrc.size().width;
		_validRoi.height = _validRoi.height *  (double) imgResized.size().height / imgSrc.size().height;
	}
	else
		imgResized = imgSrc;

	cv::Mat _imgRectified;
	cv::remap(imgResized, imgUndist, _undistMap[0], _undistMap[1], CV_INTER_LINEAR);

	return 1;
}

void MonoCalibration::printResults()
{
	printCalibResults();
}

void MonoCalibration::setFilesLists(QStringList files)
{
	_fileList.clear();

	if(_fileList.size() != _fileList.size())
            Q_EMIT consoleSignal("The amount of left and right files does not match!\n --> They MUST be the same <--\n", dti::VerboseLevel::WARN);

	_fileList.append(files);
}

void MonoCalibration::updateCalibrationFlags(CornerDetectionFlags cornerFlags, CalibrationFlags calFlags, RectificationFlags undistFlags)
{
	_cornerFlags 	= cornerFlags;
	_calFlags 		= calFlags;
	_undistFlags	= undistFlags;
}

void MonoCalibration::updateCalibrationFlags()
{
	_cornerFlags 	= _sharedData->getCornerFlags();
	_calFlags 		= _sharedData->getCalFlags();
	_undistFlags	= _sharedData->getRectFlags();
}

void MonoCalibration::initUndist()
{
	_undistMapValid = false;
	if(_rms < 0)
        Q_EMIT consoleSignal("Cant undistort before the calibration has been completed successfully", dti::VerboseLevel::WARN);

	//Precompute maps for cv::remap()
	try {
		initUndistortRectifyMap(
				_camMatRes, _distRes, cv::Mat(),
				getOptimalNewCameraMatrix(_camMatRes, _distRes, _imageSize, _undistFlags._alpha, _imageSize, &_validRoi),
				_imageSize, CV_16SC2, _undistMap[0], _undistMap[1]);
		_undistMapValid = true;
	} catch (std::exception& ex) {
        Q_EMIT consoleSignal("initUndist failed!!\n" + QString(ex.what()), dti::VerboseLevel::ERROR);
		_undistMapValid = false;
	}
}

int MonoCalibration::addImagePoints(std::vector<cv::Point2f> corners, int imgId)
{

	_imagePoints[_validImgs]  = corners;

	switch(_sharedData->getBoardType())
	{
	case dti::CHECKERBOARD:
	case dti::SYMMETRIC_CIRCLES_GRID:
		for(int j = 0; j < _boardSize.height; j++ )
		{
			for( int k = 0; k < _boardSize.width; k++ )
			{
				_objectPoints[_validImgs].push_back(cv::Point3f(k * getSquareSize(), j * getSquareSize(), 0)); //todo needs to be tested.
			}
		}
		break;

	case dti::ASYMMETRIC_CIRCLES_GRID:
		for(int j = 0; j < _boardSize.height; j++ )
		{
			for( int k = 0; k < _boardSize.width; k++ )
			{
				_objectPoints[_validImgs].push_back(cv::Point3f((2*k + j % 2) * getSquareSize(), j * getSquareSize(), 0));
			}
		}
		break;
	}

	_goodImages.push_back(imgId);

	_validImgs++;
	return _validImgs;
}


} /* namespace dti */
