/*
 * stereoCalib.cpp
 *
 *  Created on: Dec 2, 2011
 *      Author: THSO/RAHA
 */


#include "../../include/cal_ti/CalibThread.hpp"
#include "../../include/cal_ti/Calibration.hpp"
#include "../../include/cal_ti/StereoCalibration.hpp"
#include "../../include/cal_ti/MonoCalibration.hpp"
#include "../../include/cal_ti/config.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string>
#include <iostream>
#include <cmath>

namespace dti {


CalibThread::CalibThread(SharedData *sharedData)
{
	QMutexLocker locker(&_mutexRunning);
	_running = false;
	locker.unlock();
	_sharedData		= sharedData;
	_pCalibStereo 	= new StereoCalibration(_sharedData);
	_pCalibMono 	= new MonoCalibration(_sharedData);
	_PCalibHandEye  = new HandEyeCalibration(_sharedData);
	init();
}

CalibThread::~CalibThread() {

}

void CalibThread::init()
{
	_rms = -1;
	setStereoState(SS_init);
	setMonoState(MS_init);
	setHandEyeState(HE_init);
	activate();
}

void CalibThread::initMono()
{
	using namespace cv;

	_fileList[LEFT]  = getFileList(LEFT);
	int fileNum 	= _fileList[LEFT].size();

	_rms = -1;
	_pCalibMono->init();
	// set data;
	_pCalibMono->resizePoints(fileNum);
	_pCalibMono->setFilesLists(_fileList[LEFT]);
	_pCalibMono->setBoardSize(_sharedData->getBoardSize());
	_pCalibMono->setSquareSize(_sharedData->getSquareSize());
}

void CalibThread::initHandEye()
{
	using namespace cv;

	_fileList[HAND]  = getFileList(HAND);
	int fileNum 	= _fileList[HAND].size();

	_PCalibHandEye->init();
	// set data;
	_PCalibHandEye->resizePoints(fileNum);
	_PCalibHandEye->setFilesLists(_fileList[HAND]);
	_PCalibHandEye->setBoardSize(_sharedData->getBoardSizeHandEye());
	_PCalibHandEye->setSquareSize(_sharedData->getSquareSizeHandEye());
}

void CalibThread::initStereo(){
	using namespace cv;
	_rms = -1;
	_fileList[LEFT]  = getFileList(LEFT);
	_fileList[RIGHT] = getFileList(RIGHT);

	if(_fileList[LEFT].size() != _fileList[RIGHT].size())
	{
        Q_EMIT consoleSignal("The amount of left and right files does not match!\n --> They MUST be the same <--\n", dti::VerboseLevel::WARN);
		return;
	}

	int fileNum 	= _fileList[LEFT].size();

    Q_EMIT consoleSignal("Number of image to process:  " +  QString::number(fileNum) + "\n", dti::VerboseLevel::INFO);

	_pCalibStereo->init();
	// set data;
	_pCalibStereo->resizePoints(fileNum);
	_pCalibStereo->setFilesLists(_fileList[LEFT], _fileList[RIGHT]);
	_pCalibStereo->setBoardSize(_sharedData->getBoardSize());
	_pCalibStereo->setSquareSize(_sharedData->getSquareSize());
}

double CalibThread::detectMonoCorners()
{
	using namespace cv;
	int validPairs = 0;
	_pCalib = _pCalibMono;
	// detect all corners and store results
	// Detect the valid images in the set
	int imgOffsetIndex = 0;
	std::vector<Point2f> corners;
	int fileNum = _fileList[LEFT].size();
	for(int curImgID = 0 + imgOffsetIndex; curImgID< fileNum; curImgID++) {
		if(shouldStop()){
			return cleanUpAndStop(-1);
		}
        Q_EMIT consoleSignal("Processing pair: " + QString::number(curImgID+1) + "\n",dti::VerboseLevel::INFO);
		_imgSrc[LEFT] 	=	imread(_fileList[LEFT][curImgID].toStdString(), 0);
		corners.clear();

		//	detect left corners and iff succeeded detect Right aswell
		bool found[3] = {false, false, false};
		processCorners(corners, LEFT, found, "Annotate Image #" + QString::number(curImgID));

		// only process the right if marker is found in the RIGHT
		if (found[LEFT])
            Q_EMIT consoleSignal(		"  --> Marker found in image\n", dti::VerboseLevel::INFO);

		if(_cornerFlags._displayCorners) {
			_imgCorners[LEFT]  = _pCalib->drawCorners(_imgSrc[LEFT],  corners, found[LEFT]);
			displayCorners();
		}

		if(found[LEFT])
		{
			validPairs =_pCalibMono->addImagePoints(corners, curImgID);
		} else
			_pCalibMono->addToBadImgs(curImgID);
	}
	_pCalib->setImageSize(_imgSrc[LEFT].size());

	return validPairs;
}

double CalibThread::detectHandEyeCorners()
{
	using namespace cv;
	int validPairs = 0;
	_pCalib = _PCalibHandEye;
	// detect all corners and store results
	// Detect the valid images in the set
	int imgOffsetIndex = 0;
	std::vector<Point2f> corners;
	int fileNum = _fileList[HAND].size();
	for(int curImgID = 0 + imgOffsetIndex; curImgID< fileNum; curImgID++) {
		if(shouldStop()){
			return cleanUpAndStop(-1);
		}
        Q_EMIT consoleSignal("Processing Image: " + QString::number(curImgID+1) + "\n", dti::VerboseLevel::INFO);
		_imgSrc[HAND] 	=	imread(_fileList[HAND][curImgID].toStdString(), 0);
		corners.clear();

		//	detect left corners and iff succeeded detect Right aswell
		bool found[3] = {false, false, false};

		if(_sharedData->getBoardTypeHandEye() == dti::AR_MARKER){
			_PCalibHandEye->detectARMarker(_imgSrc[HAND],HAND, found,_sharedData->getSquareSizeHandEye());

			//_imgSrc[HAND].copyTo(_imgCorners[HAND]);
		  //cv::imshow("in",_imgCorners[HAND]);
		  //cv::waitKey(0);//wait for key to be pressed

			//displayCorners();
			// only process the right if marker is found in the RIGHT
			if (found[HAND])
                Q_EMIT consoleSignal(		"  --> Marker found in image\n", dti::VerboseLevel::INFO);
		}
		else
		{
			processCorners(corners, HAND, found, "Annotate Image #" + QString::number(curImgID));

			// only process the right if marker is found in the RIGHT
			if (found[HAND])
                Q_EMIT consoleSignal(		"  --> Marker found in image\n", dti::VerboseLevel::INFO);

			if(_cornerFlagsHandEye._displayCorners) {
				_imgCorners[HAND]  = _pCalib->drawCorners(_imgSrc[HAND],  corners, found[HAND]);
				displayCorners();
			}

			if(found[HAND])
			{
				validPairs =_PCalibHandEye->addImagePoints(corners, curImgID);
			} else
				_PCalibHandEye->addToBadImgs(curImgID);
		}
	}
	_pCalib->setImageSize(_imgSrc[HAND].size());

	return validPairs;
}


double CalibThread::detectStereoCorners()
{
	using namespace cv;
	int validPairs;
	std::string str;
	// detect all corners and store results
	// Detect the valid images in the set
	int imgOffsetIndex = 0;
	std::vector<Point2f> cornersL;// = _imagePoints[LEFT][curImgID];
	std::vector<Point2f> cornersR;// = _imagePoints[RIGHT][curImgID];
	int fileNum = _fileList[LEFT].size();
	for(int curImgID = 0 + imgOffsetIndex; curImgID< fileNum; curImgID++) {
		if(shouldStop()){
			return cleanUpAndStop(-1);
		}
        Q_EMIT consoleSignal("Processing pair: " + QString::number(curImgID+1) + "\n", dti::VerboseLevel::INFO);
        Q_EMIT consoleSignal("Processing pair: " + QString::fromStdString(_fileList[LEFT][curImgID].toStdString())	 + "\n", dti::VerboseLevel::INFO);
        Q_EMIT consoleSignal("Processing pair: " +  QString::fromStdString(_fileList[RIGHT][curImgID].toStdString())  + "\n", dti::VerboseLevel::INFO);
		str = _fileList[LEFT][curImgID].toStdString();
		_imgSrc[LEFT] 	=	imread(str, 0);
		str = _fileList[RIGHT][curImgID].toStdString();
		_imgSrc[RIGHT] 	=	imread(str, 0); // TODO REMEMBER that i've forced gray read
		cornersL.clear();
		cornersR.clear();

		//	detect left corners and iff succeeded detect Right aswell
		bool found[3] = {false, false, false};
		processCorners(cornersL, LEFT, found, "Annotate Image #" + QString::number(curImgID) + " LEFT");

		// only process the right if marker is found in the RIGHT
		if (found[LEFT]) {
            Q_EMIT consoleSignal(		"  --> Marker found in LEFT image\n", dti::VerboseLevel::INFO);
			processCorners(cornersR, RIGHT, found,"Annotate Image #" + QString::number(curImgID) + " RIGHT");
        } else Q_EMIT consoleSignal(	"  --> Marker NOT found in LEFT image\n", dti::VerboseLevel::INFO);

        if(found[RIGHT]) 	Q_EMIT consoleSignal("  --> Marker found in RIGHT image\n", dti::VerboseLevel::INFO);
        else     			Q_EMIT consoleSignal("  --> Marker NOT found in RIGHT image\n", dti::VerboseLevel::INFO);

		if(_cornerFlags._displayCorners) {
			_imgCorners[LEFT]  = _pCalibStereo->drawCorners(_imgSrc[LEFT],  cornersL, found[LEFT]);
			_imgCorners[RIGHT] = _pCalibStereo->drawCorners(_imgSrc[RIGHT], cornersR, found[RIGHT]);
			displayCorners();
		}

		if(found[RIGHT] && found[LEFT])
		{
			validPairs =_pCalibStereo->addImagePointPairs(cornersL, cornersR, curImgID);
		} else
			_pCalibStereo->addToBadImgs(curImgID);
	}

	_pCalibStereo->setImageSizeStereo(_imgSrc[LEFT].size(), _imgSrc[RIGHT].size());
	return validPairs;
}

double CalibThread::calibrate(bool print)
{
	_rms =  _pCalib->calibrate();
    if(print)// && _rms < 0)
		_pCalib->printCalibResults();

	return _rms;
}

double CalibThread::undistort()
{
	if(_rms < 0 ) 	{
        Q_EMIT consoleSignal("Calibration must be performed before storing the results!!", dti::VerboseLevel::INFO);
		return -1;
	}

	int undistImgNum  = 0;
		// based on flags then rectify those from the calibration/corner detection step of those from the the supplied directory!
    Q_EMIT consoleSignal("_pCalibMono->initUndist()\n", dti::VerboseLevel::INFO);
	_pCalibMono->initUndist();
	undistImgNum = undistAllFromCalib();

	return undistImgNum;
}

int CalibThread::undistAllFromCalib()
{
	int rectifiedImgs = 0;
	if(!_pCalibMono->isUndistReady())
	{
        Q_EMIT consoleSignal("Rectification must be initialized either by calculating the rectification or by loading the parameters!\n",dti::VerboseLevel::WARN);
		return -1;
	}
	int totalImgNum = _pCalibMono->getGoodImgNum(), i;
	QString filename;
	for (i = 0; i<totalImgNum; i++)
	{
		if(shouldStop()){
			cleanUpAndStop(-1);
			return -1;
		}
		filename  = _pCalibMono ->getGoodImgFilename(i);
		_imgSrc[LEFT] = cv::imread(filename.toStdString(), 0);
		if(_pCalibMono->undistortImage(_imgSrc[LEFT], _imgRectified[LEFT] )){
			if(_rectFlags._displayRectification){
//				_imgSrc[LEFT].copyTo(_imgRectified[LEFT]);
				displayRectified();
				msleep(200);
			}
			if(_rectFlags._saveRectificationImg)
			{
				QString save;

				save = getRectSaveFileName(filename, true);
				cv::imwrite(save.toStdString(), _imgRectified[LEFT]);
			}
			rectifiedImgs++;
		}
		else
		{
            Q_EMIT consoleSignal("Error while rectifying " + filename[LEFT] + "\n", dti::VerboseLevel::ERROR);
            Q_EMIT consoleSignal("and                    " + filename[RIGHT] + "\n", dti::VerboseLevel::ERROR);
		}
	}
	return rectifiedImgs;
}

double CalibThread::rectification(bool print)
{
	int calRes = 1;
	if(calRes < 0)
        Q_EMIT consoleSignal("Cant rectify before the calibration has been completed", dti::VerboseLevel::WARN);
	else
	{
		// based on flags then rectify those from the calibration/corner detection step of those from the the supplied directory!
        Q_EMIT consoleSignal("_pCalibStereo->rectify(); IMPLEMENT!\n", dti::VerboseLevel::INFO);
		_pCalibStereo->initRectification();
		rectifyAllFromCalib();
		if(print)
			_pCalibStereo->printRectificationRes();
	}

	return 1;
}

void CalibThread::started(bool print)
{
	QMutexLocker locker(&_mutexRunning);
	_running = true;
	if(print)
        Q_EMIT consoleSignal(QString("Calibration started\n"), dti::VerboseLevel::INFO);
}

void CalibThread::finished(bool print)
{
	QMutexLocker locker(&_mutexRunning);
	_running = false;
	Q_EMIT finishedSignal();
	if(print)
        Q_EMIT consoleSignal(QString("Calibration finished\n"), dti::VerboseLevel::INFO);
}

void CalibThread::finishedPart()
{
	Q_EMIT finishedSignal();
}

void CalibThread::activate(bool print)
{
	_sharedData->setStop(false);
	start();
	if(print)
        Q_EMIT consoleSignal(QString("Calibration activated\n"), dti::VerboseLevel::INFO);
}

void CalibThread::deactivate(bool print)
{
	_sharedData->setStop(true);
	if(print)
        Q_EMIT consoleSignal(QString("Calibration deactivated\n"), dti::VerboseLevel::INFO);
}

void CalibThread::toggleThread()
{
	if(isRunning())
		deactivate();
	else
		activate();
}

bool CalibThread::isRunning()
{
	QMutexLocker locker(&_mutexRunning);
	return _running;
}

void CalibThread::run()
{
	started(true);
    Q_EMIT consoleSignal(QString("CalibrationThread is running!\n"), dti::VerboseLevel::INFO);

	// okay to set both here. If mono only left is used onwards.
	_imgPath[LEFT].setPath(_sharedData->getImgPathL());
	_imgPath[RIGHT].setPath(_sharedData->getImgPathR());
	_imgPath[HAND].setPath(_sharedData->getImgPathHandEyeMono());

	updateCalibrationFlags();

	switch (_sharedData->getCalType()) {
		case dti::StereoCalibration_offline:
		case dti::StereoCalibration_online:
            Q_EMIT consoleSignal(QString("\tStereoCalibration_offline\n"), dti::VerboseLevel::INFO);

            Q_EMIT consoleSignal(" " + _imgPath[LEFT].path()  + "\n", dti::VerboseLevel::INFO);
            Q_EMIT consoleSignal(" " + _imgPath[RIGHT].path() + "\n", dti::VerboseLevel::INFO);
//			initStereo();
			runStereoNew();
		break;

		case MonoCalibration_offline:
		case MonoCalibration_online:
            Q_EMIT consoleSignal(QString("MonoCalibration_offline\n"), dti::VerboseLevel::INFO);
//			initMono();
			runMono();
//			_pCalib = new MonoCalibration();
		break;

		case HandEyeCalibration_offline:
		case HandEyeCalibration_online:
            Q_EMIT consoleSignal(QString("HandEyeCalibration_offline\n"), dti::VerboseLevel::INFO);
            Q_EMIT consoleSignal(" " + _imgPath[HAND].path()  + "\n", dti::VerboseLevel::INFO);
			runHandEye();
		break;
		default:
		break;
	}


	static bool firstRun = true;

	finished(true);
    Q_EMIT consoleSignal(QString("CalibThread has ended!!\n"), dti::VerboseLevel::INFO);
}

//void CalibThread::setDoStop(bool stop)
//{
//	QMutexLocker locker(&_mutexDoStop);
//	_doStop = stop;
//}
//
//bool CalibThread::doStop()
//{
//	QMutexLocker locker(&_mutexDoStop);
//	return _doStop;
//}
//
//void CalibThread::runStereo()
//{
//	using namespace cv;
//	Q_EMIT consoleSignal("\n\n\n");
////	_imagePoints[LEFT].clear();
////	_imagePoints[RIGHT].clear();
//
//	_fileList[LEFT]  = getFileList(LEFT);
//	_fileList[RIGHT] = getFileList(RIGHT);
//
//	if(_fileList[LEFT].size() != _fileList[RIGHT].size())
//	{
//		Q_EMIT consoleSignal("The amount of left and right files does not match!\n --> They MUST be the same <--\n");
//		return;
//	}
//
//	int fileNum 	= _fileList[LEFT].size();
//	int validPairs 	= 0;
//
//	Q_EMIT consoleSignal("Number of image to process:  " +  QString::number(fileNum) + "\n");
////	_imagePoints[LEFT].resize(fileNum);
////	_imagePoints[RIGHT].resize(fileNum);
//
//	_pCalibStereo->init();
//	// set data;
//	_pCalibStereo->resizePoints(fileNum);
//	((StereoCalibration*)_pCalibStereo)->setFilesLists(_fileList[LEFT], _fileList[RIGHT]);
//	_pCalibStereo->setBoardSize(_sharedData->getBoardSize());
//	_pCalibStereo->setSquareSize(_sharedData->getSquareSize());
//
//	// detect all corners and store results
//	// Detect the valid images in the set
//	int imgOffsetIndex = 0;
//	std::vector<Point2f> cornersL;// = _imagePoints[LEFT][curImgID];
//	std::vector<Point2f> cornersR;// = _imagePoints[RIGHT][curImgID];
//
//	for(int curImgID = 0 + imgOffsetIndex; curImgID< fileNum; curImgID++) {
//		if(shouldStop()){
//			cleanUpAndStop(-1);
//			return;
//		}
//		Q_EMIT consoleSignal("Processing pair: " + QString::number(curImgID+1) + "\n");
//		_imgSrc[LEFT] 	=	imread(_fileList[LEFT][curImgID].toStdString(), 0);
//		_imgSrc[RIGHT] 	=	imread(_fileList[RIGHT][curImgID].toStdString(), 0); // TODO REMEMBER that i've forced gray read
//		cornersL.clear();
//		cornersR.clear();
//
//		//	detect left corners and iff succeeded detect Right aswell
//		bool found[2] = {false, false};
//		processCorners(cornersL, LEFT, found, "Annotate Image #" + QString::number(curImgID) + " LEFT");
//
//		// only process the right if marker is found in the RIGHT
//		if (found[LEFT]) {
//			Q_EMIT consoleSignal(		"  --> Marker found in LEFT image\n");
//			processCorners(cornersR, RIGHT, found,"Annotate Image #" + QString::number(curImgID) + " RIGHT");
//		} else Q_EMIT consoleSignal(	"  --> Marker NOT found in LEFT image\n");
//
//		if(found[RIGHT]) 	Q_EMIT consoleSignal("  --> Marker found in RIGHT image\n");
//		else     			Q_EMIT consoleSignal("  --> Marker NOT found in RIGHT image\n");
//
//		if(_cornerFlags._displayCorners) {
//			_imgCorners[LEFT]  = _pCalibStereo->drawCorners(_imgSrc[LEFT],  cornersL, found[LEFT]);
//			_imgCorners[RIGHT] = _pCalibStereo->drawCorners(_imgSrc[RIGHT], cornersR, found[RIGHT]);
//			displayCorners();
//		}
//
//		if(found[RIGHT] && found[LEFT])
//		{
//			validPairs =_pCalibStereo->addImagePointPairs(cornersL, cornersR, curImgID);
//		} else
//			_pCalibStereo->addToBadImgs(curImgID);
//	}
//
//	_pCalibStereo->setImageSize(_imgSrc[LEFT].size());
//
//	double calRes = _pCalibStereo->calibrate();
//
//	if(calRes > 0)
//	{
//		Q_EMIT consoleSignal("_pCalibStereo->rectify();\n");
//		_pCalibStereo->initRectification();
//		rectifyAllFromCalib();
//
////	for(int i = 0; i< validPairs; i++) {
////		_pCalibStereo->rectifyImagePair(_goodImages[i]);
////	}
//		_pCalibStereo->printResults();
//		_pCalibStereo->storeResults();
//	}
//
//}

QStringList CalibThread::getFileList(CameraID cam)
{
	QStringList nameFilter = _sharedData->getNameFilter(cam);
	_imgPath[cam].setNameFilters(nameFilter);

	QStringList fileList = _imgPath[cam].entryList();
	for(int i = 0; i<fileList.size(); i++)
		fileList[i].prepend(_imgPath[cam].absolutePath() + "/");

	return fileList;
}

void CalibThread::updateCalibrationFlags()
{
	_calFlags 		= _sharedData->getCalFlags();
	_cornerFlags 	= _sharedData->getCornerFlags();
	_stereoFlags	= _sharedData->getStereoFlags();
	_rectFlags		= _sharedData->getRectFlags();
	_cornerFlagsHandEye = _sharedData->getCornerFlagsHandEye();
}

bool CalibThread::shouldStop()
{
	QMutexLocker locker(&_mutexRunning);
	return _sharedData->shouldStop();
}

//void CalibThread::s

void CalibThread::runMono()
{
	bool doCont = true;
	MonoState state;
	QString savePath;

	while (doCont) {
		doCont = false;
		_pCalib = _pCalibMono;
		state = getMonoState();

		switch (state) {

		case MS_init:
			initMono();
			setMonoState(MS_final);
		break;

		case MS_detectCorners:
			initMono();
			updateCalibrationFlags();
			_pCalib->updateCalibrationFlags();
			detectMonoCorners();
			setMonoState(MS_waitForGUI);

		break;

		case MS_calibrate:
			updateCalibrationFlags();
			_pCalib->updateCalibrationFlags();
			calibrate(true);

			setMonoState(MS_waitForGUI);
//			finishedPart();
		break;

		case MS_undist:
			updateCalibrationFlags();
			_pCalibMono->updateCalibrationFlags();
			undistort();

			setMonoState(MS_waitForGUI);
//			finishedPart();
		break;

		case MS_saveResults:
			savePath = _sharedData->getSavePath();
			_pCalib->storeResults(savePath.toStdString());
			setMonoState(MS_final);
		break;

		case MS_doAll:
//			init();
			initMono();
			updateCalibrationFlags();
			_pCalib->updateCalibrationFlags();
			detectMonoCorners();
			calibrate(true);
			undistort();
			setMonoState(MS_waitForGUI);
			finishedPart();
		break;

		case MS_final:
			doCont = false;
			finished();

		break;

		case MS_clearData:
            _pCalibMono->clearPoints();
			setMonoState(dti::MS_waitForGUI);

		break;


		case MS_waitForGUI:
			//NOP

		break;

		}
//		usleep(100);
	}
	finished();
}

void CalibThread::runStereoNew()
{
	bool doCont = true;
	StereoState state;
	QString savePath;

	while(doCont) {
		doCont = false;
		_pCalib = _pCalibStereo;
		state = getStereoState();

		switch (state) {
			case SS_init:
				initStereo();
				setStereoState(SS_waitForGUI);
//				finishedPart();
			break;

			case SS_detectCorners:
				initStereo();
				updateCalibrationFlags();
				_pCalibStereo->updateCalibrationFlags();
				detectStereoCorners();
				setStereoState(SS_waitForGUI);
//				finishedPart();
			break;

			case SS_calibrate:
				updateCalibrationFlags();

				_pCalibStereo->updateCalibrationFlags();
				calibrate(true);
//				_pCalibStereo->calibrate();
//				_pCalibStereo->printCalibResults();

				setStereoState(SS_waitForGUI);
//				finishedPart();
			break;

			case SS_rectify:
				updateCalibrationFlags();
				_pCalibStereo->updateCalibrationFlags();
				rectification(true);

				setStereoState(dti::SS_waitForGUI);
//				finishedPart();
			break;

			case SS_doAll:
				initStereo();
				updateCalibrationFlags();
				_pCalibStereo->updateCalibrationFlags();
				detectStereoCorners();
				calibrate(true);
				rectification(true);
				setStereoState(dti::SS_final);
//				finishedPart();
			break;

			case SS_saveResults:
				savePath = _sharedData->getSavePath();
				_pCalib->storeResults(savePath.toStdString());
				setStereoState(dti::SS_final);
//				finishedPart();
			break;

			case SS_final:
				doCont = false;
				setStereoState(dti::SS_waitForGUI);

			break;
			case SS_clearData:
                _pCalibStereo->clearPoints();
				setStereoState(dti::SS_waitForGUI);

			break;

			case SS_waitForGUI:
				//NOP;
			break;
			default:
				//NOP
				break;
		}
//		usleep(100);
	}

	finished();
}

void CalibThread::runHandEye()
{
	bool doCont = true;
	HandEyeState state;
	QString savePath;

	while(doCont) {
		doCont = false;
		_pCalib = _PCalibHandEye;
		state = getHandEyeState();

		switch (state) {
			case HE_init:
				//initStereo();
				//setStereoState(SS_waitForGUI);
			break;

			case HE_detectCorners:
				initHandEye();
				updateCalibrationFlags();
				_pCalib->updateCalibrationFlags();
				detectHandEyeCorners();
				setHandEyeState(HE_waitForGUI);

			break;

			case HE_computePose:
				_PCalibHandEye->computeCameraPose();
				setHandEyeState(HE_waitForGUI);

			break;

			case HE_calibrate:
				updateCalibrationFlags();

				_PCalibHandEye->updateCalibrationFlags();

				if(_sharedData->getCalTypeHandEye() == dti::HandEyeCalibration_offline){
					//Loading robot poses from file
                    if(_sharedData->getRobotPosePath().contains(".xml"))
                        _PCalibHandEye->loadRobotPoses_xml();
                    if(_sharedData->getRobotPosePath().contains(".txt"))
                        _PCalibHandEye->loadRobotPoses_txt();
                    //_PCalibHandEye->loadRobotPoses();
				}else{
					//Set Robotdata from "live robot poses" store in shared data
					_PCalibHandEye->setRobotPoses(_sharedData->getRobotPoseArray());
				}

				calibrate(true);

				setHandEyeState(HE_waitForGUI);

			break;

			case HE_doAll:
				initHandEye();
				updateCalibrationFlags();
				_pCalib->updateCalibrationFlags();
				detectHandEyeCorners();
				_PCalibHandEye->computeCameraPose();
				_PCalibHandEye->calibrate();
				setHandEyeState(HE_final);

			break;

			case HE_saveResults:
				savePath = _sharedData->getHandEyeSavePath();
				_PCalibHandEye->storeResults(savePath.toStdString());
				setHandEyeState(HE_final);
			break;

			case HE_final:
				doCont = false;
				setHandEyeState(HE_waitForGUI);

			break;

			case HE_clearData:
                _PCalibHandEye->clearAllData();
				setHandEyeState(HE_waitForGUI);

			break;
			case HE_waitForGUI:
				//NOP;
			break;
			default:
				//NOP
				break;
		}
	}

	finished();
}

void CalibThread::doAll()
{
	setStereoState(	SS_doAll);
	setMonoState(	MS_doAll);
	setHandEyeState(HE_doAll);
}

void CalibThread::doCalibrate() {
	setStereoState(	SS_calibrate);
	setMonoState(MS_calibrate);
	setHandEyeState(HE_calibrate);
}

void CalibThread::doDetectCorners() {
	setStereoState(	SS_detectCorners);
	setMonoState(	MS_detectCorners);
	setHandEyeState(HE_detectCorners);
}

void CalibThread::doRectify() {
	setStereoState(	SS_rectify);
	setMonoState(	MS_undist);
}

void CalibThread::doSaveResults() {
	setStereoState(	SS_saveResults);
	setMonoState(	MS_saveResults);
	setHandEyeState(HE_saveResults);
}

void CalibThread::doStop() {
	setStereoState(	SS_final);
	setMonoState(	MS_final);
	setHandEyeState(HE_final);
	_sharedData->setStop(true);
}

void CalibThread::doComputePose() {

	setHandEyeState(HE_computePose);

}

void CalibThread::doClearAllData() {

	setStereoState(	SS_clearData);
	setMonoState(	MS_clearData);
	setHandEyeState(HE_clearData);

}

void CalibThread::dosaveRobotPoses()
{

	_PCalibHandEye->setRobotPoses(_sharedData->getRobotPoseArray());
	_PCalibHandEye->saveRobotPoses(_sharedData->getRobotPoseLivePath().toStdString());

}

void CalibThread::doLoadRobotPoses(){
    _PCalibHandEye->clearRobotPoses();
    if(_sharedData->getRobotPosePath().contains(".xml"))
        _PCalibHandEye->loadRobotPoses_xml();
    if(_sharedData->getRobotPosePath().contains(".txt"))
       _PCalibHandEye->loadRobotPoses_txt();

    //_PCalibHandEye->loadRobotPoses();

}

int CalibThread::cleanUpAndStop(int reason)
{

	// NOP
	return reason;
}

void CalibThread::readIntrinsicGuessPath(CameraID CAM, QString intrinsicMatName, QString distortionMatName)
{
    Q_EMIT consoleSignal("readIntrinsicGuessPath\n", dti::VerboseLevel::INFO);
	QString path = "";
	bool res = false;
	cv::Mat intrinsic, distortion;

	switch (CAM) {
		case LEFT:
			path = _sharedData->getIntrinsicGuessPathL();
			break;
		case RIGHT:
			path = _sharedData->getIntrinsicGuessPathR();
			break;
		default:
            Q_EMIT consoleSignal("Invalid cam param in CalibThread::readIntrinsicGuessPath(CameraID CAM)\n", dti::VerboseLevel::INFO);
			return;
			break;
	}

	res = _pCalib->readIntrinsic(path.toStdString(),
								&intrinsic, 	&distortion,
								intrinsicMatName.toStdString(),
								distortionMatName.toStdString() );

	if(!res){
        Q_EMIT consoleSignal("Error while reading intrinsic from " + path +" - all values have been set to zero\n", dti::VerboseLevel::ERROR);
//		return;
	}

	_pCalibStereo->setIntrinsicAndDistortion(CAM, intrinsic, distortion);
	_sharedData->setIntrinsicGuessVals(CAM, intrinsic, distortion);

	Q_EMIT updateGuiIntrinsicValsSignal();
}

int CalibThread::rectifyAllFromCalib()
{
	int rectifiedImgs = 0;
	if(!_pCalibStereo->isRectificationReady())
	{
        Q_EMIT consoleSignal("Rectification must be initialized either by calculating the rectification or by loading the parameters!\n", dti::VerboseLevel::WARN);
		return -1;
	}
	int totalImgNum = _pCalibStereo->getGoodImgNum(), i;
	QString filename[2];

	for (i = 0; i<totalImgNum; i++)
	{
		if(shouldStop()){
			cleanUpAndStop(-1);
			return -1;
		}
		filename[LEFT]  = _pCalibStereo ->getGoodImgFilename(i, LEFT);
		filename[RIGHT] = _pCalibStereo->getGoodImgFilename(i, RIGHT);
		_imgSrc[LEFT] = cv::imread(filename[LEFT].toStdString(), 1);
		_imgSrc[RIGHT] = cv::imread(filename[RIGHT].toStdString(), 1);
		if(_pCalibStereo->rectifyImagePair(_imgSrc, _imgRectified )){
			if(_rectFlags._displayRectification){
				displayRectified();
				msleep(200);
			}
			if(_rectFlags._saveRectificationImg)
			{
				QString saveL, saveR;
				saveL = getRectSaveFileName(filename[LEFT], true);
				saveR = getRectSaveFileName(filename[RIGHT], false);

				cv::imwrite(saveL.toStdString(), _imgRectified[LEFT]);
				cv::imwrite(saveR.toStdString(), _imgRectified[RIGHT]);
			}
			rectifiedImgs++;
		}
		else
		{
            Q_EMIT consoleSignal("Error while rectifying " + filename[LEFT] + "\n", dti::VerboseLevel::ERROR);
            Q_EMIT consoleSignal("and                    " + filename[RIGHT] + "\n", dti::VerboseLevel::ERROR);
		}
	}
	return rectifiedImgs;
}

// ref is the LEFT img
QString CalibThread::getRectSaveFileName(QString orgName, bool isRef)
{
	int i;
	QString newName, tmp, savePath, postFix, preFix = "rect_";
	QStringList sList = orgName.split("/");

	// add the path
	if(isRef){
		newName = _sharedData->getRectSavePath(dti::LEFT) + "/";
//		postFix = "_R"; // REF!!!
	}
	else {
		newName = _sharedData->getRectSavePath(dti::RIGHT) + "/";
//		postFix = "_L"; // TODO: not shure why MLNN want's it to be L!
	}
	if(!QDir(newName).exists())
		QDir().mkdir(newName);

	tmp = sList.last();
	sList = tmp.split(".");

	newName.append(preFix);
	for(i = 0; i< sList.size()-1; i++)
		newName.append(sList[i]);
//	newName.append(postFix);
	if(sList.last().compare("pgm") == 0)
		newName.append(".ppm");
	else
		newName.append("." + sList.last());

	return newName;
}

void CalibThread::processCorners(std::vector<cv::Point2f>& corners,CameraID CAM, bool found[3], QString title)
{
	cv::Rect roiRect;
	bool autoDetection = false, manuelDetection = false;
	AnnotationType annotationFlag;
	if(CAM == dti::HAND){
		annotationFlag = _cornerFlagsHandEye._cornerAnnotation;
	}else{
		annotationFlag = _cornerFlags._cornerAnnotation;
	}
	switch (annotationFlag) {
		case AutoThenManual:
			autoDetection = true;
			manuelDetection = true;
		break;
		case OnlyManual:
			autoDetection = false;
			manuelDetection = true;
		break;
		case OnlyAuto:
			autoDetection = true;
			manuelDetection = false;
		break;
		default:
			break;
	}

	//	Calibrate based on the extracted/selected image-pairs
	if(autoDetection)
		found[CAM] = _pCalib->detectCorners(_imgSrc[CAM], corners);
	if (manuelDetection && !found[CAM]) {
		manuelMarkerRoiSelection(_imgSrc[CAM], title);
		while(!_sharedData->hasFreshAnnotation())
		{
			if(shouldStop()) {
				cleanUpAndStop(-1);
                Q_EMIT consoleSignal("\n", dti::VerboseLevel::INFO);
				return;
			}
			msleep(30);
		}
		roiRect = _sharedData->getROI();

		if (roiRect.area() <= 1 && roiRect.area() >= -1){
			//skip image
//			continue;
			found[CAM] = false;
			return;
		} else {

			// detect corners using rois and first
			// clip roi if larger than imgSize
			if (roiRect.x + roiRect.width > _imgSrc[CAM].size().width)
				roiRect.width = _imgSrc[CAM].size().width - roiRect.x;
			if (roiRect.y + roiRect.height> _imgSrc[CAM].size().height)
				roiRect.height = _imgSrc[CAM].size().height - roiRect.y;

			found[CAM] = _pCalib->detectCorners(_imgSrc[CAM](roiRect), corners);

			// rescale points according to roi.
			for (int i = 0; i < (int) corners.size(); i++)
				corners[i] += cv::Point2f(roiRect.x, roiRect.y);

		}
	}
}

void CalibThread::manuelMarkerRoiSelection(cv::Mat img, QString title)
{
	QImage imgQ = SharedData::cvImg2QImg(&img);
	_sharedData->setAnnotationRoi(false);
	Q_EMIT annotateGuiSignal(imgQ, title); // signals the thread that the image needs to be annotated
}

void CalibThread::displayImagePair(cv::Mat imgs[2])
{
	_sharedData->setImgL(imgs[LEFT]);
	_sharedData->setImgR(imgs[RIGHT]);
	// Q_EMIT signal to Gui
//	Q_EMIT consoleSignal("Changed so that is it always LEFT img to be shown\n");
	Q_EMIT updateGuiPreviewSignal();
}

void CalibThread::displayCorners()
{

	_sharedData->setImgL(_imgCorners[LEFT]);
	_sharedData->setImgR(_imgCorners[RIGHT]);
	_sharedData->setImgHandEye(_imgCorners[HAND]);

	Q_EMIT updateGuiPreviewSignal();
}

void CalibThread::displayRectified()
{

	_sharedData->setImgL(_imgRectified[LEFT]);
	_sharedData->setImgR(_imgRectified[RIGHT]);
	// emit signal to Gui
//	Q_EMIT consoleSignal("Changed so that is it always LEFT img to be shown\n");
	Q_EMIT updateGuiPreviewSignal();
	// but remember to connect signal/slot!
//	Q_EMIT displayCornersSignal();
}
void CalibThread::annotateImage(cv::Mat img)
{
	// start gui annotation
    Q_EMIT annotateGuiSignal(SharedData::cvImg2QImg(&img), "");
    Q_EMIT consoleSignal("Waiting for roi ", dti::VerboseLevel::INFO);
}

void CalibThread::doCommitToROS()
{
	std::cout << "Commit to ROS" << std::endl;
	_pCalibStereo->storeROSCalibResults(_sharedData->getCameraNameLeft().toStdString(),_sharedData->getCameraNameRight().toStdString(),2);
}

void CalibThread::doNarrowStereoCalib()
{
	//_pCalib->narrowStereoCalib();
	using namespace cv;
    Q_EMIT consoleSignal("Computing Narrow stereo extrinsic using single camera intrinsic and camera pose estimation! \n", dti::VerboseLevel::INFO);

	/* Read intrinsic parameters from files */
	QString intrinsicLeftPath = _sharedData->getNarrowIntrinsicLeftPath();
	QString intrinsicRightPath = _sharedData->getNarrowIntrinsicRightPath();

	updateCalibrationFlags();
	_pCalibStereo->updateCalibrationFlags();
	//_pCalibStereo->setBoardSize(_sharedData->getBoardSize());
	//_pCalibStereo->setSquareSize(_sharedData->getSquareSize());

	cv::Mat intrinsicLeft, distortionLeft;
	cv::Mat intrinsicRight, distortionRight;
	_pCalibStereo->readIntrinsic(intrinsicLeftPath.toStdString(),
							&intrinsicLeft, 	&distortionLeft,
							CAMERA_MATRIX_NAME,
							DISTORTION_COEFF_NAME );

	_pCalibStereo->readIntrinsic(intrinsicRightPath.toStdString(),
							&intrinsicRight, 	&distortionRight,
							CAMERA_MATRIX_NAME,
							DISTORTION_COEFF_NAME );

	/* Load images */
	QString imageLeftPath = _sharedData->getNarrowImageLeftPath();
	QString imageRightPath = _sharedData->getNarrowImageRightPath();
	cv::Mat imageLeft = cv::imread(imageLeftPath.toStdString(),0);
	cv::Mat imageRight = cv::imread(imageRightPath.toStdString(),0);

	/* Undistort images */
	cv::Mat imageLeft_undist,imageRight_undist;
	cv::undistort(imageLeft,imageLeft_undist,intrinsicLeft,distortionLeft);
	cv::undistort(imageRight,imageRight_undist,intrinsicRight,distortionRight);

	/* Detect points */
	bool found[2] = {false, false};
	std::vector<cv::Point2f> cornersLeft, cornersRight;

	try{

        Q_EMIT consoleSignal("Narrow Stereo: Detecting corners\n", dti::VerboseLevel::INFO);
		_imgSrc[LEFT] 	=	imageLeft_undist;
		_imgSrc[RIGHT] 	=	imageRight_undist; // TODO REMEMBER that i've forced gray read

		//	detect left corners and if succeeded detect Right aswell
		processCorners(cornersLeft, LEFT, found, QString("Annotate Image"));

		// only process the right if marker is found in the RIGHT
		if (found[LEFT]) {
            Q_EMIT consoleSignal(		"  --> Marker found in LEFT image\n", dti::VerboseLevel::INFO);
			processCorners(cornersRight, RIGHT, found,"Annotate Image #");
        } else Q_EMIT consoleSignal(	"  --> Marker NOT found in LEFT image\n", dti::VerboseLevel::WARN);

        if(found[RIGHT]) 	Q_EMIT consoleSignal("  --> Marker found in RIGHT image\n", dti::VerboseLevel::INFO);
        else     			Q_EMIT consoleSignal("  --> Marker NOT found in RIGHT image\n", dti::VerboseLevel::WARN);

		/* Display corners */
		if(_cornerFlags._displayCorners) {
			_imgCorners[LEFT]  = _pCalibStereo->drawCorners(imageLeft_undist,  cornersLeft, found[LEFT]);
			_imgCorners[RIGHT] = _pCalibStereo->drawCorners(imageRight_undist, cornersRight, found[RIGHT]);
			displayCorners();
		}

	}catch(cv::Exception &h){
			std::cout <<"In Narrow stereo corner detection " << h.what() << std::endl;
	}

	if(found[LEFT] && found[RIGHT]){
		/* World points (Real world chessboard corners in mm)*/

		cv::Size boardSize = _sharedData->getBoardSize();// ->getNarrowBoardSize();
		std::cout << "boardsize " << boardSize.height << " " << boardSize.width << std::endl;
		double squareSize =_sharedData->getSquareSize();
		vector<Point3f> modelPoints;
		CalBoardTypes board = _sharedData->getBoardType();

		switch(board){
			case CHECKERBOARD:
				/* The calibration target is a checker board */
				for(int i = 0; i <= boardSize.height-1;i++){
					for(int j = 0; j <= boardSize.width-1;j++){
						modelPoints.push_back(Point3f((squareSize*j),(squareSize*i),0.00f));
						}
					}
				break;

			case ASYMMETRIC_CIRCLES_GRID:
				/* The calibration target is a ASYMMETRIC_CIRCLES_GRID */
				for(int i = 0; i <= boardSize.height-1;i++){
					for(int j = 0; j <= boardSize.width-1;j++){
						modelPoints.push_back(Point3f((squareSize*j),(squareSize*i),0.00f));
						}
				}
				break;
			case SYMMETRIC_CIRCLES_GRID:
				/* The calibration target is a SYMMETRIC_CIRCLES_GRID */
				for(int i = 0; i <= boardSize.height-1;i++){
					for(int j = 0; j <= boardSize.width-1;j++){
						modelPoints.push_back(Point3f((squareSize*j),(squareSize*i),0.00f));
						}
					}
				break;

			}

			/*Compute camera pose*/
			cv::Mat rvecL, rvecR, tvecL, tvecR, rvecL_matrix,rvecR_matrix;
			cv::Mat R1,R2,P1,P2,Q, r,t;
			cv::Matx31d T;
			cv::Matx33d R;

			try{
				solvePnP(modelPoints,cornersLeft,intrinsicLeft,Mat::zeros(5,1,CV_32F),rvecL,tvecL,false,CV_ITERATIVE);
				solvePnP(modelPoints,cornersRight,intrinsicRight,Mat::zeros(5,1,CV_32F),rvecR,tvecR,false,CV_ITERATIVE);

				cv::Rodrigues(rvecL,rvecL_matrix);
				cv::Rodrigues(rvecR,rvecR_matrix);

				cv::Matx44d PL;
				PL.val[0] = rvecL_matrix.at<double>(0,0); PL.val[1] = rvecL_matrix.at<double>(0,1); PL.val[2] = rvecL_matrix.at<double>(0,2); PL.val[3] = tvecL.at<double>(0);
				PL.val[4] = rvecL_matrix.at<double>(1,0); PL.val[5] = rvecL_matrix.at<double>(1,1); PL.val[6] = rvecL_matrix.at<double>(1,2); PL.val[7] = tvecL.at<double>(1);
				PL.val[8] = rvecL_matrix.at<double>(2,0); PL.val[9] = rvecL_matrix.at<double>(2,1); PL.val[10] = rvecL_matrix.at<double>(2,2); PL.val[11] = tvecL.at<double>(2);
				PL.val[12] = 0; PL.val[13] = 0; PL.val[14] = 0; PL.val[15] = 1;

				cv::Matx44d PR;
				PR.val[0] = rvecR_matrix.at<double>(0,0); PR.val[1] = rvecR_matrix.at<double>(0,1); PR.val[2] = rvecR_matrix.at<double>(0,2); PR.val[3] = tvecR.at<double>(0);
				PR.val[4] = rvecR_matrix.at<double>(1,0); PR.val[5] = rvecR_matrix.at<double>(1,1); PR.val[6] = rvecR_matrix.at<double>(1,2); PR.val[7] = tvecR.at<double>(1);
				PR.val[8] = rvecR_matrix.at<double>(2,0); PR.val[9] = rvecR_matrix.at<double>(2,1); PR.val[10] = rvecR_matrix.at<double>(2,2); PR.val[11] = tvecR.at<double>(2);
				PR.val[12] = 0; PR.val[13] = 0; PR.val[14] = 0; PR.val[15] = 1;

				cv::Matx44d RT = PL*PR.inv();

				//std::cout << "RT" << std::endl;
				//std::cout << (Mat)RT << std::endl;

				T.val[0] = RT.val[3];
				T.val[1] = RT.val[7];
				T.val[2] = RT.val[11];

				R.val[0] = RT.val[0]; R.val[1] = RT.val[1]; R.val[2] = RT.val[2];
				R.val[3] = RT.val[4]; R.val[4] = RT.val[5]; R.val[5] = RT.val[6];
				R.val[6] = RT.val[8]; R.val[7] = RT.val[9]; R.val[8] = RT.val[10];

				r = cv::Mat(R);
				t = cv::Mat(T);

				std::cout << "R" << std::endl;
				std::cout << r << std::endl;

				std::cout << "T" << std::endl;
				std::cout << t << std::endl;


			}catch(cv::Exception &h)
			{
				std::cout << h.what() << std::endl;
			}

			_pCalibStereo->clearPoints();
			_pCalibStereo->resetCalirationData();
			_pCalibStereo->addImagePointPairs(cornersLeft, cornersRight,1);
			_pCalibStereo->calibrateNarrowStereo(intrinsicLeft,distortionLeft, intrinsicRight, distortionRight,r,t,true);


		}

		/* Compute fundamental matrix */
	/*	Mat F;
		F = findFundamentalMat(cornersLeft, cornersRight,CV_FM_8POINT,1,0.99);

		cv::Mat H1, H2;
		stereoRectifyUncalibrated(cornersLeft, cornersRight, F, _imageSize, H1, H2);

		/* Compute Rectification matrix */
	/*	Mat R1, R2;
		R1 = intrinsicLeft.inv() * H1 * intrinsicLeft;
		R2 = intrinsicRight.inv() * H2 * intrinsicRight ;

		std::cout << "R1:" << std::endl;
		std::cout << R1 << std::endl;
		std::cout << "R2:" << std::endl;
		std::cout << R2 << std::endl;

	*/

		/* Compute fundamental matrix */
	/*	Mat F;
		F = findFundamentalMat(cornersLeft, cornersRight,CV_FM_8POINT,1,0.99);

		std::cout << "Computed fundamental matrix" << std::endl;
		std::cout << F << std::endl;

		/* Compute essential matrix */
	/*	Mat E = intrinsicRight.t() * F * intrinsicLeft;
		std::cout << "Computed essential matrix" << std::endl;
		std::cout << E << std::endl;

		// Hardcoded for testing
	/*	F.at<double>(0,0) = 6.297904524229473e-09;
		F.at<double>(0,1) = 1.145611769630351e-07;
		F.at<double>(0,2) = -0.001088311675746321;
		F.at<double>(1,0) = 4.59834349380955e-07;
		F.at<double>(1,1) = -8.197953952356671e-07;
		F.at<double>(1,2) = -0.09135166576766639;
		F.at<double>(2,0) = 0.0006023918552661347;
		F.at<double>(2,1) = 0.09214003768165108;
		F.at<double>(2,2) = 1.0;

		std::cout << "Hardcoded fundamental matrix" << std::endl;
		std::cout << F << std::endl;
	*/


	/*	// Hardcoded for testing
		E.at<double>(0,0) = -0.01232992045435357;
		E.at<double>(0,1) = -0.22422980352633;
		E.at<double>(0,2) = 3.870502878618114;
		E.at<double>(1,0) = -0.8996888103806999;
		E.at<double>(1,1) = 1.60357047446212;
		E.at<double>(1,2) = 333.7291378497401;
		E.at<double>(2,0) = -2.609624556486241;
		E.at<double>(2,1) = -333.7425375101612;
		E.at<double>(2,2) = 1.593784350334587;

		std::cout << "Hardcoded essential matrix" << std::endl;
		std::cout << E << std::endl;
	*/
		/* Decompose E */
	/*	SVD decomp = SVD(E);

		//U
		Mat U = decomp.u;

		std::cout << "U " << std::endl;
		std::cout << U << std::endl;


		//S
		Mat S(3, 3, CV_64F, Scalar(0));
		S.at<double>(0, 0) = decomp.w.at<double>(0, 0);
		S.at<double>(1, 1) = decomp.w.at<double>(0, 1);
		S.at<double>(2, 2) = decomp.w.at<double>(0, 2);

		//V
		Mat V = decomp.vt.t(); //Needs to be decomp.vt.t(); (transpose once more)

		//W
		Mat W(3, 3, CV_64F, Scalar(0));
		W.at<double>(0, 1) = -1;
		W.at<double>(1, 0) = 1;
		W.at<double>(2, 2) = 1;

		Mat rot1, rot2, rodz1, rodz2;
		rot1 = U * W.t() * V.t();
		rot2 = U * W * V.t();

		/* Calculate rotation matrix */
	/*	std::cout << "Computed rotation 1: " << std::endl;
		std::cout << rot1 << std::endl;
		std::cout << "Computed rotation 2:" << std::endl;
		std::cout <<  rot2 << std::endl;

		cv::Rodrigues(rot1,rodz1);
		cv::Rodrigues(rot2,rodz2);

		std::cout << "Computed Rodrigues rotation 1: " << std::endl;
		std::cout << rodz1 << std::endl;
		std::cout << "Computed Rodrigues rotation 2: " << std::endl;
		std::cout << rodz2 << std::endl;
	*/
		/* Another test approach */
	/*	cv::Mat H1, H2;
		stereoRectifyUncalibrated(cornersLeft, cornersRight, F, _imageSize, H1, H2);

		Mat rot1, rot2, rodz1, rodz2;
		rot1 = intrinsicLeft.inv() * H1 * intrinsicLeft;
		rot2 = intrinsicRight.inv() * H2 * intrinsicRight ;

		cv::Rodrigues(rot1,rodz1);
		cv::Rodrigues(rot2,rodz2);

		std::cout << "R1:" << std::endl;
		std::cout << rot1 << std::endl;
		std::cout << "R2:" << std::endl;
		std::cout << rot2 << std::endl;
		std::cout << "Computed Rodrigues rotation 1: " << std::endl;
		std::cout << rodz1 << std::endl;
		std::cout << "Computed Rodrigues rotation 2: " << std::endl;
		std::cout << rodz2 << std::endl;


		/*std::cout << "P1:" << std::endl;
		std::cout << intrinsicLeft << std::endl;
		std::cout << "P2:" << std::endl;
		std::cout << intrinsicRight << std::endl;
	*/
}































//bool CalibThread::getImgCornerStereo(cv::Mat *imgOutL, cv::Mat *imgOutR) {
//	QMutexLocker locker(&_mutexImgCorners[LEFT]);
//	_imgCorners[LEFT].copyTo(*imgOut);
//	return true;
//}
//bool CalibThread::getImgR(cv::Mat *imgOut) {
//	QMutexLocker locker(&_mutexImgCorners[RIGHT]);
//	_imgCorners[RIGHT].copyTo(*imgOut);
//	return true;
//}

/************
void StereoCalib::openCVCalib(const QVector<QString>& imagelist, cv::Size boardSize, bool useCalibrated=true, bool showRectified=true)
{
	using namespace cv;

	if( imagelist.size() % 2 != 0 )
	    {
			Q_EMIT consoleSignal(QString("Error: the image list contains odd (non-even) number of elements\n"));
	        return;
	    }

	    bool displayCorners = false;//true;
	    const int maxScale = 2;
	    const float squareSize = 1.f;  // Set this to your actual square size

	    // ARRAY AND VECTOR STORAGE:
	    vector<vector<Point2f> > imagePoints[2];
	    vector<vector<Point3f> > objectPoints;
	    Size imageSize;

	    int i, j, k, nimages = (int)imagelist.size()/2;

	    imagePoints[0].resize(nimages);
	    imagePoints[1].resize(nimages);
	    vector<string> goodImageList;

	    // detect Corners in all pairs
	    for( i = j = 0; i < nimages; i++ )
	    {
	        for( k = 0; k < 2; k++ )
	        {
	            const QString& filename = imagelist[i*2+k];
	            Mat img = imread(filename.toStdString(), 0);
	            if(img.empty())
	                break;
	            if( imageSize == Size() )
	                imageSize = img.size();
	            else if( img.size() != imageSize )
	            {
	            	Q_EMIT consoleSignal(QString("The image ") << filename << (" has the size different from the first image size. Skipping the pair\n"));
	                break;
	            }

				vector<cv::Point2f>& corners = imagePoints[k][i];
				bool found = detectCorners(img,corners);
	            if( displayCorners )
	            {
	                Q_EMIT consoleSignal(filename + "\n");

	                Mat cimg, cimg1;
	                cvtColor(img, cimg, CV_GRAY2BGR);
	                drawChessboardCorners(cimg, boardSize, corners, found);
	                double sf = 640./MAX(img.rows, img.cols);
	                resize(cimg, cimg1, Size(), sf, sf);
	                if (k == 0) {
						setImgLCorner(&cimg1);
	                } else if (k == 1) {
	                	setImgRCorner(&cimg1);;
	                }
						Q_EMIT signal to indicate new img //imshow("corners", cimg1);
	                char c = (char)waitKey(500);
	                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
	                    exit(-1);
	            }
	            else
	                putchar('.');
	            if( !found )
	                break;
	            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
	                         TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
	                                      30, 0.01));
	        }
	        if( k == 2 )
	        {
	            goodImageList.push_back(imagelist[i*2]);
	            goodImageList.push_back(imagelist[i*2+1]);
	            j++;
	        }
	    }
	    cout << j << " pairs have been successfully detected.\n";
	    nimages = j;
	    if( nimages < 2 )
	    {
	        cout << "Error: too little pairs to run the calibration\n";
	        return;
	    }

	    imagePoints[0].resize(nimages);
	    imagePoints[1].resize(nimages);
	    objectPoints.resize(nimages);

	    for( i = 0; i < nimages; i++ )
	    {
	        for( j = 0; j < boardSize.height; j++ )
	            for( k = 0; k < boardSize.width; k++ )
	                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
	    }

	    cout << "Running stereo calibration ...\n";

	    Mat cameraMatrix[2], distCoeffs[2];
	    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	    Mat R, T, E, F;

	    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
	                    cameraMatrix[0], distCoeffs[0],
	                    cameraMatrix[1], distCoeffs[1],
	                    imageSize, R, T, E, F,
	                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
	                    CV_CALIB_FIX_ASPECT_RATIO +
	                    CV_CALIB_ZERO_TANGENT_DIST +
	                    CV_CALIB_SAME_FOCAL_LENGTH +
	                    CV_CALIB_RATIONAL_MODEL +
	                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
	    cout << "done with RMS error=" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	    double err = 0;
	    int npoints = 0;
	    vector<Vec3f> lines[2];
	    for( i = 0; i < nimages; i++ )
	    {
	        int npt = (int)imagePoints[0][i].size();
	        Mat imgpt[2];
	        for( k = 0; k < 2; k++ )
	        {
	            imgpt[k] = Mat(imagePoints[k][i]);
	            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
	            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
	        }
	        for( j = 0; j < npt; j++ )
	        {
	            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
	                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
	                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
	                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
	            err += errij;
	        }
	        npoints += npt;
	    }
	    cout << "average reprojection err = " <<  err/npoints << endl;

	    // save intrinsic parameters
	    FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
	    if( fs.isOpened() )
	    {
	        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
	            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
	        fs.release();
	    }
	    else
	        cout << "Error: can not save the intrinsic parameters\n";

	    Mat R1, R2, P1, P2, Q;
	    Rect validRoi[2];

	    stereoRectify(cameraMatrix[0], distCoeffs[0],
	                  cameraMatrix[1], distCoeffs[1],
	                  imageSize, R, T, R1, R2, P1, P2, Q,
	                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	    fs.open("extrinsics.yml", CV_STORAGE_WRITE);
	    if( fs.isOpened() )
	    {
	        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
	        fs.release();
	    }
	    else
	        cout << "Error: can not save the intrinsic parameters\n";

	    // OpenCV can handle left-right
	    // or up-down camera arrangements
	    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	    if( !showRectified )
	        return;

	    Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	    if( useCalibrated )
	    {
	        // we already computed everything
	    }
	// OR ELSE HARTLEY'S METHOD
	    else
	 // use intrinsic parameters of each camera, but
	 // compute the rectification transformation directly
	 // from the fundamental matrix
	    {
	        vector<Point2f> allimgpt[2];
	        for( k = 0; k < 2; k++ )
	        {
	            for( i = 0; i < nimages; i++ )
	                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
	        }
	        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
	        Mat H1, H2;
	        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

	        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
	        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
	        P1 = cameraMatrix[0];
	        P2 = cameraMatrix[1];
	    }

	    //Precompute maps for cv::remap()
	    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	    Mat canvas;
	    double sf;
	    int w, h;
	    if( !isVerticalStereo )
	    {
	        sf = 600./MAX(imageSize.width, imageSize.height);
	        w = cvRound(imageSize.width*sf);
	        h = cvRound(imageSize.height*sf);
	        canvas.create(h, w*2, CV_8UC3);
	    }
	    else
	    {
	        sf = 300./MAX(imageSize.width, imageSize.height);
	        w = cvRound(imageSize.width*sf);
	        h = cvRound(imageSize.height*sf);
	        canvas.create(h*2, w, CV_8UC3);
	    }

	    for( i = 0; i < nimages; i++ )
	    {
	        for( k = 0; k < 2; k++ )
	        {
	            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
	            remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
	            cvtColor(rimg, cimg, CV_GRAY2BGR);
	            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
	            resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
	            if( useCalibrated )
	            {
	                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
	                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
	                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
	            }
	        }

	        if( !isVerticalStereo )
	            for( j = 0; j < canvas.rows; j += 16 )
	                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	        else
	            for( j = 0; j < canvas.cols; j += 16 )
	                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
	        imshow("rectified", canvas);
	        char c = (char)waitKey();
	        if( c == 27 || c == 'q' || c == 'Q' )
	            break;
	    }
}
*/
} /* namespace dti */
