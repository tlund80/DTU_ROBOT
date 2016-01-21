/*
 * stereoCalib.h
 *
 *  Created on: Dec 2, 2011
 *      Author: RAHA
 */

#ifndef CALIBTHREAD_H_
#define CALIBTHREAD_H_

#include "config.hpp"
#include "ImgAnnotation.hpp"
//#include "../SharedData.h"
//#include "Calibration.h"
#include "StereoCalibration.hpp"
#include "HandEyeCalibration.hpp"
#include "MonoCalibration.hpp"
#include <opencv2/opencv.hpp>

#include <qthread.h>
#include <qmutex.h>
#include <qvector.h>
#include <qstringlist.h>
#include <qdir.h>

#include "../../src/marker/aruco.h"
#include "../../src/marker/cvdrawingutils.h"

using namespace aruco;

namespace dti {

class CalibThread: public QThread{
	Q_OBJECT;
public:
	CalibThread(SharedData *sharedData);
	virtual ~CalibThread();
	void run();
	void runStereo();
	void runMono();
	void runStereoNew();
	void runHandEye();
	double detectMonoCorners();
	double detectStereoCorners();
	double detectHandEyeCorners();
	double calibrate(bool print = false);
	double rectification(bool print = false);
	double undistort();
	void deactivate(bool print = false);
	void activate(bool print = false);
	void toggleThread();
	bool isRunning();

	bool getImgL(cv::Mat *imgOut);
	bool getImgR(cv::Mat *imgOut);
	Calibration* getStereoCalibration() { return _pCalibStereo;	}
	Calibration* getMonoCalibration() 	{ return _pCalibMono;	}
	Calibration* getHandEyeCalibration() 	{ return _PCalibHandEye;	}
	QStringList getFileList(CameraID cam);
//	void setDoStop(bool stop);
	void doAll();
	void doCalibrate();
	void doDetectCorners();
	void doRectify();
	void doSaveResults();
	void doStop();
	void doComputePose() ;
	void doCommitToROS();
	void doNarrowStereoCalib();
	void doClearAllData();
	void dosaveRobotPoses();
    void doLoadRobotPoses();

Q_SIGNALS:
//signals:
    void consoleSignal(QString msg, dti::VerboseLevel);
	void imgLSignal();
	void imgRSignal();
	void finishedSignal();
	void annotateGuiSignal(QImage imgQ, QString title);
	void updateGuiPreviewSignal();
	void updateGuiIntrinsicValsSignal();

public Q_SLOTS:
	void readIntrinsicGuessPath(dti::CameraID CAM, QString intrinsicMatName, QString distortionMatName);
//	void updateImgL(cv::Mat imgL);
//	void updateImgR(cv::Mat imgR);

private:
	QString getRectSaveFileName(QString orgName, bool isRef);
	int 	rectifyAllFromCalib();
	int 	undistAllFromCalib();
	void 	manuelMarkerRoiSelection(cv::Mat img, QString title = "");
	void 	displayImagePair(cv::Mat imgs[2]);
	void 	displayCorners();
	void 	displayRectified();
	int 	cleanUpAndStop(int reason);
	bool	shouldStop();
	void 	updateCalibrationFlags();
	void	annotateImage(cv::Mat img);
	void 	finished(bool print = false);
	void	finishedPart();
	void 	started(bool print = false);
//	bool doStop();
	void init();
	void initMono();
	void initStereo();
	void initHandEye();
	void openCVCalib(const QVector<QString>& imagelist, cv::Size boardSize, bool useCalibrated=true, bool showRectified=true);
	bool stereoCalib(cv::Mat img, std::vector<cv::Point2f>&);
	void processCorners(std::vector<cv::Point2f>& cornersL,CameraID CAM, bool found[3], QString title = "");
	bool shouldRectify();
	bool shouldDetectCorners();
	bool shouldCalibrate();

	StereoState getStereoState()
	{
		QMutexLocker lock(&_mutexState);
		return _sState;
	}

	void setStereoState(StereoState state) {
		QMutexLocker lock(&_mutexState);
		_sState = state;
	}

	MonoState getMonoState()
	{
		QMutexLocker lock(&_mutexState);
		return _mState;
	}

	void setMonoState(MonoState state) {
		QMutexLocker lock(&_mutexState);
		_mState = state;
	}

	HandEyeState getHandEyeState()
	{
		QMutexLocker lock(&_mutexState);
		return _hState;
	}

	void setHandEyeState(HandEyeState state) {
		QMutexLocker lock(&_mutexState);
		_hState = state;
	}

    // ARRAY AND VECTOR STORAGE:
//    std::vector<std::vector<cv::Point2f> > _imagePoints[2];
    std::vector<std::vector<cv::Point3f> > _objectPoints;

    cv::Mat _imgSrc[CALTI_CAMS];
	cv::Mat _imgCorners[CALTI_CAMS];
	cv::Mat _imgRectified[CALTI_CAMS];

	double 	_rms;
	volatile bool _running;
//	volatile bool _doStop;
	cv::Mat 	_img[CALTI_CAMS];
	QMutex 		_mutexRunning;
	QMutex 		_mutexDoStop;
	QMutex 		_mutexState;
//	QMutex 		_mutexImgCorners[MAX_CAMERA_2_CAL];
	QDir 		_imgPath[CALTI_CAMS];
	QStringList _fileList[CALTI_CAMS];

	StereoCalibration 		*_pCalibStereo;
	MonoCalibration 		*_pCalibMono;
	HandEyeCalibration	    *_PCalibHandEye;
	Calibration 			*_pCalib;
	SharedData				*_sharedData;

    CornerDetectionFlags 	_cornerFlags;
    CornerDetectionFlags 	_cornerFlagsHandEye;
    CalibrationFlags 		_calFlags;
    StereoCalibFlags 		_stereoFlags;
    RectificationFlags		_rectFlags;
    StereoState 			_sState;
    MonoState 				_mState;
    HandEyeState 			_hState;

};

} /* namespace dti */

#endif /* CALIBTHREAD_H_ */
