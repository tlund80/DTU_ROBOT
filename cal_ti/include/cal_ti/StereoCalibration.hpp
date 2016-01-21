/*
 * StereoCalibration.h
 *
 *  Created on: Dec 6, 2011
 *      Author: RAHA
 */

#ifndef STEREOCALIBRATION_H_
#define STEREOCALIBRATION_H_

#include "Calibration.hpp"
#include <qmutex.h>
#include <qdir.h>
#include <qstring.h>
#include <qstringlist.h>
namespace dti {

class StereoCalibration : public Calibration{
	Q_OBJECT;

public:
	StereoCalibration(SharedData* sharedData);
	virtual ~StereoCalibration();
	void init();

//	cv::Mat _img;

	void setImageSizeStereo(cv::Size size1,cv::Size size2);
	void setFilesLists(QStringList listL, QStringList listR);
	QString getFileName(int imgId, int CAM);
	void resizePoints(int fileNum );
//	bool detectCorners(int index);
	void updateCalibrationFlags(CornerDetectionFlags cornerFlags, CalibrationFlags calFlags, StereoCalibFlags stereoFlags, RectificationFlags rectFlags);
	void updateCalibrationFlags();
	int  addImagePointPairs(std::vector<cv::Point2f> cornersL, std::vector<cv::Point2f> cornersR, int imgId);
	void clearPoints();
	void resetCalirationData();
	void setIntrinsicAndDistortion(int CAM, cv::Mat intrinsic, cv::Mat distortion);
	void setIntrinsic(cv::Mat left, cv::Mat right);
	void setDistortion(cv::Mat left, cv::Mat right);
	double  calibrate();
	void  rectify();

	void printResults();
	void storeResults(std::string);
	void storeRectificationResults(std::string);
	void storeCalibResults(std::string path);
	void storeROSCalibResults(std::string cameraNameL, std::string cameraNameR, int cameraCount);
	std::string resultsAsString();
	std::string rectificationAsString();
	void printCalibResults();
	void printRectificationRes();
	void initRectification();
	int rectifyImagePair(cv::Mat imgSrc[2], cv::Mat imgRes[2]);

	QString getGoodImgFilename(int i, int CAM) { return getFileName(_goodImages[i], CAM); }

	bool isRectificationReady();

	void calibrateNarrowStereo(cv::Mat& cameraMatrix1, cv::Mat& Dist1,
			  	  	  	  	   cv::Mat& cameraMatrix2, cv::Mat& Dist2,
			  	  	  	  	   cv::Mat& R, cv::Mat& T, bool print);
private:

	int getRectFAlg();
	int doRectified(cv::Mat imgRectifiedOut[2], int i = -1);
	int calcStereoTerminationFlags();
	int getStereoTermIterations();
	int getStereoTermEpsilon();
	int calcStereoFlags();
	int calcRectficationFlags();
	int cleanUpAndStop(int reason);
	bool shouldStop();


//	bool setImgLCorner(cv::Mat *imgIn) {
//		QMutexLocker locker(&_mutexImgCorners[LEFT]);
//		imgIn->copyTo(_imgCorners[LEFT]);
//		emit imgCornerSignalL();
//		return true;
//	}
//
//	bool setImgRCorner(cv::Mat *imgIn) {
//		QMutexLocker locker(&_mutexImgCorners[RIGHT]);
//		imgIn->copyTo(_imgCorners[RIGHT]);
//		emit imgCornerSignalR();
//		return true;
//	}


	//------------------- VARS ----------------------//


//	cv::Mat _imgSrc[2];
//	cv::Mat _imgCorners[2];
	cv::Mat _imgRectified[2];

    // ARRAY AND VECTOR STORAGE:
    std::vector<std::vector<cv::Point2f> > _imagePoints[2];
    std::vector<std::vector<cv::Point3f> > _objectPoints;

	QMutex 		_mutexRunning;
	QMutex 		_mutexImgCorners[2];
	QDir 		_imgPath[2];
	QStringList _fileList[2];

//    std::vector<int>		_goodImages;
//    std::vector<int>		_badImages;
	int _validPairs;
	cv::Size 		_imageSizeR; 		// Size of the right images
	cv::Mat 		_camMatInit[2],
					_distortionInit[2],
					_camMatRes[2],
					_distRes[2],
					_R, _T, _E, _F;

	cv::Mat 		_R1, _R2, _P1, _P2, _Q; // Retify params
	cv::Rect 		_validRoi[2];
	cv::Mat 		_rectificationMap[2][2];
	bool			_rectificationMapValid;

    StereoCalibFlags 		_stereoFlags;
    RectificationFlags		_rectFlags;

//	double 			_rectifyFlag;
//	double 			_rectificationAlpha;
//	bool 			_useCalibrated;
//	bool 			_doStoreRectified;
};

} /* namespace dti */
#endif /* STEREOCALIBRATION_H_ */
