/*
 * Calibration.h
 *
 *  Created on: Dec 6, 2011
 *      Author: RAHA
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_
#include "SharedData.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <qobject.h>

namespace dti {

class Calibration : public QObject{
	Q_OBJECT;


public:

	Calibration();
	virtual ~Calibration();
	void 	 manuelMarkerRoiSelection(cv::Mat img);
	cv::Size getBoardSize()	{ return _boardSize; }
	cv::Size getImageSize()	{ return _imageSize; }
	int   	 getMaxScale()	{ return _maxScale;  }
	double 	 getSquareSize() { return _squareSize;}


	void setImageSize(cv::Size size) 	{ _imageSize = size; 	}
	void setBoardSize(cv::Size size)	{ _boardSize  = size; 	}
	void setSquareSize(double square) 	{ _squareSize = square; }

	virtual void printResults() 	 	= 0;
	virtual void storeResults(std::string path) 	 	= 0;
	virtual void init() 			 	= 0;
	virtual double calibrate() 		 	= 0;
	virtual void printCalibResults() 	= 0;
	virtual void updateCalibrationFlags() = 0;

	void addToBadImgs(int imgId);

	//	bool 	detectCorners(int index) = 0;
	bool  	detectCorners(const std::string& imgPath, std::vector<cv::Point2f>& corners);
	bool 	detectCorners(const cv::Mat& img, std::vector<cv::Point2f>& corners);
	cv::Mat drawCorners(const cv::Mat& img, const std::vector<cv::Point2f>& corners, bool completePatternFound);
	cv::Mat draw3dAxis(cv::Mat &Image, const std::vector<cv::Point3f>& objectPoints, cv::Mat rvec, cv::Mat tvec,const cv::Mat cameraMatrix, const cv::Mat distortion);

	bool 	readIntrinsic(const std::string& pathR, cv::Mat* intrinsicR, cv::Mat* distortion,
							const std::string& intrinsicMatName = "camera_matrix", const std::string& distortionMatName = "distortion_coefficients");

	int getGoodImgNum() {return _goodImages.size();}
	int getGoodImgId(int i) { return _goodImages[i]; }

	void narrowStereoCalib();
//	virtual void displayCorners() = 0;
Q_SIGNALS:
	void 	updateGuiSignal();// er n�et her til;
	void 	annotateGuiSignal(QImage imgQ);// er n�et her til;
    void 	consoleSignal(QString msg, dti::VerboseLevel);

protected:
	bool 	doPreCornerCheck();

	int 	checkForCorners(const cv::Mat& img);
	void 	setSharedData(SharedData *sharedData) { _sharedData = sharedData; }

	int	calcCornerFlags();
	int calcCalibFlags();
	int	calcCornerTerminationFlags();
	int getCornerTerminationIte() 			{ return _cornerFlags._stopOnIterations;}
	double getCornerTerminationEpsilon()  	{ return _cornerFlags._stopOnEpsilon;	}

	void printValues(std::vector<std::vector<cv::Point2f> >& vals, const QString& name = "");
	void printValues(std::vector<std::vector<cv::Point3f> >& vals, const QString& name = "");

	/************************
	 *  Private vars
	 ************************/
	SharedData 				*_sharedData;

    cv::Size 				_imageSize;
    cv::Size 				_boardSize;
    int  					_maxScale;

    CornerDetectionFlags 	_cornerFlags;
    CalibrationFlags 		_calFlags;

    double 					_rms;		//
    double 					_squareSize; 		// = 1.f;  // Set this to your actual square size

    std::vector<int>		_goodImages;
    std::vector<int>		_badImages;

    cv::Mat GetFundamentalMat(const std::vector<cv::Point2f>& imgpts1, const std::vector<cv::Point2f>& imgpts2);
};

} /* namespace dti */
#endif /* CALIBRATION_H_ */
