/*
 * MonoCalibration.h
 *
 *  Created on: Dec 6, 2011
 *      Author: RAHA
 */

#ifndef MONOCALIBRATION_H_
#define MONOCALIBRATION_H_

#include "Calibration.hpp"

namespace dti {

class MonoCalibration :public Calibration{

public:
	MonoCalibration(SharedData* sharedData);
	virtual ~MonoCalibration();
	void init();

	double calibrate();
	void storeCalibResults(std::string path);
	void storeROSCalibResults();
	void clearPoints();
	void updateCalibrationFlags();
	void updateCalibrationFlags(CornerDetectionFlags cornerFlags, CalibrationFlags calFlags, RectificationFlags undistFlags);
	void resizePoints(int fileNum );
	void initUndist();
	int  undistortImage(const cv::Mat& imgSrc, cv::Mat& imgUndist);
	int  addImagePoints(std::vector<cv::Point2f> corners, int imgId);
	void printResults();
	void printCalibResults();
	std::string calibResultsAsString();
	void setFilesLists(QStringList files);
	void storeResults(std::string path);
	bool isUndistReady();
	QString getFileName(int imgId);
	QString getGoodImgFilename(int i) { return getFileName(_goodImages[i]); }


private:
	/** Member variables 	**/
	QStringList 		_fileList;
	int					_validImgs;

	RectificationFlags 	_undistFlags;
    std::vector<std::vector<cv::Point2f> > _imagePoints;
    std::vector<std::vector<cv::Point3f> > _objectPoints;

	cv::Mat 		_camMatInit,
					_distortionInit,
					_camMatRes,
					_distRes;

	cv::Rect 		_validRoi;
	bool 			_undistMapValid;
	cv::Mat 		_undistMap[2];

	std::vector<cv::Mat> 	_rvecs, _tvecs;
};

} /* namespace dti */
#endif /* MONOCALIBRATION_H_ */
