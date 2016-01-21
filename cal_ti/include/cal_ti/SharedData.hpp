/*
 * SharedData.h
 *
 *  Created on: Dec 6, 2011
 *      Author: RAHA
 */

#ifndef SHAREDDATA_H_
#define SHAREDDATA_H_

#include "config.hpp"
//#include "calibration/Calibration.h"
#include <opencv2/core/core.hpp>
#include <qmutex.h>
#include <qimage.h>
#include <qdir.h>
#include <cmath>

#include "tiv_types.hpp"

#include "rot.hpp"
namespace dti {

class SharedData {

private:

	// Storage path for rectifiec imgs
	QMutex 	_mutexRectSavePath;
	QMutex	_mutexSavePath;
	QMutex	_mutexHandEyeSavePath;
	QString savePath;
	QString saveHandEyePath;
	QString saveRectPathL;
	QString saveRectPathR;

	cv::Mat 			_imgL, _imgR, _imgHandEye;
	QDir 				_imgPathL, _imgPathR, _imgPathHandEyeMono;
	QStringList 		_nameFilter[CALTI_CAMS];//, _nameFilterR;
	bool				_shouldStop;
	CalibrationTypes 	_calType;
	CalibrationTypes 	_calTypeHandEye;

	QMutex 				_mutexBoardType;
	QMutex 				_mutexCalType;
	QMutex 				_mutexPaths;
	QMutex				_mutexNameFilter[CALTI_CAMS];//L, _mutexNameFilterR;
	QMutex				_mutexShoulsStop;
	QMutex 				_mutexImgL, _mutexImgR;
	QMutex				_mutexBoardSize;
	QMutex				_mutexSquareSize;
	QMutex				_mutexRobotPose;
	QMutex				_mutexRobotPoseArray;
	QMutex				_mutexRobotPath;
	QMutex				_mutexRobotLivePath;
	CalBoardTypes		_boardType;
	CalBoardTypes		_boardTypeHandEye;
	cv::Size			_boardSize;
	cv::Size			_boardSizeHandEye;
	double				_squareSize;
	double				_squareSizeHandEye;
	tiv::pose			_RobotPose;
	std::vector<tiv::pose> _RobotPoseArray;

	/**	    intrinsic guess vars	 **/
	QString _intrinsicGuessPathL, _intrinsicGuessPathR, _intrinsicHandEyePath;
	QMutex _mutexIntrinsicGuess;
	IntrinsicGuessVals _intrinsicGuessValsL, _intrinsicGuessValsR;

	/**		intrincsic result storage path  **/
	QString _intrinsicResStorePathR, _intrinsicResStorePathL;

	/**		Robot Pose load path  **/
	QString _robotPath;

	/**		Robot Pose storage path  **/
	QString _robotLivePath;

	/**	Calibration falgs		**/
	dti::CalibrationFlags 		_calFlags;
	dti::StereoCalibFlags		_stereoFlags;
	dti::CornerDetectionFlags 	_cornerFlags;
	dti::CornerDetectionFlags 	_cornerFlagsHandEye;
	dti::RectificationFlags		_rectFlags;
	dti::RobotFlags				_robotFlags;

	QMutex				_mutexCalFlag;
	QMutex				_mutexCornerFlag;
	QMutex				_mutexStereoFlag;
	QMutex				_mutexRobotFlag;

	/*		Annotation 	*/
	bool 				freshAnnotateRoi;
	cv::Rect 			_roi;
	QMutex				_mutexAnnotateRoi;

	QMutex mutexNarrowParameters;
	QMutex mutexleftROSCameraName;
	QMutex mutexrightROSCameraName;
	QString narrowIntrinsicLeftPath;
	QString narrowIntrinsicRightPath;
	QString narrowImageLeftPath;
	QString narrowImageRightPath;
	cv::Size narrowBoardSize;
	QString narrowOutputLeftPath;
	QString narrowOutputRightPath;
	QString leftROSCameraName;
	QString rightROSCameraName;

public:
	SharedData() {
		setStop(false);
		freshAnnotateRoi = false;
		clearIntrinsicGuessVals(LEFT);
		clearIntrinsicGuessVals(RIGHT);
		
		
		
    /*	tiv::pose p; Eigen::Vector3d eaa; Eigen::Matrix3d R;
		
		p.t.x = -232.09; p.t.y = -614.74; p.t.z = 649.62;
	        eaa(0,0) = 0.7785; eaa(1,0) = 0.7788; eaa(2,0) = -1.4279;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -171.43; p.t.y = -610.63; p.t.z = 654.56;
	        eaa(0,0) = 0.7705; eaa(1,0) = 0.7731; eaa(2,0) = -1.4287;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -248.60; p.t.y = -660.02; p.t.z = 608.67;
	        eaa(0,0) = 0.4580; eaa(1,0) = 0.4479; eaa(2,0) = -1.5316;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -210.16; p.t.y = -668.41; p.t.z = 539.36;
	        eaa(0,0) = 0.3689; eaa(1,0) = 0.4135; eaa(2,0) = -1.5534;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -156.31; p.t.y = -659.38; p.t.z = 546.26;
	        eaa(0,0) = 0.8788; eaa(1,0) = 0.3496; eaa(2,0) = -1.7192;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -204.82; p.t.y = -576.99; p.t.z = 511.73;
	        eaa(0,0) = 1.0860; eaa(1,0) = 0.8186; eaa(2,0) = -1.3827;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -269.05; p.t.y = -662.65; p.t.z = 465.34;
	        eaa(0,0) = -0.2965; eaa(1,0) = 0.8485; eaa(2,0) = -1.5829;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
        */

/*		p.t.x = -115.35; p.t.y = -697.83; p.t.z = 524.14;
	        eaa(0,0) = 0.4872; eaa(1,0) = -0.3269; eaa(2,0) = -1.3957;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -208.68; p.t.y = -522.14; p.t.z = 537.56;
	        eaa(0,0) = 1.0311; eaa(1,0) = 0.9927; eaa(2,0) = -1.2913;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
*/		
/*		p.t.x = -325.69; p.t.y = -678.86; p.t.z = 485.10;
	        eaa(0,0) = -0.1526; eaa(1,0) = 0.4677; eaa(2,0) = -1.3107;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -199.63; p.t.y = -566.34; p.t.z = 334.18;
	        eaa(0,0) = 0.2204; eaa(1,0) = 0.2683; eaa(2,0) = -1.4591;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
		
		p.t.x = -109.74; p.t.y = -545.03; p.t.z = 352.86;
	        eaa(0,0) = 0.8778; eaa(1,0) = 0.0167; eaa(2,0) = -1.5940;
		tiv::eaa2mat(eaa,R); p.q.fromMatrix(R);
		_RobotPoseArray.push_back(p);
    */
	}

	~SharedData() {
		;
	}

	void setSavePath(QString str)
	{
		QMutexLocker locker(&_mutexSavePath);
		savePath = str;
	}

	QString getSavePath()
	{
		QMutexLocker locker(&_mutexSavePath);
		return savePath;
	}

	void setHandEyeSavePath(QString str)
	{
		QMutexLocker locker(&_mutexHandEyeSavePath);
		saveHandEyePath = str;
	}

	QString getHandEyeSavePath()
	{
		QMutexLocker locker(&_mutexHandEyeSavePath);
		return saveHandEyePath;
	}

	void setRectSavePath(QString str, int CAM)
	{
		QMutexLocker locker(&_mutexRectSavePath);
		if( CAM == dti::LEFT)
			saveRectPathL = str;
		else if(CAM == dti::RIGHT)
			saveRectPathR = str;
	}

	QString getRectSavePath(int CAM)
	{
		QString qs;
		QMutexLocker locker(&_mutexRectSavePath);
		if( CAM == dti::LEFT)
			qs = saveRectPathL;
		else if(CAM == dti::RIGHT)
			qs = saveRectPathR;
		else
			//ERROR
			qs = "Error";

		return qs;
	}

	void clearIntrinsicGuessVals(int CAM)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);

		IntrinsicGuessVals* vals;
		switch (CAM) {
			case dti::LEFT:
				vals = &_intrinsicGuessValsL;
				break;
			case dti::RIGHT:
				vals = &_intrinsicGuessValsR;
				break;
			default:
				return;
				break;
		}

		vals->_focalLength[0] = 0.0;
		vals->_focalLength[1] = 0.0;

		vals->_principalPoint[0] = 0.0;
		vals->_principalPoint[1] = 0.0;

		vals->_radiadistortion[0] = 0.0;
		vals->_radiadistortion[1] = 0.0;
		vals->_radiadistortion[2] = 0.0;
		vals->_radiadistortion[3] = 0.0;
		vals->_radiadistortion[4] = 0.0;
		vals->_radiadistortion[5] = 0.0;

		vals->_tangentialDistortion[0] = 0.0;
		vals->_tangentialDistortion[1] = 0.0;

	}

	void setIntrinsicGuessVals(int CAM, cv::Mat intrinsic, cv::Mat distortion= cv::Mat::zeros(1,6,CV_64F))
	{
		switch (CAM) {
			case dti::LEFT:
				setIntrinsicGuessValsL(intrinsic, distortion);
			break;

			case dti::RIGHT:
				setIntrinsicGuessValsR(intrinsic, distortion);
				
			break;

			default:
				break;
		}
	}

	void setIntrinsicGuessValsL(cv::Mat intrinsic, cv::Mat distortion = cv::Mat::zeros(1,6,CV_64F))
	{
		int distSize = distortion.cols;
		QMutexLocker locker(&_mutexIntrinsicGuess);
		_intrinsicGuessValsL._focalLength[0] 	= intrinsic.at<double>(0,0);
		_intrinsicGuessValsL._focalLength[1] 	= intrinsic.at<double>(1,1);
		_intrinsicGuessValsL._principalPoint[0] = intrinsic.at<double>(0,2);
		_intrinsicGuessValsL._principalPoint[1] = intrinsic.at<double>(1,2);
		_intrinsicGuessValsL._radiadistortion[0]			 	= distortion.at<double>(0);
		_intrinsicGuessValsL._radiadistortion[1]			 	= distortion.at<double>(1);
		_intrinsicGuessValsL._tangentialDistortion[0]		 	= distortion.at<double>(2);
		_intrinsicGuessValsL._tangentialDistortion[1]		 	= distortion.at<double>(3);
		_intrinsicGuessValsL._radiadistortion[2]			 	= (distSize > 4) ? distortion.at<double>(4) : 0.0;
		_intrinsicGuessValsL._radiadistortion[3]			 	= (distSize > 5) ? distortion.at<double>(5) : 0.0;
		_intrinsicGuessValsL._radiadistortion[4]			 	= (distSize > 6) ? distortion.at<double>(6) : 0.0;
		_intrinsicGuessValsL._radiadistortion[5]			 	= (distSize > 7) ? distortion.at<double>(7) : 0.0;

	}

	void setIntrinsicGuessValsR(cv::Mat intrinsic, cv::Mat distortion = cv::Mat::zeros(1,6,CV_64F))
	{
		int distSize = distortion.cols;
		QMutexLocker locker(&_mutexIntrinsicGuess);
		_intrinsicGuessValsR._focalLength[0] 					= intrinsic.at<double>(0,0);
		_intrinsicGuessValsR._focalLength[1] 					= intrinsic.at<double>(1,1);
		_intrinsicGuessValsR._principalPoint[0] 				= intrinsic.at<double>(0,2);
		_intrinsicGuessValsR._principalPoint[1] 				= intrinsic.at<double>(1,2);
		_intrinsicGuessValsR._radiadistortion[0]			 	= distortion.at<double>(0);
		_intrinsicGuessValsR._radiadistortion[1]			 	= distortion.at<double>(1);
		_intrinsicGuessValsR._tangentialDistortion[0]		 	= distortion.at<double>(2);
		_intrinsicGuessValsR._tangentialDistortion[1]		 	= distortion.at<double>(3);
		_intrinsicGuessValsR._radiadistortion[2]			 	= (distSize > 4) ? distortion.at<double>(4) : 0.0;
		_intrinsicGuessValsR._radiadistortion[3]			 	= (distSize > 5) ? distortion.at<double>(5) : 0.0;
		_intrinsicGuessValsR._radiadistortion[4]			 	= (distSize > 6) ? distortion.at<double>(6) : 0.0;
		_intrinsicGuessValsR._radiadistortion[5]			 	= (distSize > 7) ? distortion.at<double>(7) : 0.0;
	}

	void setIntrinsicGuessValsL(IntrinsicGuessVals vals)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		_intrinsicGuessValsL = vals;
	}

	void setIntrinsicGuessValsR(IntrinsicGuessVals vals)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		_intrinsicGuessValsR = vals;
	}

	void getIntrinsicGuessMatL(cv::Mat* guess)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		guess->at<double>(0,0) = _intrinsicGuessValsL._focalLength[0] 	;
		guess->at<double>(1,1) = _intrinsicGuessValsL._focalLength[1] 	;
		guess->at<double>(0,2) = _intrinsicGuessValsL._principalPoint[0] ;
		guess->at<double>(1,2) = _intrinsicGuessValsL._principalPoint[1] ;
	}

	void getIntrinsicGuessMatR(cv::Mat* guess)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		guess->at<double>(0,0) = _intrinsicGuessValsR._focalLength[0] 	;
		guess->at<double>(1,1) = _intrinsicGuessValsR._focalLength[1] 	;
		guess->at<double>(0,2) = _intrinsicGuessValsR._principalPoint[0] ;
		guess->at<double>(1,2) = _intrinsicGuessValsR._principalPoint[1] ;
	}


	void getDistortionGuessMatL(cv::Mat* distortion)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		int distSize = distortion->cols;
		distortion->at<double>(0) = _intrinsicGuessValsL._radiadistortion[0] 	;
		distortion->at<double>(1) = _intrinsicGuessValsL._radiadistortion[1] 	;
		distortion->at<double>(2) = _intrinsicGuessValsL._tangentialDistortion[0] ;
		distortion->at<double>(3) = _intrinsicGuessValsL._tangentialDistortion[1] ;


		if(distSize>4)
		distortion->at<double>(4) = _intrinsicGuessValsL._radiadistortion[2];
		if(distSize>5)
		distortion->at<double>(5) = _intrinsicGuessValsL._radiadistortion[3];
		if(distSize>6)
		distortion->at<double>(6) = _intrinsicGuessValsL._radiadistortion[4];
		if(distSize>7)
		distortion->at<double>(7) = _intrinsicGuessValsL._radiadistortion[5];
	}

	void getDistortionGuessMatR(cv::Mat* distortion)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		int distSize = distortion->cols;
		distortion->at<double>(0) = _intrinsicGuessValsR._radiadistortion[0] 	;
		distortion->at<double>(1) = _intrinsicGuessValsR._radiadistortion[1] 	;
		distortion->at<double>(2) = _intrinsicGuessValsR._tangentialDistortion[0] ;
		distortion->at<double>(3) = _intrinsicGuessValsR._tangentialDistortion[1] ;


		if(distSize>4)
		distortion->at<double>(4) = _intrinsicGuessValsR._radiadistortion[2];
		if(distSize>5)
		distortion->at<double>(5) = _intrinsicGuessValsR._radiadistortion[3];
		if(distSize>6)
		distortion->at<double>(6) = _intrinsicGuessValsR._radiadistortion[4];
		if(distSize>7)
		distortion->at<double>(7) = _intrinsicGuessValsR._radiadistortion[5];
	}

	IntrinsicGuessVals getIntrinsicGuessValsR()
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		return _intrinsicGuessValsR;
	}

	IntrinsicGuessVals getIntrinsicGuessValsL()
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		return _intrinsicGuessValsL;
	}

	void setIntrinsicGuessPath(int CAM, QString path)
	{
		switch (CAM) {
			case LEFT: // or mono
				setIntrinsicGuessPathL(path);
				break;
			case RIGHT:
				setIntrinsicGuessPathR(path);
				break;
			default:
				// invalid CAM clearing all
				setIntrinsicGuessPathL("");
				setIntrinsicGuessPathR("");
				break;
		}
	}

	void setIntrinsicGuessPathL(QString path)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		_intrinsicGuessPathL = path;
	}

	void setIntrinsicGuessPathR(QString path)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		_intrinsicGuessPathR = path;
	}

	void setIntrinsicHandEyePath(QString path)
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
			_intrinsicHandEyePath = path;
	}

	void setRobotPosePath(QString path)
	{
		QMutexLocker locker(&_mutexRobotPath);
		_robotPath = path;
	}

	void setRobotPoseLivePath(QString path)
	{
		QMutexLocker locker(&_mutexRobotLivePath);
		_robotLivePath = path;
	}

	QString getIntrinsicGuessPathL()
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		return _intrinsicGuessPathL;
	}

	QString getIntrinsicGuessPathR()
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		return _intrinsicGuessPathR;
	}

	QString getIntrinsicHandEyePath()
	{
		QMutexLocker locker(&_mutexIntrinsicGuess);
		return _intrinsicHandEyePath;
	}

	QString getRobotPosePath()
	{
		QMutexLocker locker(&_mutexRobotPath);
		return _robotPath;
	}

	QString getRobotPoseLivePath()
	{
		QMutexLocker locker(&_mutexRobotLivePath);
		return _robotLivePath;
	}

	void setAnnotationRoi(bool val)
	{
		QMutexLocker locker(&_mutexAnnotateRoi);
		freshAnnotateRoi = val;
	}

	bool hasFreshAnnotation()
	{
		QMutexLocker locker(&_mutexAnnotateRoi);
		return freshAnnotateRoi;
	}

	cv::Rect getROI()
	{
		QMutexLocker locker(&_mutexAnnotateRoi);
		return _roi;
	}

	void setRoi(QRect roi)
	{
		QMutexLocker locker(&_mutexAnnotateRoi);
//		Adjust position such the TopLeft corner is infact the topleft corner
		if(roi.left()<roi.right())
			_roi.x = roi.left();
		else
			_roi.x=roi.right();
		_roi.width = std::abs((long int) roi.left() - roi.right());
		if (roi.top() < roi.bottom())
			_roi.y = roi.top();
		else
			_roi.y = roi.bottom();
		_roi.height = (int) std::abs((long int) roi.top() - roi.bottom());

		// cut of if negative
		if(_roi.x < 0) {
			_roi.width += _roi.x;
			_roi.x = 0;
		}

		if(_roi.y < 0) {
			_roi.height += _roi.y;
			_roi.y = 0;
		}

		locker.unlock();
		setAnnotationRoi(true);
	}

	void setRectFlags(dti::RectificationFlags rectFlags)
	{
		QMutexLocker locker(&_mutexStereoFlag);
		_rectFlags= rectFlags;
	}

	dti::RectificationFlags getRectFlags()
	{
		QMutexLocker locker(&_mutexStereoFlag);
		return _rectFlags;
	}

	void setStereoFlags(dti::StereoCalibFlags stereoFlags)
	{
		QMutexLocker locker(&_mutexStereoFlag);
		_stereoFlags= stereoFlags;
	}

	void setCornerFlags(dti::CornerDetectionFlags cornerFlags)
	{
		QMutexLocker locker(&_mutexCornerFlag);
		_cornerFlags = cornerFlags;
	}

	void setCornerFlagsHandEye(dti::CornerDetectionFlags cornerFlags)
	{
		QMutexLocker locker(&_mutexCornerFlag);
		_cornerFlagsHandEye = cornerFlags;
	}

	void setCalFlags(dti::CalibrationFlags calFlags)
	{
		QMutexLocker locker(&_mutexCalFlag);
		_calFlags = calFlags;
	}

	dti::CalibrationFlags getCalFlags()
	{
		QMutexLocker locker(&_mutexCalFlag);
		return _calFlags;
	}

	dti::StereoCalibFlags getStereoFlags()
	{
		QMutexLocker locker(&_mutexStereoFlag);
		return _stereoFlags;
	}

	dti::CornerDetectionFlags getCornerFlags()
	{
		QMutexLocker locker(&_mutexCornerFlag);
		return _cornerFlags;
	}

	dti::CornerDetectionFlags getCornerFlagsHandEye()
	{
		QMutexLocker locker(&_mutexCornerFlag);
		return _cornerFlagsHandEye;
	}

	tiv::pose getRobotPose()
	{
		QMutexLocker locker(&_mutexRobotPose);
		return _RobotPose;

	}

	std::vector<tiv::pose> getRobotPoseArray()
	{
		QMutexLocker locker(&_mutexRobotPoseArray);
		
		return _RobotPoseArray;

	}

	dti::RobotFlags getRobotFlags()
	{
		QMutexLocker locker(&_mutexRobotFlag);
		return _robotFlags;

	}

	void setRobotFlags(dti::RobotFlags flags)
	{
		QMutexLocker locker(&_mutexRobotFlag);
		_robotFlags = flags;
	}

	void addRobotPose2Array(tiv::pose pose)
	{
		QMutexLocker locker(&_mutexRobotPoseArray);
		_RobotPoseArray.push_back(pose);

	}

	void clearRobotPoseArray()
	{
		QMutexLocker locker(&_mutexRobotPoseArray);
		_RobotPoseArray.clear();

	}

	void setRobotPose(tiv::pose pose)
	{
		QMutexLocker locker(&_mutexRobotPose);
		_RobotPose =  pose;

	}

	void setCalType(CalibrationTypes type)
	{
		QMutexLocker locker(&_mutexCalType);
		_calType = type;
	}

	void setCalTypeHandEye(CalibrationTypes type)
	{
		QMutexLocker locker(&_mutexCalType);
		_calTypeHandEye = type;
		}

	void setBoardType(CalBoardTypes type)
	{
		QMutexLocker locker(&_mutexBoardType);
		_boardType = type;
	}

	void setBoardTypeHandEye(CalBoardTypes type)
	{
		QMutexLocker locker(&_mutexBoardType);
		_boardTypeHandEye = type;
	}

	void setBoardSize(int x, int y)	{
		QMutexLocker lock(&_mutexBoardSize);
		_boardSize.height = x;
		_boardSize.width = y;
	};

	void setBoardSizeHandEye(int x, int y)	{
		QMutexLocker lock(&_mutexBoardSize);
		_boardSizeHandEye.height = x;
		_boardSizeHandEye.width = y;
	};

	void setBoardSizeX(int val)	{
		QMutexLocker lock(&_mutexBoardSize);
		_boardSize.height = val;
	};

	void setBoardSizeXHandEye(int val)	{
		QMutexLocker lock(&_mutexBoardSize);
		_boardSizeHandEye.height = val;
	};

	void setBoardSizeY(int val)	{
		QMutexLocker lock(&_mutexBoardSize);
		_boardSize.width = val;
	};

	void setBoardSizeYHandEye(int val)	{
		QMutexLocker lock(&_mutexBoardSize);
		_boardSizeHandEye.width = val;
	};

	CalBoardTypes getBoardType()
	{
		QMutexLocker locker(&_mutexBoardType);
		return _boardType;
	}

	CalBoardTypes getBoardTypeHandEye()
	{
		QMutexLocker locker(&_mutexBoardType);
		return _boardTypeHandEye;
	}

	cv::Size getBoardSize() {
		QMutexLocker lock(&_mutexBoardSize);
		return _boardSize;
	}

	cv::Size getBoardSizeHandEye() {
		QMutexLocker lock(&_mutexBoardSize);
		return _boardSizeHandEye;
	}

	double getSquareSize() {
		QMutexLocker lock(&_mutexSquareSize);
		return _squareSize;
	}

	double getSquareSizeHandEye() {
		QMutexLocker lock(&_mutexSquareSize);
		return _squareSizeHandEye;
	}

	void setSquareSize(double square) {
		QMutexLocker lock(&_mutexSquareSize);
		_squareSize = square;
	}

	void setSquareSizeHandEye(double square) {
		QMutexLocker lock(&_mutexSquareSize);
		_squareSizeHandEye = square;
	}

	CalibrationTypes getCalType()
	{
		QMutexLocker locker(&_mutexCalType);
		return _calType;
	}

	CalibrationTypes getCalTypeHandEye()
	{
		QMutexLocker locker(&_mutexCalType);
		return _calTypeHandEye;
	}

	void setNameFilter(QStringList filter, CameraID cam)
	{
		QMutexLocker locker(&_mutexNameFilter[cam]);
		_nameFilter[cam].clear();
		_nameFilter[cam].append(filter);
	}

	QStringList getNameFilter(CameraID cam)
	{
		QMutexLocker locker(&_mutexNameFilter[cam]);
		return _nameFilter[cam];
	}

	void setImgPathL(QString path) {
		QMutexLocker locker(&_mutexPaths);
		_imgPathL.setPath(path);
	}

	void setImgPathHandEyeMono(QString path) {
			QMutexLocker locker(&_mutexPaths);
			_imgPathHandEyeMono.setPath(path);
		}

	void setImgPathR(QString path) {
		QMutexLocker locker(&_mutexPaths);
		_imgPathR.setPath(path);
	}

	void setIntrinsicResStorePathL(QString path)
	{
		QMutexLocker locker(&_mutexPaths);
		_intrinsicResStorePathL = path;
	}

	void setIntrinsicResStorePathR(QString path)
	{
		QMutexLocker locker(&_mutexPaths);
		_intrinsicResStorePathR = path;
	}

	QString getImgPathL() {
		QMutexLocker locker(&_mutexPaths);
		return _imgPathL.path();
	}

	QString getImgPathHandEyeMono() {
			QMutexLocker locker(&_mutexPaths);
			return _imgPathHandEyeMono.path();
		}

	QString getImgPathR() {
		QMutexLocker locker(&_mutexPaths);
		return _imgPathR.path();
	}
	QImage getImgL() {
		QMutexLocker locker(&_mutexImgL);
		return cvImg2QImg(&_imgL);
	}

	void setImgL(cv::Mat img) {
		QMutexLocker locker(&_mutexImgL);
		img.copyTo(_imgL);
	}

	QImage getImgR() {
		QMutexLocker locker(&_mutexImgR);
		return cvImg2QImg(&_imgR);
	}

	void setImgR(cv::Mat img) {
		QMutexLocker locker(&_mutexImgR);
		img.copyTo(_imgR);
	}

	QImage getImgHandEye() {
		QMutexLocker locker(&_mutexImgR);
		return cvImg2QImg(&_imgHandEye);
	}

	void setImgHandEye(cv::Mat img) {
		QMutexLocker locker(&_mutexImgR);
		img.copyTo(_imgHandEye);
	}

	static QImage cvImg2QImg(cv::Mat *imgOCV) {
		QImage *imgQ;
		if (imgOCV->type() == CV_8UC1 )
			imgQ = new QImage(imgOCV->data, imgOCV->cols, imgOCV->rows,
					imgOCV->step, QImage::Format_Indexed8);
		else if (imgOCV->type() == CV_8UC3)
			imgQ = new QImage(imgOCV->data, imgOCV->cols, imgOCV->rows,
					imgOCV->step, QImage::Format_RGB888);
		else if (imgOCV->type() == CV_16UC3 )
			imgQ = new QImage(imgOCV->data, imgOCV->cols, imgOCV->rows,
					imgOCV->step, QImage::Format_RGB16);

		return imgQ->copy(0,0,imgQ->width(), imgQ->height());
	}

	void setStop(bool stop)
	{
		QMutexLocker locker(&_mutexShoulsStop);
		_shouldStop = stop;
	}

	bool shouldStop()
	{
		QMutexLocker locker(&_mutexShoulsStop);
		return _shouldStop;
	}

	void setNarrowIntrinsicLeftPath(QString path)
	{
		QMutexLocker locker(&mutexNarrowParameters);
		narrowIntrinsicLeftPath = path;
	}

	QString getNarrowIntrinsicLeftPath()
	{
		QMutexLocker locker(&mutexNarrowParameters);
		return narrowIntrinsicLeftPath;
	}

	void setNarrowIntrinsicRightPath(QString path)
	{
		QMutexLocker locker(&mutexNarrowParameters);
		narrowIntrinsicRightPath = path;
	}

	QString getNarrowIntrinsicRightPath()
	{
		QMutexLocker locker(&mutexNarrowParameters);
		return narrowIntrinsicRightPath;
	}

	void setNarrowImageLeftPath(QString path)
	{
		QMutexLocker locker(&mutexNarrowParameters);
		narrowImageLeftPath = path;
	}

	QString getNarrowImageLeftPath()
	{
		QMutexLocker locker(&mutexNarrowParameters);
		return narrowImageLeftPath;
	}

	void setNarrowImageRightPath(QString path)
	{
		QMutexLocker locker(&mutexNarrowParameters);
		narrowImageRightPath = path;
	}

	QString getNarrowImageRightPath()
	{
		QMutexLocker locker(&mutexNarrowParameters);
		return narrowImageRightPath;
	}

	void setNarrowBoardSize(int x, int y)	{
		QMutexLocker lock(&mutexNarrowParameters);
		narrowBoardSize.height = x;
		narrowBoardSize.width = y;
	};

	void setNarrowBoardSizeX(int val)	{
		QMutexLocker lock(&mutexNarrowParameters);
		narrowBoardSize.height = val;
	};

	void setNarrowBoardSizeY(int val)	{
		QMutexLocker lock(&mutexNarrowParameters);
		narrowBoardSize.width = val;
	};

	cv::Size getNarrowBoardSize() {
		QMutexLocker lock(&mutexNarrowParameters);
		return narrowBoardSize;
	}

	void setNarrowOutputLeftPath(QString path)
	{
		QMutexLocker locker(&mutexNarrowParameters);
		narrowOutputLeftPath = path;
	}

	QString getNarrowOutputLeftPath()
	{
		QMutexLocker locker(&mutexNarrowParameters);
		return narrowOutputLeftPath;
	}

	void setNarrowOutputRightPath(QString path)
	{
		QMutexLocker locker(&mutexNarrowParameters);
		narrowOutputRightPath = path;
	}

	QString getNarrowOutputRightPath()
	{
		QMutexLocker locker(&mutexNarrowParameters);
		return narrowOutputRightPath;
	}

	void setCameraNameRight(QString name)
	{
		QMutexLocker locker(&mutexrightROSCameraName);
		rightROSCameraName = name;
	}

	QString getCameraNameRight()
	{
		QMutexLocker locker(&mutexrightROSCameraName);
		return rightROSCameraName;
	}

	void setCameraNameLeft(QString name)
	{
		QMutexLocker locker(&mutexleftROSCameraName);
		leftROSCameraName = name;
	}

	QString getCameraNameLeft()
	{
		QMutexLocker locker(&mutexleftROSCameraName);
		return leftROSCameraName;
	}

};

} /* namespace dti */
#endif /* SHAREDDATA_H_ */
