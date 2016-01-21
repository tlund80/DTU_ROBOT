/*
 * config.h
 *
 *  Created on: Dec 6, 2011
 *      Author: RAHA
 */

#ifndef CONFIG_H_
#define CONFIG_H_

namespace dti {

#define CALTI_MAX_CAMERA_NUM 3 // left and right and Hand
#define CALTI_CAMS CALTI_MAX_CAMERA_NUM
#define DAFAULT_EPSILON 0.001
#define DAFAULT_ITERATION 30
#define CAMERA_MATRIX_NAME		"camera_matrix"
#define DISTORTION_COEFF_NAME 	"distortion_coefficients"


typedef enum RectAlg {
	RE_Bougets = 0,
	RE_Hartleys
} RectAlg;

typedef enum RectFMatAlg {
	RE_FM_Points_7 = 0,
	RE_FM_Points_8,
	RE_FM_LeastMedian,
	RE_FM_RANSAC
} RectFMatAlg;

typedef enum AnnotationType {
	AutoThenManual = 0,
	OnlyManual,
	OnlyAuto
} AnnotationType;

enum RotationType {
	Euler_RPY = 0,
	Euler_YPR,
	Quaternion,
    Axis_Angle
};

typedef struct IntrinsicGuessVals
{
	double _principalPoint[2];// = {0.0,0.0};
	double _focalLength[2];//	 = {0.0,0.0};
	double _radiadistortion[6];//				 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double _tangentialDistortion[2];
} IntrinsicGuessVals;

typedef struct CornerDetectionFlags
{
	// gui flags..
	bool 		_displayCorners; 	// = false;//true;

	// Corner flags
	bool		_useAdaptiveThres;
	bool		_normalizeImage;
	bool		_filterQuads;
	bool 		_doPreCornerCheck;

	// annotation Flag
	AnnotationType	_cornerAnnotation;
	// Iterative termination flags
	int 		_stopOnIterations;
	double 		_stopOnEpsilon;

} CornerDetectionFlags;

typedef struct CalibrationFlags{
	// Calib flags
	bool 		_useIntrinsicGuess;
	bool		_fixAspectRatio;
	bool		_fixPrincipalPoint;
	bool		_zeroTangentDistortion;
	bool		_fixFocalLength;
	bool		_fix_K1;
	bool		_fix_K2;
	bool		_fix_K3;
	bool		_fix_K4;
	bool		_fix_K5;
	bool		_fix_K6;
	bool		_useRationalModel;
} CalibrationFlags;

typedef struct RectificationFlags{ // also used when undistortion from mono setup
	// for stereo rectification
	RectAlg		_rectAlg; 				// Chose algorithm
	bool		_zeroRectifyDisparity;	// CALIB_ZERO_DISPARITY = CV_CALIB_ZERO_DISPARITY
	bool		_saveRectificationImg;	// save the rectified imgs
	bool		_displayRectification;  // Display the rectification results?
	bool		_displayValidRect;
	double		_alpha;					// Scaling parameter!

	RectFMatAlg _fundamentalAlg;		// If using Harley's method the fundamental matrix is estimated using this alg
	double		_param1;				// Parameters depending on _fundamentalAlg
	double		_param2;				// ============ | |  ============

} RectificationFlags;

typedef struct 	StereoCalibFlags{

	// Iterative termination flags
	int 		_stopOnIterations;
	double 		_stopOnEpsilon;

	// Stereocalib Flags
	bool		_fixIntrinsicStereo;
	bool		_sameFocalLengthStereo;

} StereoCalibFlags;

typedef struct 	RobotPose{

	double		x;
	double		y;
	double		z;
	double		roll;
	double		pitch;
	double		yaw;


} RobotPose;

typedef struct 	RobotFlags{

	dti::RotationType		_type;
	bool					_displayRobotPose;
	bool					_isInverted;
	bool					_SaveRobotPose;
} RobotFlags;

enum CalibrationTypes{
	StereoCalibration_offline = 0,
	MonoCalibration_offline,
	StereoCalibration_online,
	MonoCalibration_online,
	HandEyeCalibration_offline,
	HandEyeCalibration_online,
};

enum CalBoardTypes{
	CHECKERBOARD = 0,
	SYMMETRIC_CIRCLES_GRID,
	ASYMMETRIC_CIRCLES_GRID,
	AR_MARKER
};

enum PreviewSizeValues {
	w320xh240 = 0,
	w640xh480,
	custom_resolution
};

enum CameraID {
	LEFT = 0,
	RIGHT,
	HAND,
	MONO = LEFT,

};

enum TabID {
	TAB_CAMERA = 0,
	TAB_HANDEYE,
};


typedef enum StereoState
{
	SS_init = 0,
	SS_detectCorners,
	SS_waitForGUI,
	SS_calibrate,
	SS_rectify,
	SS_doAll,
	SS_saveResults,
	SS_final,
	SS_clearData
} StereoState;

typedef enum MonoState
{
	MS_init = 0,
	MS_detectCorners,
	MS_calibrate,
	MS_undist,
	MS_doAll,
	MS_saveResults,
	MS_final,
	MS_waitForGUI,
	MS_clearData
} MonoState;

typedef enum HandEyeState
{
	HE_init = 0,
	HE_detectCorners,
	HE_computePose,
	HE_calibrate,
	HE_doAll,
	HE_saveResults,
	HE_final,
	HE_waitForGUI,
	HE_clearData
} HandEyeState;

typedef enum VerboseLevel {
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL
}VerboseLevel;

}  // namespace dti


#endif /* CONFIG_H_ */
