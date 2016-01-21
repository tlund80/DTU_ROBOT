/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <QMessageBox>
#include <iostream>
#include "../../include/cal_ti/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace CalTI {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
#ifdef USE_ROS
	, qnode(argc,argv)
#endif
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    _annotationDialog = NULL;
	_sharedData 	= new dti::SharedData();
	_calThread 		= new dti::CalibThread(_sharedData);
	_calibMono 		= _calThread->getMonoCalibration();
	_calibStereo 	= _calThread->getStereoCalibration();
	_calibHandEye   = _calThread->getHandEyeCalibration();

	init();
	readSettings();
	_calThread->activate();
}

MainWindow::~MainWindow() {}

void MainWindow::initGui()
{
	// some bug on the gui requires to disable and then enable!
	ui._itc_epsilon_Corner_LineEdit->setEnabled(true);
	ui._itc_max_Iter_Corner_LineEdit_->setEnabled(true);
	ui._itc_epsilon_Stereo_LineEdit->setEnabled(true);
	ui._itc_max_Iter_Stereo_LineEdit->setEnabled(true);

	/** 	PREVIEW RESIZE 			**/
	// set combo box
	ui.comboPreviewSize->clear();
	ui.comboPreviewSize->insertItem(dti::w320xh240, QString("320x240"));
	ui.comboPreviewSize->insertItem(dti::w640xh480, QString("640x480"));
	ui.comboPreviewSize->insertItem(dti::custom_resolution, QString("Custom resolution"));

	ui.comboPreviewSize_2->clear();
	ui.comboPreviewSize_2->insertItem(dti::w320xh240, QString("320x240"));
	ui.comboPreviewSize_2->insertItem(dti::w640xh480, QString("640x480"));
	ui.comboPreviewSize_2->insertItem(dti::custom_resolution, QString("Custom resolution"));

	// set the edit fields
	QValidator *validator = new QIntValidator(1, 100000,this);
	ui.lineEditPreviewHeight->setValidator(validator);
	ui.lineEditPreviewWidth->setValidator(validator);

	// set the edit fields
	QValidator *validator_2 = new QIntValidator(1, 100000,this);
	ui.lineEditPreviewHeight_2->setValidator(validator_2);
	ui.lineEditPreviewWidth_2->setValidator(validator_2);

	/** 	CALIBRATION TYPE		**/
	ui.comboSelectCalibType->clear();
	ui.comboSelectCalibType->insertItem(dti::StereoCalibration_offline, QString("Stereo calibration - Offline"));
	ui.comboSelectCalibType->insertItem(dti::StereoCalibration_online, 	QString("Stereo calibration - Online"));
	ui.comboSelectCalibType->insertItem(dti::MonoCalibration_offline, 	QString("Mono calibration - Offline"));
	ui.comboSelectCalibType->insertItem(dti::MonoCalibration_online, 	QString("Mono calibration - Online"));

	ui.comboSelectCalibType_2->clear();
	ui.comboSelectCalibType_2->insertItem(dti::HandEyeCalibration_offline, 	QString("Hand/Eye calibration - Offline"));
	ui.comboSelectCalibType_2->insertItem(dti::HandEyeCalibration_online, 	QString("Hand/Eye calibration - Online"));

	/** 	CALIBRATION BOARD TYPE		**/
	ui.comboBoardType->clear();
	ui.comboBoardType->insertItem(dti::CHECKERBOARD, QString("Black-white checkerboard"));
	ui.comboBoardType->insertItem(dti::SYMMETRIC_CIRCLES_GRID, QString("Symmetrical circle grid"));
	ui.comboBoardType->insertItem(dti::ASYMMETRIC_CIRCLES_GRID,	QString("Asymmetrical circle grid"));

	ui.comboBoardType_2->clear();
	ui.comboBoardType_2->insertItem(dti::CHECKERBOARD, QString("Black-white checkerboard"));
	ui.comboBoardType_2->insertItem(dti::SYMMETRIC_CIRCLES_GRID, QString("Symmetrical circle grid"));
	ui.comboBoardType_2->insertItem(dti::ASYMMETRIC_CIRCLES_GRID,	QString("Asymmetrical circle grid"));
	ui.comboBoardType_2->insertItem(dti::AR_MARKER,	QString("Rectangular AR Marker"));

	/** 	MARKER ANNOTATION TYPE CAMERA CALIBRATION		**/
	ui._annotationType_Combo->clear();
	ui._annotationType_Combo->insertItem(dti::AutoThenManual, 	QString("Auto then manual"));
	ui._annotationType_Combo->insertItem(dti::OnlyAuto, 		QString("Only auto"));
	ui._annotationType_Combo->insertItem(dti::OnlyManual, 		QString("All manual"));

	/** 	MARKER ANNOTATION TYPE HAND EYE		**/
	ui._annotationType_Combo_3->clear();
	ui._annotationType_Combo_3->insertItem(dti::AutoThenManual, 	QString("Auto then manual"));
	ui._annotationType_Combo_3->insertItem(dti::OnlyAuto, 		QString("Only auto"));
	ui._annotationType_Combo_3->insertItem(dti::OnlyManual, 		QString("All manual"));

	/** 	Rectification settings 	**/
	ui._rectAlgorithm_Combo->clear();
	ui._rectAlgorithm_Combo->insertItem(dti::RE_Bougets, 	QString("Bouget's Method"));
	ui._rectAlgorithm_Combo->insertItem(dti::RE_Hartleys, 	QString("Hartley's Method"));

	ui._rectFundamental_Combo->clear();
	ui._rectFundamental_Combo->insertItem(dti::RE_FM_Points_7, 	 QString("7 Points Method"));
	ui._rectFundamental_Combo->insertItem(dti::RE_FM_Points_8, 	 QString("8 Points Method"));
	ui._rectFundamental_Combo->insertItem(dti::RE_FM_LeastMedian, QString("Least Median Method"));
	ui._rectFundamental_Combo->insertItem(dti::RE_FM_RANSAC, 	 QString("RANSAC Method"));

	/**		Robot pose representation	**/
	ui.comboAngleType->clear();
	ui.comboAngleType->insertItem(dti::Euler_RPY, 	QString("Euler RPY"));
	ui.comboAngleType->insertItem(dti::Euler_YPR, 	QString("Euler YRP"));
	ui.comboAngleType->insertItem(dti::Quaternion, 	QString("Quaternions"));
	ui.comboAngleType->insertItem(dti::Axis_Angle, 	QString("Axis Angle"));

	/** ITERATIVE TERM CRIT **/
	// set the edit fields
	QValidator *validatorI = new QIntValidator(0, 100000,this);
	ui._itc_max_Iter_Corner_LineEdit_->setValidator(validatorI);
	ui._itc_max_Iter_Stereo_LineEdit->setValidator(validatorI);
	QValidator *validatorD = new QDoubleValidator(0, 10, 25, this);
	ui._itc_epsilon_Corner_LineEdit->setValidator(validatorD);
	ui._itc_epsilon_Stereo_LineEdit->setValidator(validatorD);

	ui._itc_max_Iter_Stereo_LineEdit->setValidator(validator);
	ui.lineEditPreviewWidth->setValidator(validator);

	/** set validators for left and right principalpoint and focal length **/
	delete validatorD;
	validatorD = new QDoubleValidator(0, 10000, 25, this);
	ui._fixFocalLength_VALx_CL->setValidator(validatorD);
	ui._fixFocalLength_VALy_CL->setValidator(validatorD);
	ui._fixFocalLength_VALx_CR->setValidator(validatorD);
	ui._fixFocalLength_VALy_CR->setValidator(validatorD);

	ui._fixPrincipalPoint_VALx_CL->setValidator(validatorD);
	ui._fixPrincipalPoint_VALy_CL->setValidator(validatorD);
	ui._fixPrincipalPoint_VALx_CR->setValidator(validatorD);
	ui._fixPrincipalPoint_VALy_CR->setValidator(validatorD);

	// set all KX to use validator
	ui._fix_K1_VAL_CL->setValidator(validatorD);
	ui._fix_K2_VAL_CL->setValidator(validatorD);
	ui._fix_K3_VAL_CL->setValidator(validatorD);
	ui._fix_K4_VAL_CL->setValidator(validatorD);
	ui._fix_K5_VAL_CL->setValidator(validatorD);
	ui._fix_K6_VAL_CL->setValidator(validatorD);
	ui._fix_K1_VAL_CR->setValidator(validatorD);
	ui._fix_K2_VAL_CR->setValidator(validatorD);
	ui._fix_K3_VAL_CR->setValidator(validatorD);
	ui._fix_K4_VAL_CR->setValidator(validatorD);
	ui._fix_K5_VAL_CR->setValidator(validatorD);
	ui._fix_K6_VAL_CR->setValidator(validatorD);

	// set all P-vals
	ui._fix_P1_VAL_CR->setValidator(validatorD);
	ui._fix_P2_VAL_CR->setValidator(validatorD);
	ui._fix_P1_VAL_CL->setValidator(validatorD);
	ui._fix_P2_VAL_CL->setValidator(validatorD);

	/** 	update intrinsic file paths to trigger signals	**/
	ui._intrinsicGuessCam1Path->setText(ui._intrinsicGuessCam1Path->text());
	ui._intrinsicGuessCam2Path->setText(ui._intrinsicGuessCam2Path->text());
	ui._rectificationSavePathTxtL->setText(ui.imgPathL->text() + "rectified");
	ui._rectificationSavePathTxtR->setText(ui.imgPathR->text() + "rectified");
	/** Rectification values are set 	**/
	//set validators
	ui._rectParam1_VAL->setValidator(validatorD);
	ui._rectParam2_VAL->setValidator(validatorD);
	double val = 3.0;
	ui._rectParam1_VAL->setText(QString::number(val, 'g', 6));
	val = 0.99;
	ui._rectParam2_VAL->setText(QString::number(val, 'g', 6));

	ui._itc_IterativeTreminationFlags_Corner_BOX->setEnabled(true);
	ui._itc_IterativeTreminationFlags_Stereo_BOX->setEnabled(true);

	ui._useIntrinsicGuess_CB->setChecked(false);

	//Always camera calibration tab in front
	ui.tabWidget_2->setCurrentIndex(0);
	tab = dti::TAB_CAMERA;

    //Set maximum size to screen resolution
    QRect res = QApplication::desktop()->availableGeometry( QApplication::desktop()->primaryScreen());
    this->setMaximumSize(res.width(),res.height());
    this->setMinimumSize(res.width()*0.2,res.height()*0.2);

#ifndef USE_ROS
    ui.pushButtonStreamStart->setEnabled(false);
    ui.pushButtonStreamSaveImage->setEnabled(false);
    ui.pushButtonStreamStop->setEnabled(false);
#endif

}

void MainWindow::init()
{
	initGui();

#ifdef USE_ROS
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    connect(&qnode, SIGNAL(consoleSignal(QString, dti::VerboseLevel)), this, SLOT(consoleOut(QString, dti::VerboseLevel)), Qt::UniqueConnection);
	//connect(&qnode, SIGNAL(updateImageLeft(const QImage&)), this, SLOT(updateImageL(const QImage&)));
	//connect(&qnode, SIGNAL(updateImageRight(const QImage&)), this, SLOT(updateImageR(const QImage&)));
	connect(&qnode, SIGNAL(updateImage(const QImage&, int)), this, SLOT(updateImage(const QImage&, int)));
	connect(&qnode, SIGNAL(updateTCP(const tiv::pose&, int)), this, SLOT(updateRobotPose(const tiv::pose&, int)));
#endif

	onlineImages = false;
	saveNextImage[0] = false;
	saveNextImage[1] = false;
	saveNextImage[2] = false;
	imageFolderEmpty = true;

    qRegisterMetaType<dti::VerboseLevel>("dti::VerboseLevel");

	// Connect main thread to console
    connect(this, SIGNAL(consoleOutSig(QString,dti::VerboseLevel)), this, SLOT(consoleOut(QString, dti::VerboseLevel)), Qt::UniqueConnection );
	/*****************************************************************************
	 *****************************  Connect GUI signals 	 *********************
	 *****************************************************************************/
	//QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

	// collapse flag box's
	connect(ui._CalFlags_groupBox,SIGNAL(toggled(bool)), this, SLOT(showFlagsGroupBox(bool)), Qt::UniqueConnection);
	connect(ui.groupBox_StereoCalFlags,SIGNAL(toggled(bool)), this, SLOT(showFlagsGroupBox(bool)), Qt::UniqueConnection);
	connect(ui.groupBox_inspectIntrinsic,SIGNAL(toggled(bool)), this, SLOT(showFlagsGroupBox(bool)), Qt::UniqueConnection);

	// Calibration board selection
	connect(ui.comboBoardType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateBoardType(int)), Qt::UniqueConnection);
	connect(ui.spinBoxBoardSizeX, SIGNAL(valueChanged(int)), this, SLOT(updateBoardSize(int)), Qt::UniqueConnection);
	connect(ui.spinBoxBoardSizeY, SIGNAL(valueChanged(int)), this, SLOT(updateBoardSize(int)), Qt::UniqueConnection);
	connect(ui._squarePhysicalSize_SPINB, SIGNAL(editingFinished()), this, SLOT(updateSquareSize()), Qt::UniqueConnection);

	connect(ui.comboBoardType_2, SIGNAL(currentIndexChanged(int)), this, SLOT(updateBoardType(int)), Qt::UniqueConnection);
	connect(ui.spinBoxBoardSizeX_2, SIGNAL(valueChanged(int)), this, SLOT(updateBoardSize(int)), Qt::UniqueConnection);
	connect(ui.spinBoxBoardSizeY_2, SIGNAL(valueChanged(int)), this, SLOT(updateBoardSize(int)), Qt::UniqueConnection);
	connect(ui._squarePhysicalSize_SPINB_2, SIGNAL(editingFinished()), this, SLOT(updateSquareSize()), Qt::UniqueConnection);

	// Connect calibration type combobox
	connect(ui.comboSelectCalibType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCalType(int)), Qt::UniqueConnection);
	connect(ui.comboSelectCalibType_2, SIGNAL(currentIndexChanged(int)), this, SLOT(updateHandEyeCalType(int)), Qt::UniqueConnection);
	connect(ui.tabWidget_2, SIGNAL(currentChanged(int)),this, SLOT(updateTab(int)), Qt::UniqueConnection);
	// Connect path_select btns
	connect(ui.btnFileBrowseL, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui.btnFileBrowseL_2, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui.btnFileBrowseR, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui._RobotPathbtnFileBrowse, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui._RobotSavePathbtnFileBrowse_3, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui._intrinsicGuessCam1btnFileBrowse, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui._intrinsicGuessCam2btnFileBrowse, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);

	connect(ui._intrinsicGuessCam1btnFileBrowse_2, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);

	connect(ui._rectificationSavePathBtnL, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui._rectificationSavePathBtnR, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui.btnFileBrowserNSIntrinsicLeft, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui.btnFileBrowserNSIntrinsicRight, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui.btnFileBrowserNSLeftImage, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);
	connect(ui.btnFileBrowserNSRightImage, SIGNAL(pressed()), this, SLOT(selectPath()), Qt::UniqueConnection);

	/** CONNECT INTRINSIC GUESS VALUES **/
//	connect(ui._intrinsicGuessCam1Path, SIGNAL(editingFinished()), this, SLOT(updateIntrinsicGuessPath()), Qt::UniqueConnection);
//	connect(ui._intrinsicGuessCam2Path, SIGNAL(editingFinished()), this, SLOT(updateIntrinsicGuessPath()), Qt::UniqueConnection);
	connect(ui._useIntrinsicGuess_CB, SIGNAL(toggled(bool)), this, SLOT(updateUseIntrinsic(bool)), Qt::UniqueConnection);

	/** connect intrinsic file read btn	**/
	connect(ui._intrinsicReadFromFileBtnL, SIGNAL(released()), this, SLOT(updateIntrinsicGuessPath()), Qt::UniqueConnection);
	connect(ui._intrinsicReadFromFileBtnR, SIGNAL(released()), this, SLOT(updateIntrinsicGuessPath()), Qt::UniqueConnection);

	/** connect intrinsic clear btns **/
	connect(ui._intrinsicClearValsBtnL, SIGNAL(released()), this, SLOT(clearIntrinsicValsSlot()), Qt::UniqueConnection);
	connect(ui._intrinsicClearValsBtnR, SIGNAL(released()), this, SLOT(clearIntrinsicValsSlot()), Qt::UniqueConnection);

	/** connect calibration flags for both cams **/
	connect(ui._fixAspectRatio_CB_CL, SIGNAL(toggled(bool)), ui._fixAspectRatio_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fixAspectRatio_CB_CR, SIGNAL(toggled(bool)), ui._fixAspectRatio_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	connect(ui._fixPrincipalPoint_CB_CL, SIGNAL(toggled(bool)), ui._fixPrincipalPoint_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fixPrincipalPoint_CB_CR, SIGNAL(toggled(bool)), ui._fixPrincipalPoint_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	connect(ui._fixZeroTangentDistortion_CB_CL, SIGNAL(toggled(bool)), ui._fixZeroTangentDistortion_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fixZeroTangentDistortion_CB_CR, SIGNAL(toggled(bool)), ui._fixZeroTangentDistortion_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	connect(ui._fixZeroTangentDistortion_CB_CR, SIGNAL(toggled(bool)), this, SLOT(updateTangentDistortion(bool)), Qt::UniqueConnection);

	connect(ui._fixFocalLength_CB_CL, SIGNAL(toggled(bool)), ui._fixFocalLength_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fixFocalLength_CB_CR, SIGNAL(toggled(bool)), ui._fixFocalLength_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	connect(ui._fix_K1_CB_CL, SIGNAL(toggled(bool)), ui._fix_K1_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fix_K1_CB_CR, SIGNAL(toggled(bool)), ui._fix_K1_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	connect(ui._fix_K2_CB_CL, SIGNAL(toggled(bool)), ui._fix_K2_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fix_K2_CB_CR, SIGNAL(toggled(bool)), ui._fix_K2_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	connect(ui._fix_K3_CB_CL, SIGNAL(toggled(bool)), ui._fix_K3_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fix_K3_CB_CR, SIGNAL(toggled(bool)), ui._fix_K3_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	connect(ui._fix_K4_CB_CL, SIGNAL(toggled(bool)), ui._fix_K4_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fix_K4_CB_CR, SIGNAL(toggled(bool)), ui._fix_K4_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	connect(ui._fix_K5_CB_CL, SIGNAL(toggled(bool)), ui._fix_K5_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fix_K6_CB_CL, SIGNAL(toggled(bool)), ui._fix_K6_CB_CR, SLOT(setChecked(bool)), Qt::UniqueConnection);
	connect(ui._fix_K6_CB_CR, SIGNAL(toggled(bool)), ui._fix_K6_CB_CL, SLOT(setChecked(bool)), Qt::UniqueConnection);

	// skal sikre at den sti der er tastet i textfeltet ogs\E5 blive gemt i vars i CalibThread!!1
	connect(ui.imgPathL, SIGNAL(editingFinished()), this, SLOT(updatePaths()), Qt::UniqueConnection);
	connect(ui.imgPathR, SIGNAL(editingFinished()), this, SLOT(updatePaths()), Qt::UniqueConnection);
	connect(ui.imgPathL_2, SIGNAL(editingFinished()), this, SLOT(updatePaths()), Qt::UniqueConnection);
	connect(ui._RobotPosePath, SIGNAL(editingFinished()), this, SLOT(updatePaths()), Qt::UniqueConnection);
	connect(ui._RobotPoseSavePath, SIGNAL(editingFinished()), this, SLOT(updatePaths()), Qt::UniqueConnection);

	// Connect activateCalibration btn;
	connect(ui._doAllBtn, SIGNAL(pressed()), this, SLOT(processDoAllBtn()), Qt::UniqueConnection);
	connect(ui._doDetectCornersBtn, SIGNAL(pressed()), this, SLOT(processDoDetectCornersBtn()), Qt::UniqueConnection);
	connect(ui._doCalibrateBtn, SIGNAL(pressed()), this, SLOT(processDoCalibrateBtn()), Qt::UniqueConnection);
	connect(ui._doRectifyBtn, SIGNAL(pressed()), this, SLOT(processDoRectifyBtn()), Qt::UniqueConnection);
	connect(ui._doSaveResultsBtn, SIGNAL(pressed()), this, SLOT(processDoSaveBtn()), Qt::UniqueConnection);
	connect(ui._cancelBtn, SIGNAL(pressed()), this, SLOT(processDoCancelBtn()), Qt::UniqueConnection);
	connect(ui._clearConsoleBtn, SIGNAL(pressed()), this, SLOT(consoleClear()), Qt::UniqueConnection);
	connect(ui._clearAllData, SIGNAL(pressed()), this, SLOT(clearAllData()), Qt::UniqueConnection);
	connect(ui._commitROS, SIGNAL(pressed()), this, SLOT(commitROS_clicked()), Qt::UniqueConnection);

	//Hand eye signals
	connect(ui._doAllBtn_2, SIGNAL(pressed()), this, SLOT(processDoAllBtn()), Qt::UniqueConnection);
	connect(ui._doDetectCornersBtn_2, SIGNAL(pressed()), this, SLOT(processDoDetectCornersHandEyeBtn()), Qt::UniqueConnection);
	connect(ui._doCalibrateBtn_2, SIGNAL(pressed()), this, SLOT(processDoCalibrateHandEyeBtn()), Qt::UniqueConnection);
	connect(ui._doSaveResultsBtn_2, SIGNAL(pressed()), this, SLOT(processDoSaveHandEyeBtn()), Qt::UniqueConnection);
	connect(ui._cancelBtn_2, SIGNAL(pressed()), this, SLOT(processDoCancelBtn()), Qt::UniqueConnection);
	connect(ui._doComputeCamPoseBtn_2, SIGNAL(pressed()), this, SLOT(processDoComputeCameraPosesBtn()), Qt::UniqueConnection);
	connect(ui._clearAllData_2, SIGNAL(pressed()), this, SLOT(clearAllData()), Qt::UniqueConnection);
	connect(ui._commitROS_2, SIGNAL(pressed()), this, SLOT(commitHandEyeROS_clicked()), Qt::UniqueConnection);
	connect(ui._btnSaveRobotPoses, SIGNAL(pressed()), this, SLOT(processDoSaveRobotPoseBtn()), Qt::UniqueConnection);
	connect(ui._btnClearRobotPoses, SIGNAL(pressed()), this, SLOT(processDoClearRobotPoseBtn()), Qt::UniqueConnection);
	connect(ui._intrinsicReadFromFileBtnR_2, SIGNAL(pressed()), this, SLOT(processDoShowRobotPoseBtn()), Qt::UniqueConnection);
	


	// Connect previewsize
	connect(ui.comboPreviewSize, SIGNAL(currentIndexChanged(int)), this, SLOT(updatePreviewSizeFields(int)), Qt::UniqueConnection);
	connect(ui.comboPreviewSize, SIGNAL(highlighted(int)), this, SLOT(updatePreviewSizeFields(int)), Qt::UniqueConnection);
	connect(ui.lineEditPreviewHeight, SIGNAL(textChanged(QString)), this, SLOT(updatePreviewSizeHeight(QString)), Qt::UniqueConnection);
	connect(ui.lineEditPreviewWidth, SIGNAL(textChanged(QString)), this, SLOT(updatePreviewSizeWidth(QString)), Qt::UniqueConnection);

	connect(ui.comboPreviewSize_2, SIGNAL(currentIndexChanged(int)), this, SLOT(updatePreviewSizeFields(int)), Qt::UniqueConnection);
	connect(ui.comboPreviewSize_2, SIGNAL(highlighted(int)), this, SLOT(updatePreviewSizeFields(int)), Qt::UniqueConnection);
	connect(ui.lineEditPreviewHeight_2, SIGNAL(textChanged(QString)), this, SLOT(updatePreviewSizeHeight(QString)), Qt::UniqueConnection);
	connect(ui.lineEditPreviewWidth_2, SIGNAL(textChanged(QString)), this, SLOT(updatePreviewSizeWidth(QString)), Qt::UniqueConnection);

	// Connect
	connect(ui.lineEditNameFilterL, SIGNAL(textChanged(QString)), this, SLOT(updateNameFilterL(QString)), Qt::UniqueConnection);
	connect(ui.lineEditNameFilterR, SIGNAL(textChanged(QString)), this, SLOT(updateNameFilterR(QString)), Qt::UniqueConnection);
	connect(ui.lineEditNameFilterL_2, SIGNAL(textChanged(QString)), this, SLOT(updateNameFilterHandEye(QString)), Qt::UniqueConnection);

	/** connect the caliobration thread to the main thread (gui) **/
	// Connect CalibThread to the console
    connect(_calThread, SIGNAL(consoleSignal(QString, dti::VerboseLevel)), this, SLOT(consoleOut(QString, dti::VerboseLevel)), Qt::UniqueConnection);

	// Connect CalibThread to imgLabel
	connect(_calThread, SIGNAL(imgLSignal()), this, SLOT(updateImgL()), Qt::DirectConnection);
	connect(_calThread, SIGNAL(imgRSignal()), this, SLOT(updateImgR()), Qt::DirectConnection);

	connect(_calThread, SIGNAL(finishedSignal()), this, SLOT(calThreadFinished()), Qt::UniqueConnection);


	//connect "low-level" calibration, maybe not optimal solution..
	connect(_calThread, SIGNAL(updateGuiPreviewSignal()), this, SLOT(updateImgs()), Qt::UniqueConnection);
    connect(_calibMono, SIGNAL(consoleSignal(QString, dti::VerboseLevel)), this, SLOT(consoleOut(QString, dti::VerboseLevel)), Qt::UniqueConnection);
    connect(_calibStereo, SIGNAL(consoleSignal(QString, dti::VerboseLevel)), this, SLOT(consoleOut(QString, dti::VerboseLevel)), Qt::UniqueConnection);
    connect(_calibHandEye, SIGNAL(consoleSignal(QString, dti::VerboseLevel)), this, SLOT(consoleOut(QString, dti::VerboseLevel)), Qt::UniqueConnection);
	// connect the annotation signal fra low-level calibration to the thread
//	connect(_calib, SIGNAL(annotateGuiSignal(QImage)),
//			_calThread, SLOT(annotateImage(QImage)),
//			Qt::AutoConnection);
	// connect the annotation from the thread to the gui
	connect(_calThread, SIGNAL(annotateGuiSignal(QImage, QString)), this, SLOT(annotateRoiInImage(QImage, QString)), Qt::UniqueConnection);

	// connect rectification
	connect(ui._rectAlgorithm_Combo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateRectificationGUI(int)), Qt::UniqueConnection);
	connect(ui._rectFundamental_Combo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateRectificationGUI(int)), Qt::UniqueConnection);
	// connect the rectification custom alpha checkbox
	connect(ui._rectAlpha_CB, SIGNAL(toggled(bool)), ui._rectAlpha_VAL, SLOT(setEnabled(bool)), Qt::UniqueConnection);
	connect(ui._rectificationSavePathTxtL, SIGNAL(editingFinished()), this, SLOT(updateRectSavePath()), Qt::UniqueConnection);
	connect(ui._rectificationSavePathTxtR, SIGNAL(editingFinished()), this, SLOT(updateRectSavePath()), Qt::UniqueConnection);

		// connect termCrit
	connect(ui._itc_epsilon_Corner_checkBox_2, 	SIGNAL(toggled(bool)), 	this, SLOT(updateTermCrit()), Qt::UniqueConnection);
	connect(ui._itc_epsilon_Stereo_checkBox, 	SIGNAL(toggled(bool)), 	this, SLOT(updateTermCrit()), Qt::UniqueConnection);
	connect(ui._itc_iterations_Corner_checkBox, SIGNAL(toggled(bool)), 	this, SLOT(updateTermCrit()), Qt::UniqueConnection);
	connect(ui._itc_iterations_Stereo_checkBox, SIGNAL(toggled(bool)), 	this, SLOT(updateTermCrit()), Qt::UniqueConnection);

	/** Connect intrinsicGuessPath **/
	connect(this, SIGNAL(intrinsicGuessPath(dti::CameraID, QString, QString)), _calThread, SLOT(readIntrinsicGuessPath(dti::CameraID, QString, QString)), Qt::UniqueConnection);
	connect(_calThread, SIGNAL(updateGuiIntrinsicValsSignal()), this, SLOT(updateGuiIntrinsicVals()), Qt::UniqueConnection);

	connect(ui.imgPathL, SIGNAL(editingFinished()), this, SLOT(updateImageLists()), Qt::UniqueConnection);
	connect(ui.imgPathL_2, SIGNAL(editingFinished()), this, SLOT(updateImageLists()), Qt::UniqueConnection);

	ui._useIntrinsicGuess_CB->setChecked(true);
	ui._useIntrinsicGuess_CB->setChecked(false);
	_sharedData->setBoardSize(ui.spinBoxBoardSizeX->value(), ui.spinBoxBoardSizeY->value());
	ui._rectAlgorithm_Combo->setCurrentIndex(1);
	ui._rectAlgorithm_Combo->setCurrentIndex(0);
	ui._rectFundamental_Combo->setCurrentIndex(1);
	ui._rectFundamental_Combo->setCurrentIndex(0);
	ui._rectAlpha_CB->setChecked(true);
	ui._rectAlpha_CB->setChecked(false);
	ui.groupBox_inspectIntrinsic->setChecked(false);

	readFlags();

}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::readSettings()
{
    QSettings settings("DTI", "CalTI");
    QString img_path_L = settings.value("img_path_L").toString();
    QString img_path_L_2 = settings.value("img_path_L_2").toString();
    QString img_path_R = settings.value("img_path_R").toString();
    QString robotPosePath = settings.value("RobotPosePath").toString();
    ui.imgPathL->setText(img_path_L);
    ui.imgPathR->setText(img_path_R);
    ui.imgPathL_2->setText(img_path_L_2);
    ui._RobotPosePath->setText(robotPosePath);
	updatePaths();

    QString img_filter_L = settings.value("img_filter_L").toString();
    QString img_filter_R = settings.value("img_filter_R").toString();
    QString img_filter_HandEye = settings.value("img_filter_L_2").toString();
    ui.lineEditNameFilterL->setText(img_filter_L);
    ui.lineEditNameFilterL_2->setText(img_filter_HandEye);
    ui.lineEditNameFilterR->setText(img_filter_R);
	updateNameFilterL(img_filter_L);
	updateNameFilterR(img_filter_R);

    int boardTypeIndex = settings.value("board_type").toInt();
    ui.comboBoardType->setCurrentIndex(boardTypeIndex);
	updateBoardType(boardTypeIndex);

    int board_size_x = settings.value("board_size_x").toInt();
    int board_size_y = settings.value("board_size_y").toInt();
    double square_size = settings.value("square_size").toDouble();
    ui.spinBoxBoardSizeX->setValue(board_size_x);
    ui.spinBoxBoardSizeY->setValue(board_size_y);
    ui._squarePhysicalSize_SPINB->setValue(square_size);

    int board_size_x_2 = settings.value("board_size_x_2").toInt();
    int board_size_y_2 = settings.value("board_size_y_2").toInt();
    double square_size_2 = settings.value("square_size_2").toDouble();
    ui.spinBoxBoardSizeX_2->setValue(board_size_x_2);
    ui.spinBoxBoardSizeY_2->setValue(board_size_y_2);
    ui._squarePhysicalSize_SPINB_2->setValue(square_size_2);

    QString left_topic = settings.value("left_topic").toString();
    QString right_topic = settings.value("right_topic").toString();
    ui.lineEditImgTopicL->setText(left_topic);
    ui.lineEditImgTopicR->setText(right_topic);

    QString GuessPath1 = settings.value("intrinsicGuessCam1Path").toString();
    QString GuessPath2 = settings.value("intrinsicGuessCam2Path").toString();
    QString GuessPathHandEye = settings.value("intrinsicGuessCam1Path_2").toString();
    QString RobotPoseSavePath = settings.value("RobotPoseSavePath").toString();
    ui._intrinsicGuessCam1Path->setText(GuessPath1);
    ui._intrinsicGuessCam2Path->setText(GuessPath2);
    ui._intrinsicGuessCam1Path_2->setText(GuessPathHandEye);
    ui._RobotPoseSavePath->setText(RobotPoseSavePath);

    int calTypeIndex = settings.value("cal_type").toInt();
    ui.comboSelectCalibType->setCurrentIndex(calTypeIndex);
    updateCalType(calTypeIndex);
    int calTypeIndex_2 = settings.value("cal_type_2").toInt();
    ui.comboSelectCalibType_2->setCurrentIndex(calTypeIndex_2);
    updateHandEyeCalType(calTypeIndex_2);

	/* Narrow Stereo tab */
    ui.lineEditNarrowIntrinsicLeft->setText(settings.value("narrowIntrinsicLeft").toString());
    ui.lineEditNarrowIntrinsicRight->setText(settings.value("narrowIntrinsicRight").toString());
    ui.lineEditNarrowImageLeft->setText(settings.value("narrowImageLeft").toString());
    ui.lineEditNarrowImageRight->setText(settings.value("narrowImageRight").toString());
  //  ui.spinBoxNarrowBoardSizeX->setValue(settings.value("narrowBoardSizeX").toInt());
  //  ui.spinBoxNarrowBoardSizeY->setValue(settings.value("narrowBoardSizeY").toInt());
    ui.lineEditNarrowOutputLeft->setText(settings.value("narrowOutputLeft").toString());
    ui.lineEditNarrowOutputRight->setText(settings.value("narrowOutputRight").toString());



	// set sharedData vars;
	updateImageLists();
    updatePreviewSizeFields(1);
	updateSquareSize();
	updateRectSavePath();
}

void MainWindow::writeSettings() {
    QSettings settings("DTI", "CalTI");
    settings.setValue("img_path_L",ui.imgPathL->text());
    settings.setValue("img_path_L_2",ui.imgPathL_2->text());
    settings.setValue("img_path_R",ui.imgPathR->text());
    settings.setValue("img_filter_L",ui.lineEditNameFilterL->text());
    settings.setValue("img_filter_R",ui.lineEditNameFilterR->text());
    settings.setValue("img_filter_L_2",ui.lineEditNameFilterL_2->text());
    settings.setValue("RobotPosePath",ui._RobotPosePath->text());
    settings.setValue("board_type",ui.comboBoardType->currentIndex());
    settings.setValue("board_size_x",ui.spinBoxBoardSizeX->value());
    settings.setValue("board_size_y",ui.spinBoxBoardSizeY->value());
    settings.setValue("square_size",ui._squarePhysicalSize_SPINB->value());
    settings.setValue("board_type_2",ui.comboBoardType_2->currentIndex());
    settings.setValue("board_size_x_2",ui.spinBoxBoardSizeX_2->value());
    settings.setValue("board_size_y_2",ui.spinBoxBoardSizeY_2->value());
    settings.setValue("square_size_2",ui._squarePhysicalSize_SPINB_2->value());
    settings.setValue("left_topic",ui.lineEditImgTopicL->text());
    settings.setValue("right_topic",ui.lineEditImgTopicR->text());
    settings.setValue("cal_type",ui.comboSelectCalibType->currentIndex());

    settings.setValue("intrinsicGuessCam1Path", ui._intrinsicGuessCam1Path->text());
    settings.setValue("intrinsicGuessCam2Path", ui._intrinsicGuessCam2Path->text());
    settings.setValue("intrinsicGuessCam1Path_2", ui._intrinsicGuessCam1Path_2->text());
    settings.setValue("RobotPoseSavePath", ui._RobotPoseSavePath->text());


    /* Narrow Stereo tab */
    settings.setValue("narrowIntrinsicLeft", ui.lineEditNarrowIntrinsicLeft->text());
    settings.setValue("narrowIntrinsicRight", ui.lineEditNarrowIntrinsicRight->text());
    settings.setValue("narrowImageLeft", ui.lineEditNarrowImageLeft->text());
    settings.setValue("narrowImageRight", ui.lineEditNarrowImageRight->text());
  //  settings.setValue("narrowBoardSizeX", ui.spinBoxNarrowBoardSizeX->value());
   // settings.setValue("narrowBoardSizeY", ui.spinBoxNarrowBoardSizeY->value());
    settings.setValue("narrowOutputLeft", ui.lineEditNarrowOutputLeft->text());
    settings.setValue("narrowOutputRight", ui.lineEditNarrowOutputRight->text());



   // settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::clearIntrinsicVals(dti::CameraID CAM)
{
	_sharedData->setIntrinsicGuessPath(CAM, "");
	Q_EMIT intrinsicGuessPath((dti::CameraID) CAM, QString(CAMERA_MATRIX_NAME), QString(DISTORTION_COEFF_NAME));
}

/********************************************************************************
 *																				*
 *                              SLOTS 											*
 *																				*
 ********************************************************************************/

void MainWindow::clearIntrinsicValsSlot()
{
	QPushButton* src;
	try {
		src = (QPushButton* ) sender();
		if(src == ui._intrinsicClearValsBtnL)
			clearIntrinsicVals(dti::LEFT);
		else if(src == ui._intrinsicClearValsBtnR)
			clearIntrinsicVals(dti::RIGHT);

	} catch (std::exception& ex) {
        Q_EMIT consoleOutSig(ex.what(), dti::VerboseLevel::ERROR);
	}
}


void MainWindow::updateIntrinsicGuessPath()
{
	QPushButton* src;
	QString path = "";

	int CAM = -1;
	try{
		src = (QPushButton *) sender();

		if (src == ui._intrinsicReadFromFileBtnR)
		{
			CAM = dti::RIGHT;
			path = ui._intrinsicGuessCam2Path->text();
		}

		else if (src == ui._intrinsicReadFromFileBtnL)
		{
		 	CAM = dti::LEFT;
		 	path = ui._intrinsicGuessCam1Path->text();
		}
		else return; // not a valid sender

		if (!QFile::exists(path)) {
			path = "";
		}
		_sharedData->setIntrinsicGuessPath(CAM, path);
	}
	catch (std::exception& ex) {
        Q_EMIT consoleOutSig(ex.what(),dti::VerboseLevel::ERROR);
	}

	Q_EMIT intrinsicGuessPath((dti::CameraID) CAM, QString(CAMERA_MATRIX_NAME), QString(DISTORTION_COEFF_NAME));
}


void MainWindow::processDoAllBtn()
{
	readFlags();
	_calThread->doAll();
	// disable all but cancel btn
	ui._doAllBtn->setEnabled(false);
	ui._doDetectCornersBtn->setEnabled(false);
	ui._doCalibrateBtn->setEnabled(false);
	ui._doRectifyBtn->setEnabled(false);
	ui._doSaveResultsBtn->setEnabled(false);

	_calThread->activate();
}

void MainWindow::processDoCalibrateBtn()
{
//	readCalFlags();
	readFlags();
	_calThread->doCalibrate();
	// disable all but cancel btn
	ui._doAllBtn->setEnabled(false);
	ui._doDetectCornersBtn->setEnabled(false);
	ui._doCalibrateBtn->setEnabled(false);
	ui._doRectifyBtn->setEnabled(false);
	ui._doSaveResultsBtn->setEnabled(false);

	_calThread->activate();
}

void MainWindow::processDoCalibrateHandEyeBtn()
{

	std::cout << "Running hand eye calibration!!" << std::endl;
	readRobotFlags();

	_calThread->doCalibrate();
	_calThread->activate();
}

void MainWindow::processDoDetectCornersBtn()
{
	readCornerDetectionFlags();
	_calThread->doDetectCorners();
	// disable all but cancel btn
	ui._doAllBtn->setEnabled(false);
	ui._doDetectCornersBtn->setEnabled(false);
	ui._doCalibrateBtn->setEnabled(false);
	ui._doRectifyBtn->setEnabled(false);
	ui._doSaveResultsBtn->setEnabled(false);
	ui._commitROS->setEnabled(false);

	_calThread->activate();
}

void MainWindow::processDoDetectCornersHandEyeBtn()
{
	readHandEyeCornerDetectionFlags();
	_calThread->doDetectCorners();
	// disable all but cancel btn
	ui._doAllBtn_2->setEnabled(false);
	ui._doDetectCornersBtn_2->setEnabled(false);
	ui._doCalibrateBtn_2->setEnabled(false);
	ui._doComputeCamPoseBtn_2->setEnabled(false);
	ui._doSaveResultsBtn_2->setEnabled(false);
	ui._commitROS_2->setEnabled(false);

	_calThread->activate();
}

void MainWindow::processDoRectifyBtn()
{
	readRectificationFlags();
	_calThread->doRectify();
	// disable all but cancel btn
	ui._doAllBtn->setEnabled(false);
	ui._doDetectCornersBtn->setEnabled(false);
	ui._doCalibrateBtn->setEnabled(false);
	ui._doRectifyBtn->setEnabled(false);
	ui._doSaveResultsBtn->setEnabled(false);

	_calThread->activate();

}


void MainWindow::processDoSaveBtn()
{
	// set storage paths
	QPushButton *btn;
	QString startPath,
			filePath;
	try {
		btn = (QPushButton*) sender();
	} catch (std::exception& e) {
		//error
		return;
	}

	// save results path
	if(btn == ui._doSaveResultsBtn)
	{
		startPath = ui.imgPathL->text();
		if (!QDir(startPath).exists())
				startPath = QDir::currentPath();
		filePath = QFileDialog::getExistingDirectory(this,tr("Select directory"), startPath);
		if (filePath != "")
		{
			if( !filePath.endsWith("/") )
				filePath.append("/");

			_sharedData->setSavePath(filePath);
		}
	}
	_calThread->doSaveResults();

	ui._doAllBtn->setEnabled(false);
	ui._doDetectCornersBtn->setEnabled(false);
	ui._doCalibrateBtn->setEnabled(false);
	ui._doRectifyBtn->setEnabled(false);
	ui._doSaveResultsBtn->setEnabled(false);

	_calThread->activate();
}

void MainWindow::processDoSaveHandEyeBtn()
{
	// set storage paths
	QPushButton *btn;
	QString startPath,
			filePath;
	try {
		btn = (QPushButton*) sender();
	} catch (std::exception& e) {
		//error
		return;
	}

	// save results path
	if(btn == ui._doSaveResultsBtn_2)
	{
		startPath = ui._RobotPosePath->text();
		if (!QDir(startPath).exists())
				startPath = QDir::currentPath();
		filePath = QFileDialog::getSaveFileName(this,tr("Save Hand Eye calibration data as"));
	//	filePath = QFileDialog::getExistingDirectory( (this,tr("Select directory"), startPath);

		if (filePath != "")
		{
			if( !filePath.endsWith(".yml") )
				filePath.append(".yml");

			_sharedData->setHandEyeSavePath(filePath);
		}

	}
	readRobotFlags();
	_calThread->doSaveResults();

	ui._doAllBtn->setEnabled(false);
	ui._doDetectCornersBtn->setEnabled(false);
	ui._doCalibrateBtn->setEnabled(false);
	ui._doRectifyBtn->setEnabled(false);
	ui._doSaveResultsBtn->setEnabled(false);

	_calThread->activate();
}

void MainWindow::processDoClearRobotPoseBtn()
{
	std::cout << "processDoCancelRobotPoseBtn" << std::endl;

}

void MainWindow::processDoShowRobotPoseBtn()
{
    if(!_sharedData->getRobotPosePath().isEmpty()){
       _calThread->doLoadRobotPoses();
    }

	std::vector<tiv::pose> poses = _sharedData->getRobotPoseArray();
	QString str;
	str.append("================ Number of robot poses loaded: "); str.append(QString::number(poses.size())); str.append("===================\n");
	
	if(poses.size() > 0){
	  for(int i=0; i<=poses.size()-1;i++)
	  { 
	    str.append("\n==============Robot Pose: "); str.append(QString::number(i)); str.append("==================\n");
        Q_EMIT consoleOutSig(str, dti::VerboseLevel::INFO);
	    str.clear();

	    str.append("X:");str.append(" " + QString::number(poses.at(i).t.x) + " ");
	    str.append("Y:");str.append(" " + QString::number(poses.at(i).t.y) + " ");
	    str.append("Z:");str.append(" " + QString::number(poses.at(i).t.z) + " ");
        str.append("QW:");str.append(" " + QString::number(poses.at(i).q.w) + " ");
        str.append("QX:");str.append(" " + QString::number(poses.at(i).q.x) + " ");
        str.append("QY:");str.append(" " + QString::number(poses.at(i).q.y) + " ");
        str.append("QZ:");str.append(" " + QString::number(poses.at(i).q.z) + " ");

        Q_EMIT consoleOutSig(str, dti::VerboseLevel::INFO);
	    str.clear();
	 
	  }
	}else
      Q_EMIT consoleOutSig("No robot poses in file!!", dti::VerboseLevel::WARN);
	
	updateRobotPoseCount();

}

void MainWindow::processDoSaveRobotPoseBtn()
{
	readRobotFlags();
	_calThread->dosaveRobotPoses();

}

void MainWindow::processDoCancelBtn()
{
	_calThread->doStop();
}

void MainWindow::processDoComputeCameraPosesBtn()
{
	std::cout << "Computing camera poses" << std::endl;
	_calThread->doComputePose();
	_calThread->activate();
}

void MainWindow::activateCalibrate() {
	using namespace dti;
	// load params
	readFlags();
	// set params to CalibThread
	// Den henter params fra _sharedData

	// run the calibration
	if (!_calThread->isRunning()) {
		_calThread->activate();
		ui._doAllBtn->setText("Cancel");
	} else {
		if (ui._doAllBtn->text() == "Cancel") {
			ui._doAllBtn->setEnabled(false);
			ui._doAllBtn->setText("Waiting");
			if(_annotationDialog != NULL) {
				delete _annotationDialog;
				_annotationDialog = NULL;
			}
		}
		_calThread->deactivate(true);
	}
}

void MainWindow::calThreadFinished()
{
		ui._doAllBtn->setEnabled(true);
		ui._doDetectCornersBtn->setEnabled(true);
		ui._doCalibrateBtn->setEnabled(true);
		ui._doRectifyBtn->setEnabled(true);
		ui._doSaveResultsBtn->setEnabled(true);
		ui._commitROS->setEnabled(true);

		ui._doAllBtn_2->setEnabled(true);
		ui._doDetectCornersBtn_2->setEnabled(true);
		ui._doCalibrateBtn_2->setEnabled(true);
		ui._doComputeCamPoseBtn_2->setEnabled(true);
		ui._doSaveResultsBtn_2->setEnabled(true);
		ui._commitROS_2->setEnabled(true);
}

void MainWindow::readCornerDetectionFlags(){

	double defaultEps = 0.00001;
	int defaultIte = 100;
	bool res = false;
	double epsilon;
	int ite;

	dti::CornerDetectionFlags cornerFlags;

	cornerFlags._displayCorners 	= ui._previewCorner_CB->isChecked();
	QString s = (ui._previewCorner_CB->isChecked() ? "Yes":"No");
    Q_EMIT consoleOutSig("cornerFlags -> preview? " + s + "\n",dti::VerboseLevel::INFO);
	cornerFlags._doPreCornerCheck 	= ui._preCornerAnalysis_CB->isChecked();
	cornerFlags._filterQuads		= ui._filterQuads_CB->isChecked();
	cornerFlags._normalizeImage		= ui._normalizeImg_CB->isChecked();
	cornerFlags._useAdaptiveThres 	= ui._adaptiveThres_CB->isChecked();

	// termination criteria
	if(ui._itc_epsilon_Corner_checkBox_2->isChecked()) {
		epsilon		= ui._itc_epsilon_Corner_LineEdit->text().toDouble(&res);
		cornerFlags._stopOnEpsilon = res ? epsilon : defaultEps;
	} else
		cornerFlags._stopOnEpsilon = 0;

	if(ui._itc_iterations_Corner_checkBox->isChecked()) {
		ite		= ui._itc_max_Iter_Corner_LineEdit_->text().toInt(&res);
		cornerFlags._stopOnIterations	= res ? ite : defaultIte;
	} else
		cornerFlags._stopOnIterations = 0;


	cornerFlags._cornerAnnotation = (dti::AnnotationType) ui._annotationType_Combo->currentIndex();

	_sharedData->setCornerFlags(cornerFlags);

}

void MainWindow::readHandEyeCornerDetectionFlags(){

	double defaultEps = 0.00001;
	int defaultIte = 100;
	bool res = false;
	double epsilon;
	int ite;

	dti::CornerDetectionFlags cornerFlags;

	cornerFlags._displayCorners 	= ui._previewCorner_CB_2->isChecked();
	QString s = (ui._previewCorner_CB_2->isChecked() ? "Yes":"No");
    Q_EMIT consoleOutSig("cornerFlags -> preview? " + s + "\n",dti::VerboseLevel::INFO);
	cornerFlags._doPreCornerCheck 	= ui._preCornerAnalysis_CB_2->isChecked();
	cornerFlags._filterQuads		= ui._filterQuads_CB_2->isChecked();
	cornerFlags._normalizeImage		= ui._normalizeImg_CB_2->isChecked();
	cornerFlags._useAdaptiveThres 	= ui._adaptiveThres_CB_2->isChecked();

	// termination criteria
	if(ui._itc_epsilon_Corner_checkBox_4->isChecked()) {
		epsilon		= ui._itc_epsilon_Corner_LineEdit_3->text().toDouble(&res);
		cornerFlags._stopOnEpsilon = res ? epsilon : defaultEps;
	} else
		cornerFlags._stopOnEpsilon = 0;

	if(ui._itc_iterations_Corner_checkBox_3->isChecked()) {
		ite		= ui._itc_max_Iter_Corner_LineEdit_2->text().toInt(&res);
		cornerFlags._stopOnIterations	= res ? ite : defaultIte;
	} else
		cornerFlags._stopOnIterations = 0;


	cornerFlags._cornerAnnotation = (dti::AnnotationType) ui._annotationType_Combo_3->currentIndex();

	_sharedData->setCornerFlagsHandEye(cornerFlags);

}

void MainWindow::readRobotFlags(){

	dti::RobotFlags flags;

	flags._type = (dti::RotationType) ui.comboAngleType->currentIndex();
	flags._displayRobotPose = ui.checkDisplayLiveRobotPoses->isChecked();
	flags._isInverted = ui.checkBoxIsInverted->isChecked();
	flags._SaveRobotPose = ui.checkSaveLiveRobotPoses->isChecked();

	_sharedData->setRobotFlags(flags);

}

void MainWindow::readIntrinsicGuessVals()
{
	dti::IntrinsicGuessVals valsL, valsR;
	bool ok = false;
	double tmp = 0;
	//focal length
	tmp = ui._fixFocalLength_VALx_CL->text().toDouble(&ok);;
	valsL._focalLength[0] = ok ? tmp : 0.0;
	tmp = ui._fixFocalLength_VALy_CL->text().toDouble(&ok);
	valsL._focalLength[1] = ok ? tmp : 0.0;

	// Principal point
	tmp = ui._fixPrincipalPoint_VALx_CL->text().toDouble(&ok);;
	valsL._principalPoint[0] = ok ? tmp : 0.0;
	tmp = ui._fixPrincipalPoint_VALy_CL->text().toDouble(&ok);
	valsL._principalPoint[1] = ok ? tmp : 0.0;


	// Distortion radial
	tmp = ui._fix_K1_VAL_CL->text().toDouble(&ok);
	valsL._radiadistortion[0] = ok ? tmp : 0.0;
	tmp = ui._fix_K2_VAL_CL->text().toDouble(&ok);
	valsL._radiadistortion[1] = ok ? tmp : 0.0;
	tmp = ui._fix_K3_VAL_CL->text().toDouble(&ok);
	valsL._radiadistortion[2] = ok ? tmp : 0.0;
	tmp = ui._fix_K4_VAL_CL->text().toDouble(&ok);
	valsL._radiadistortion[3] = ok ? tmp : 0.0;
	tmp = ui._fix_K5_VAL_CL->text().toDouble(&ok);
	valsL._radiadistortion[4] = ok ? tmp : 0.0;
	tmp = ui._fix_K6_VAL_CL->text().toDouble(&ok);
	valsL._radiadistortion[5] = ok ? tmp : 0.0;

	// Distortion tangential
	tmp = ui._fix_P1_VAL_CL->text().toDouble(&ok);
	valsL._tangentialDistortion[0] = ok ? tmp : 0.0;

	tmp = ui._fix_P2_VAL_CL->text().toDouble(&ok);
	valsL._tangentialDistortion[1] = ok ? tmp : 0.0;



	//focal length
	tmp = ui._fixFocalLength_VALx_CR->text().toDouble(&ok);;
	valsR._focalLength[0] = ok ? tmp : 0.0;
	tmp = ui._fixFocalLength_VALy_CR->text().toDouble(&ok);
	valsR._focalLength[1] = ok ? tmp : 0.0;

	// Principal point
	tmp = ui._fixPrincipalPoint_VALx_CR->text().toDouble(&ok);;
	valsR._principalPoint[0] = ok ? tmp : 0.0;
	tmp = ui._fixPrincipalPoint_VALy_CR->text().toDouble(&ok);
	valsR._principalPoint[1] = ok ? tmp : 0.0;


	// Distortion radial
	tmp = ui._fix_K1_VAL_CR->text().toDouble(&ok);
	valsR._radiadistortion[0] = ok ? tmp : 0.0;
	tmp = ui._fix_K2_VAL_CR->text().toDouble(&ok);
	valsR._radiadistortion[1] = ok ? tmp : 0.0;
	tmp = ui._fix_K3_VAL_CR->text().toDouble(&ok);
	valsR._radiadistortion[2] = ok ? tmp : 0.0;
	tmp = ui._fix_K4_VAL_CR->text().toDouble(&ok);
	valsR._radiadistortion[3] = ok ? tmp : 0.0;
	tmp = ui._fix_K5_VAL_CR->text().toDouble(&ok);
	valsR._radiadistortion[4] = ok ? tmp : 0.0;
	tmp = ui._fix_K6_VAL_CR->text().toDouble(&ok);
	valsR._radiadistortion[5] = ok ? tmp : 0.0;

	// Distortion tangential
	tmp = ui._fix_P1_VAL_CR->text().toDouble(&ok);
	valsR._tangentialDistortion[0] = ok ? tmp : 0.0;

	tmp = ui._fix_P2_VAL_CR->text().toDouble(&ok);
	valsR._tangentialDistortion[1] = ok ? tmp : 0.0;



	_sharedData->setIntrinsicGuessValsL(valsL);
	_sharedData->setIntrinsicGuessValsR(valsR);
}

void MainWindow::readCalFlags()
{
	// it is only neccesatry to read the left flags since left and right must be equal, and are so
	dti::CalibrationFlags calFlags;
	calFlags._fixAspectRatio		= ui._fixAspectRatio_CB_CL->isChecked();
	calFlags._useIntrinsicGuess		= ui._useIntrinsicGuess_CB->isChecked();
	calFlags._fixAspectRatio		= ui._fixAspectRatio_CB_CL->isChecked();
	calFlags._fixPrincipalPoint		= ui._fixPrincipalPoint_CB_CL->isChecked();
	calFlags._zeroTangentDistortion	= ui._fixZeroTangentDistortion_CB_CL->isChecked();
	calFlags._fixFocalLength		= ui._fixFocalLength_CB_CL->isChecked();
	calFlags._fix_K1				= ui._fix_K1_CB_CL->isChecked();
	calFlags._fix_K2				= ui._fix_K2_CB_CL->isChecked();
	calFlags._fix_K3				= ui._fix_K3_CB_CL->isChecked();
	calFlags._fix_K4				= ui._fix_K4_CB_CL->isChecked();
	calFlags._fix_K5				= ui._fix_K5_CB_CL->isChecked();
	calFlags._fix_K6				= ui._fix_K6_CB_CL->isChecked();
	calFlags._useRationalModel		= ui._useRationalModel_CB->isChecked();
	_sharedData->setCalFlags(calFlags);
}

void MainWindow::readStereoFlags()
{
	double defaultEps = 0.00001;
	int defaultIte = 100;
	bool res = false;
	double epsilon;
	int ite;

	dti::StereoCalibFlags scFlags;
	scFlags._fixIntrinsicStereo		= ui._fixIntrinsicStereo_CB->isChecked();
	scFlags._sameFocalLengthStereo	= ui._sameFocalLengthStereo_CB->isChecked();


	// Iteration termination criteria
	if(ui._itc_epsilon_Stereo_checkBox->isChecked()) {
		epsilon		= ui._itc_epsilon_Stereo_LineEdit->text().toDouble(&res);
		scFlags._stopOnEpsilon = res ? epsilon : defaultEps;
	} else
		scFlags._stopOnEpsilon = 0;

	if(ui._itc_iterations_Stereo_checkBox->isChecked()) {
		ite		= ui._itc_max_Iter_Stereo_LineEdit->text().toInt(&res);
		scFlags._stopOnIterations	= res ? ite : defaultIte;
	} else
		scFlags._stopOnIterations = 0;

	_sharedData->setStereoFlags(scFlags);
}

void MainWindow::readRectificationFlags()
{
	dti::RectificationFlags rectFlags;

	rectFlags._displayRectification = ui._displayRectification_CB->isChecked();
	rectFlags._displayValidRect		= ui._rectDisplayValidRect_CB->isChecked();
	rectFlags._zeroRectifyDisparity	= ui._zeroRectifyDisparity_CB->isChecked();

	rectFlags._saveRectificationImg = ui._saveRectification_CB->isChecked();
	rectFlags._rectAlg				= (dti::RectAlg) 	ui._rectAlgorithm_Combo->currentIndex();
	rectFlags._fundamentalAlg		= (dti::RectFMatAlg)ui._rectFundamental_Combo->currentIndex();
	rectFlags._alpha				= ui._rectAlpha_CB->isChecked() ? ui._rectAlpha_VAL->value() : -1;
	// Read param 1 and 2
	bool 	res = false;
	double 	tmp = ui._rectParam1_VAL->text().toDouble(&res);
	rectFlags._param1				= res ? tmp : 3.0; // if fails set to openCV defaults
	res 	= false;
	tmp 	= ui._rectParam2_VAL->text().toDouble(&res);
	rectFlags._param2				= res ? tmp : 0.99; // if fails set to openCV defaults
	_sharedData->setRectFlags(rectFlags);

}

void MainWindow::readFlags()
{
	// Corner flags camera Calibration
	readCornerDetectionFlags();

	// Calib flags
	readCalFlags();

	// Intrinsic guess vals
	readIntrinsicGuessVals();
	// Stereo Flags
	readStereoFlags();

	// Rectification flags
	readRectificationFlags();

	//Narrow Stereo Flags
	readNarrowStereoParameters();
}

void MainWindow::updateUseIntrinsic(bool val)
{
		ui._intrinsicGuessCam1Path->setEnabled(val);
		ui._intrinsicGuessCam2Path->setEnabled(val);
		ui._intrinsicGuessCam1btnFileBrowse->setEnabled(val);
		ui._intrinsicGuessCam2btnFileBrowse->setEnabled(val);
		ui._intrinsicGuessCam1label->setEnabled(val);
		ui._intrinsicGuessCam2label->setEnabled(val);
		ui.groupBox_inspectIntrinsic->setChecked(val);
		ui._sameFocalLengthStereo_CB->setEnabled(!val);
		ui.groupBox_inspectIntrinsic->setEnabled(val);
}

void MainWindow::selectPath() {
	QToolButton *btn = (QToolButton *) sender();
//	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "",
//			tr("Text Files (*.txt);;C++ Files (*.cpp *.h)"));

	QString filePath;
	QString startPath;
	if (btn == ui.btnFileBrowseL) {
		startPath = ui.imgPathL->text();
		if (!QDir(startPath).exists())
			startPath = QDir::current().absolutePath();
		filePath = QFileDialog::getExistingDirectory(this, tr("Select directory"), startPath);
		if (filePath != "")
			ui.imgPathL->setText(filePath);
	} else if (btn == ui.btnFileBrowseR) {
		startPath = ui.imgPathR->text();
		if (!QDir(startPath).exists())
			startPath = QDir::currentPath();
		filePath = QFileDialog::getExistingDirectory(this, tr("Select directory"), startPath);
		if (filePath != "")
			ui.imgPathR->setText(filePath);

	}
	//Hand eye imagePath
	else if (btn == ui.btnFileBrowseL_2) {
		startPath = ui.imgPathL_2->text();
		if (!QDir(startPath).exists())
			startPath = QDir::currentPath();
		filePath = QFileDialog::getExistingDirectory(this, tr("Select directory"), startPath);
		if (filePath != "")
			ui.imgPathL_2->setText(filePath);
	}
    //Path to hand eye robot poses
	else if (btn == ui._RobotPathbtnFileBrowse) {
				startPath = ui._RobotPosePath->text();
				if (!QDir(startPath).exists())
					startPath = QDir::currentPath();
                filePath = QFileDialog::getOpenFileName(this, tr("Load a Robot file (.xml)"), startPath);
                std::cout << "filepath " << filePath.toStdString() << std::endl;
                if (filePath != ""){
					ui._RobotPosePath->setText(filePath);
                }
	}
	else if (btn == ui._RobotSavePathbtnFileBrowse_3) {
		startPath = ui._RobotPoseSavePath->text();
		if (!QDir(startPath).exists())
			startPath = QDir::currentPath();
		filePath = QFileDialog::getSaveFileName(this, tr("Create or select file"), startPath);
		//filePath = QFileDialog::getExistingDirectory(this, tr("Select directory"), startPath);
		if (filePath != "")

            if( !filePath.endsWith(".xml") )
                    filePath.append(".xml");

         if (filePath != ""){
			ui._RobotPoseSavePath->setText(filePath);     
         }
	}

	 else if (btn == ui._intrinsicGuessCam1btnFileBrowse_2) {
			startPath = ui._intrinsicGuessCam1Path_2->text();
			if (!QDir(startPath).exists())
				startPath = QDir::currentPath();
			filePath = QFileDialog::getOpenFileName(this, tr("Select where to load intrinsic guess for the camera"), startPath);
			if (filePath != "")
				ui._intrinsicGuessCam1Path_2->setText(filePath);
		}
	// intrinsic file selection
	else if (btn == ui._intrinsicGuessCam1btnFileBrowse) {
		startPath = ui._intrinsicGuessCam1Path->text();
		if (!QDir(startPath).exists())
			startPath = QDir::currentPath();
		filePath = QFileDialog::getOpenFileName(this, tr("Select where to load intrinsic guess for the LEFT camera"), startPath);
		if (filePath != "")
			ui._intrinsicGuessCam1Path->setText(filePath);
	} else if (btn == ui._intrinsicGuessCam2btnFileBrowse) {
		startPath = ui._intrinsicGuessCam2Path->text();
		if (!QDir(startPath).exists())
			startPath = QDir::currentPath();
		filePath = QFileDialog::getOpenFileName(this, tr("Select where to load intrinsic guess for the RIGHT camera"), startPath);
		if (filePath != "")
			ui._intrinsicGuessCam2Path->setText(filePath);
	}

	// save rectification paths
	else if (btn == ui._rectificationSavePathBtnL)
	{
		startPath = ui.imgPathL->text();
		if (!QDir(startPath).exists())
			startPath = QDir::currentPath();
		filePath = QFileDialog::getExistingDirectory(this,tr("Select directory"), startPath);
		if (filePath != "")
			ui._rectificationSavePathTxtL->setText(filePath);
		updateRectSavePath();
	}
	else if (btn == ui._rectificationSavePathBtnR)
	{
		startPath = ui.imgPathR->text();
		if (!QDir(startPath).exists())
			startPath = QDir::currentPath();
		filePath = QFileDialog::getExistingDirectory(this,tr("Select directory"), startPath);
		if (filePath != "")
			ui._rectificationSavePathTxtR->setText(filePath);
		updateRectSavePath();
	}
	//Narrow stereo
	else if (btn == ui.btnFileBrowserNSIntrinsicLeft)
		{
			startPath = ui.lineEditNarrowIntrinsicLeft->text();
			if (!QDir(startPath).exists())
				startPath = QDir::currentPath();
			filePath = QFileDialog::getOpenFileName(this, tr("Select where to load intrinsic for the LEFT camera"), startPath);
			if (filePath != "")
				ui.lineEditNarrowIntrinsicLeft->setText(filePath);
			//TODO: save path in shared data
			}
		else if (btn == ui.btnFileBrowserNSIntrinsicRight)
		{
			startPath = ui.lineEditNarrowIntrinsicRight->text();
			if (!QDir(startPath).exists())
				startPath = QDir::currentPath();
			filePath = QFileDialog::getOpenFileName(this, tr("Select where to load intrinsic for the RIGHT camera"), startPath);
			if (filePath != "")
				ui.lineEditNarrowIntrinsicRight->setText(filePath);
			//TODO: save path in shared data
		}

		else if (btn == ui.btnFileBrowserNSLeftImage)
				{
					startPath = ui.lineEditNarrowImageLeft->text();
					if (!QDir(startPath).exists())
						startPath = QDir::currentPath();
					filePath = QFileDialog::getOpenFileName(this, tr("Select where to load intrinsic for the LEFT camera"), startPath);
					if (filePath != "")
						ui.lineEditNarrowImageLeft->setText(filePath);
					//TODO: save path in shared data
					}
				else if (btn == ui.btnFileBrowserNSRightImage)
				{
					startPath = ui.lineEditNarrowImageRight->text();
					if (!QDir(startPath).exists())
						startPath = QDir::currentPath();
					filePath = QFileDialog::getOpenFileName(this, tr("Select where to load intrinsic for the RIGHT camera"), startPath);
					if (filePath != "")
						ui.lineEditNarrowImageRight->setText(filePath);
					//TODO: save path in shared data
				}

	//Renew all paths and images!!
	updatePaths();
	updateImageLists();
	updateImageView();

}

void MainWindow::updatePaths()
{
	QString leftPath = ui.imgPathL->text();
	QString handeyeMonoPath = ui.imgPathL_2->text();
	QString rightPath = ui.imgPathR->text();
	_sharedData->setImgPathL(leftPath);
	_sharedData->setImgPathR(rightPath);
	_sharedData->setImgPathHandEyeMono(handeyeMonoPath);
	_sharedData->setIntrinsicGuessPathL(ui._intrinsicGuessCam1Path->text());
	_sharedData->setIntrinsicGuessPathR(ui._intrinsicGuessCam2Path->text());
	_sharedData->setIntrinsicHandEyePath(ui._intrinsicGuessCam1Path_2->text());
	_sharedData->setRobotPosePath(ui._RobotPosePath->text());
	_sharedData->setRobotPoseLivePath(ui._RobotPoseSavePath->text());


	if( leftPath.endsWith("/") )
		leftPath.append("rectified");
	else
		leftPath.append("/rectified");

	if( rightPath.endsWith("/") )
		rightPath.append("rectified");
	else
		rightPath.append("/rectified");
	ui._rectificationSavePathTxtL->setText(leftPath);
	ui._rectificationSavePathTxtR->setText(rightPath);
	updateRectSavePath();
}


void MainWindow::updateTermCrit() {
	QCheckBox* src = (QCheckBox*) sender();
	if (src == ui._itc_epsilon_Corner_checkBox_2) {
		if (!ui._itc_epsilon_Corner_checkBox_2->isChecked())
			ui._itc_iterations_Corner_checkBox->setChecked(true);
	} else if (src == ui._itc_iterations_Corner_checkBox) {
		if (!ui._itc_iterations_Corner_checkBox->isChecked())
			ui._itc_epsilon_Corner_checkBox_2->setChecked(true);
	} else if (src == ui._itc_iterations_Stereo_checkBox) {
		if (!ui._itc_iterations_Stereo_checkBox->isChecked())
			ui._itc_epsilon_Stereo_checkBox->setChecked(true);
	} else if (src == ui._itc_epsilon_Stereo_checkBox) {
		if (!ui._itc_epsilon_Stereo_checkBox->isChecked())
			ui._itc_iterations_Stereo_checkBox->setChecked(true);
	}
}

void MainWindow::updatePreviewSizeWidth(QString size)
{
	bool res;
	int w = size.toInt(&res);
	if(res){
		_previewSize.setWidth(w);
		ui.imgLabelR->setMinimumWidth(w);
		ui.imgLabelL->setMinimumWidth(w);
		ui.imgLabelR->setMaximumWidth(w);
		ui.imgLabelL->setMaximumWidth(w);
	}
}


void MainWindow::updatePreviewSizeHeight(QString size)
{
	bool res;
	int h = size.toInt(&res);
	if(res){
		_previewSize.setHeight(h);
		ui.imgLabelR->setMinimumHeight(h);
		ui.imgLabelR->setMaximumHeight(h);
		ui.imgLabelL->setMinimumHeight(h);
		ui.imgLabelL->setMaximumHeight(h);
	}
}

void MainWindow::updatePreviewSizeFields(int index)
{
	switch (index) {
		case dti::w320xh240:
			ui.lineEditPreviewHeight->setEnabled(false);
			ui.lineEditPreviewWidth->setEnabled(false);
			ui.lineEditPreviewHeight->setText("240");
			ui.lineEditPreviewWidth->setText("320");

			ui.lineEditPreviewHeight_2->setEnabled(false);
			ui.lineEditPreviewWidth_2->setEnabled(false);
			ui.lineEditPreviewHeight_2->setText("240");
			ui.lineEditPreviewWidth_2->setText("320");

			updateImageView();
			break;
		case dti::w640xh480:
			ui.lineEditPreviewHeight->setEnabled(false);
			ui.lineEditPreviewWidth->setEnabled(false);
            ui.lineEditPreviewHeight->setText("240");
            ui.lineEditPreviewWidth->setText("320");

			ui.lineEditPreviewHeight_2->setEnabled(false);
			ui.lineEditPreviewWidth_2->setEnabled(false);
			ui.lineEditPreviewHeight_2->setText("480");
			ui.lineEditPreviewWidth_2->setText("640");

			updateImageView();
			break;
		case dti::custom_resolution:
			ui.lineEditPreviewHeight->setEnabled(true);
			ui.lineEditPreviewWidth->setEnabled(true);
			ui.lineEditPreviewWidth->setReadOnly(false);
			ui.lineEditPreviewHeight->setReadOnly(false);
			ui.lineEditPreviewWidth->setFocus();

			ui.lineEditPreviewHeight_2->setEnabled(true);
			ui.lineEditPreviewWidth_2->setEnabled(true);
			ui.lineEditPreviewWidth_2->setReadOnly(false);
			ui.lineEditPreviewHeight_2->setReadOnly(false);
			ui.lineEditPreviewWidth_2->setFocus();

			updateImageView();
			break;
		default:
            Q_EMIT consoleOutSig("unhandled event in updatePreviewSizeFields\n",dti::VerboseLevel::WARN);
			return;
			break;
	}

}

void MainWindow::updateCalType(int index)
{
	// TODO s\F8rg for at GUI opdateres korrekt!
	calType = (dti::CalibrationTypes) index;
	_sharedData->setCalType((dti::CalibrationTypes) index);
	switch ((dti::CalibrationTypes) index){
		case dti::StereoCalibration_online:
			ui.lineEditImgTopicR->setEnabled(true);
			//break;
		case dti::StereoCalibration_offline:
			ui.comboPreviewSize->setCurrentIndex(0);
			ui.groupBox_StereoCalFlags->setChecked(true);
			ui._CalFlags_groupBox->setChecked(true);
			ui._intrinsicGuessCam2label->setVisible(true);
			ui._intrinsicGuessCam2Path->setVisible(true);
			ui._intrinsicGuessCam2btnFileBrowse->setVisible(true);
//			ui._inspectIntrinsicCamera2Tab->setVisible(true);
			ui.lineEditNameFilterR->setVisible(true);
			ui._fileFilterR_label->setVisible(true);
			ui.tabWidget->setTabText(ui.tabWidget->indexOf(ui.tabRectFlags), "Rectification");
			ui._intrinsicGuessCam1label->setVisible(true);
			ui._intrinsicGuessCam1Path->setVisible(true);
			ui._intrinsicGuessCam1btnFileBrowse->setVisible(true);
//			ui._inspectIntrinsicCamera1Tab->setVisible(true);
			if(ui._inspectCamTabs->count() < 2)
				ui._inspectCamTabs->addTab(ui._inspectIntrinsicCamera2Tab, QString("Camera #2"));
			ui.imgLabelL->setVisible(true);
			ui.imgLabelR->setVisible(true);
			ui.btnFileBrowseR->setVisible(true);
			ui.imgPathR->setVisible(true);
			ui.groupBox_StereoCalFlags->setVisible(true);
			ui._rectSavePathRightLabel->setVisible(true);
			ui._rectificationSavePathBtnR->setVisible(true);
			ui._rectificationSavePathTxtR->setVisible(true);
		break;
		case dti::MonoCalibration_offline:
		case dti::MonoCalibration_online:
			ui.lineEditImgTopicR->setEnabled(false);

		//case dti::HandEyeCalibration_offline:
		//case dti::HandEyeCalibration_online:
			//break;
		default: // mono
			ui.comboPreviewSize->setCurrentIndex(1);
			ui.tabRectFlags->setWindowTitle("Undistortion flags");
			ui._CalFlags_groupBox->setChecked(true);
			ui.groupBox_StereoCalFlags->setVisible(false);
			ui.lineEditNameFilterR->setVisible(false);
			ui._fileFilterR_label->setVisible(false);
			ui.groupBox_StereoCalFlags->setChecked(false);

			ui._intrinsicGuessCam2label->setVisible(false);
			ui._intrinsicGuessCam2Path->setVisible(false);
			ui._intrinsicGuessCam2btnFileBrowse->setVisible(false);
//			ui._inspectIntrinsicCamera2Tab->hide();
			ui._inspectCamTabs->removeTab(1);
			ui._intrinsicGuessCam1label->setVisible(true);
			ui._intrinsicGuessCam1Path->setVisible(true);
			ui._intrinsicGuessCam1btnFileBrowse->setVisible(true);
//			ui._inspectIntrinsicCamera1Tab->setVisible(true);
			ui.imgLabelL->setVisible(true);
			ui.imgLabelR->setVisible(false);
			ui.btnFileBrowseR->setVisible(false);
			ui.imgPathR->setVisible(false);
			ui._rectSavePathRightLabel->setVisible(false);
			ui._rectificationSavePathBtnR->setVisible(false);
			ui._rectificationSavePathTxtR->setVisible(false);
		break;
	}

	switch ((dti::CalibrationTypes) index){
		case dti::StereoCalibration_online:
		case dti::MonoCalibration_online:
			ui.liveStreamBox->setVisible(true);
			updateImageView();
			break;
		case dti::StereoCalibration_offline:
		case dti::MonoCalibration_offline:
		default:
			on_pushButtonStreamStop_clicked();
			ui.liveStreamBox->setVisible(false);
			break;
	}
}

void MainWindow::updateHandEyeCalType(int index)
{
	int in = index + 4;
	// TODO s\F8rg for at GUI opdateres korrekt!
	calType = (dti::CalibrationTypes) in;
	_sharedData->setCalTypeHandEye((dti::CalibrationTypes) in);
	switch ((dti::CalibrationTypes) in){


		case dti::HandEyeCalibration_offline:
			ui.liveStreamBox->setVisible(false);
			ui.lineEditImgTopicR->setEnabled(false);
			on_pushButtonStreamStop_clicked();

			ui.groupBoxRobotPoseOnline->setEnabled(false);
			ui.groupBoxLoadRobotData->setEnabled(true);

			ui.imgLabelL->setVisible(true);
			ui.imgLabelR->setVisible(false);
			updateImageView();
			break;
		case dti::HandEyeCalibration_online:
			ui.liveStreamBox->setVisible(true);
			ui.lineEditImgTopicL->setEnabled(true);
			ui.lineEditImgTopicR->setEnabled(true);
			on_pushButtonStreamStop_clicked();

			ui.groupBoxRobotPoseOnline->setEnabled(true);
			ui.groupBoxLoadRobotData->setEnabled(false);

			ui.imgLabelL->setVisible(true);
			ui.imgLabelR->setVisible(false);
			updateImageView();
		break;
		default: // mono

		/*	ui.comboPreviewSize->setCurrentIndex(1);
			ui.tabRectFlags->setWindowTitle("Undistortion flags");
			ui._CalFlags_groupBox->setChecked(true);
			ui.groupBox_StereoCalFlags->setVisible(false);
			ui.lineEditNameFilterR->setVisible(false);
			ui._fileFilterR_label->setVisible(false);
			ui.groupBox_StereoCalFlags->setChecked(false);

			ui._intrinsicGuessCam2label->setVisible(false);
			ui._intrinsicGuessCam2Path->setVisible(false);
			ui._intrinsicGuessCam2btnFileBrowse->setVisible(false);
//			ui._inspectIntrinsicCamera2Tab->hide();
			ui._inspectCamTabs->removeTab(1);
			ui._intrinsicGuessCam1label->setVisible(true);
			ui._intrinsicGuessCam1Path->setVisible(true);
			ui._intrinsicGuessCam1btnFileBrowse->setVisible(true);
//			ui._inspectIntrinsicCamera1Tab->setVisible(true);
			ui.imgLabelL->setVisible(true);
			ui.imgLabelR->setVisible(false);
			ui.btnFileBrowseR->setVisible(false);
			ui.imgPathR->setVisible(false);
			ui._rectSavePathRightLabel->setVisible(false);
			ui._rectificationSavePathBtnR->setVisible(false);
			ui._rectificationSavePathTxtR->setVisible(false);
			*/
		break;
	}

/*	switch ((dti::CalibrationTypes) index){
		case dti::StereoCalibration_online:
		case dti::MonoCalibration_online:

			break;
		case dti::StereoCalibration_offline:
		case dti::MonoCalibration_offline:
		default:

			break;
	}
	*/
}

void MainWindow::updateTab(int index){

	switch (index){
			case 0:
				calType = dti::StereoCalibration_offline;
				tab = dti::TAB_CAMERA;
				_sharedData->setCalType(dti::StereoCalibration_offline);
				ui.labelLeftTopic->setText("Left Topic:");
				ui.labelRightTopic->setText("Right Topic:");
				ui.checkBoxIsInverted->hide();
				//Update boardsize
				_sharedData->setBoardSizeX(ui.spinBoxBoardSizeX->value());
				_sharedData->setBoardSizeY(ui.spinBoxBoardSizeY->value());
				_sharedData->setSquareSize(ui._squarePhysicalSize_SPINB->value());
				_sharedData->setBoardType((dti::CalBoardTypes) ui.comboBoardType->currentIndex());

				on_pushButtonStreamStop_clicked();

				updateImgL();
				updateImgR();
				updatePaths(); //Update image path
				updateImageLists();
				updateImageView();
				break;
			case 1:
				calType = dti::HandEyeCalibration_offline;
				tab = dti::TAB_HANDEYE;
				ui.labelLeftTopic->setText("Image Topic");
				ui.labelRightTopic->setText("Robot TCP Topic:");
				ui.checkBoxIsInverted->show();
				_sharedData->setCalType(dti::HandEyeCalibration_offline);
				//Update boardsize
				_sharedData->setBoardSizeXHandEye(ui.spinBoxBoardSizeX_2->value());
				_sharedData->setBoardSizeYHandEye(ui.spinBoxBoardSizeY_2->value());
				_sharedData->setSquareSizeHandEye(ui._squarePhysicalSize_SPINB_2->value());
				_sharedData->setBoardTypeHandEye((dti::CalBoardTypes) ui.comboBoardType_2->currentIndex());

				on_pushButtonStreamStop_clicked();

				updateImgHandEye();
				updatePaths();
				updateImageLists();
				updateImageView();

//                if(!_sharedData->getRobotPosePath().isEmpty())
 //                   _calThread->doLoadRobotPoses();

				break;

			default:

				break;
		}
}

void MainWindow::updateRobotPoseCount(){
	int count = (int)_sharedData->getRobotPoseArray().size();
	std::stringstream ss;
	ss << count;
	std::string str = ss.str();

	ui.labelRobotPoseCount->setText(QString::fromStdString(str));
}


void MainWindow::consoleOut(QString msg, dti::VerboseLevel level)
{
    QString errorHtml = "<font color=\"Red\">";
    QString warnHtml = "<font color=\"#E9AB17\">";
    QString infoHtml = "<font color=\"Black\">";
    QString endHtml = "</font><br>";

    switch(level)
    {
        case dti::VerboseLevel::DEBUG:
            msg = infoHtml % msg;
            break;
        case dti::VerboseLevel::INFO:
            msg = infoHtml % msg;
            break;
        case dti::VerboseLevel::WARN:
            msg = warnHtml % msg;
            break;
        case dti::VerboseLevel::ERROR:
            msg = errorHtml % msg;
            break;
        case dti::VerboseLevel::FATAL:
            msg = errorHtml % msg;
            break;
        default:
            msg = infoHtml % msg;
            break;
    }

    msg = msg % endHtml;
    ui._console->insertHtml(msg);
    ui._console->moveCursor(QTextCursor::End, QTextCursor::MoveAnchor);

 //  ui._console->insertPlainText(msg);

	QScrollBar *sb = ui._console->verticalScrollBar();
	sb->setValue(sb->maximum());
}

void MainWindow::consoleClear()
{
	ui._console->clear();
}

void MainWindow::clearAllData()
{
	_calThread->doClearAllData();
	_calThread->activate();
}

void MainWindow::commitROS_clicked()
{
	_calThread->doCommitToROS();
	_calThread->activate();

}

void MainWindow::commitHandEyeROS_clicked()
{
	std::cout << "Commit Hand eye result to ROS!!" << std::endl;

}

void MainWindow::updateImgs()
{
	if(tab == dti::HAND){
		updateImgHandEye();
	}else{
		updateImgL();
		updateImgR();
	}
}

void MainWindow::updateImgL()
{
	if(tab == dti::TAB_HANDEYE){
		updateImgHandEye();

	}else{
		QImage imgQ, imgQ_ = _sharedData->getImgL();

		int pW = _previewSize.width(), pH = _previewSize.height();
		int iW = imgQ_.size().width(), iH = imgQ_.size().height();
		if(pW < iW || pH < iH) {
			imgQ = imgQ_.scaled(_previewSize,  Qt::KeepAspectRatio, Qt::FastTransformation);
		}
		else
			imgQ = imgQ_;
		ui.imgLabelL->setPixmap(QPixmap::fromImage(imgQ));
	}
}

void MainWindow::updateImgR()
{
	QImage imgQ, imgQ_ = _sharedData->getImgR();

	int pW = _previewSize.width(), pH = _previewSize.height();
	int iW = imgQ_.size().width(), iH = imgQ_.size().height();
	if(pW < iW || pH < iH) {
		imgQ = imgQ_.scaled(_previewSize,  Qt::KeepAspectRatio, Qt::FastTransformation);
	} else
		imgQ = imgQ_;
	ui.imgLabelR->setPixmap(QPixmap::fromImage(imgQ));
}

void MainWindow::updateImgHandEye()
{
	QImage imgQ, imgQ_ = _sharedData->getImgHandEye();

	int pW = _previewSize.width(), pH = _previewSize.height();
	int iW = imgQ_.size().width(), iH = imgQ_.size().height();
	if(pW < iW || pH < iH) {
		imgQ = imgQ_.scaled(_previewSize,  Qt::KeepAspectRatio, Qt::FastTransformation);
	}
	else
		imgQ = imgQ_;
	ui.imgLabelL->setPixmap(QPixmap::fromImage(imgQ));
}

void MainWindow::updateNameFilterL(QString filter)
{
	QStringList nameFilter = filter.split(" ");
	_sharedData->setNameFilter(nameFilter, dti::LEFT);

	//Renew all paths and images!!
	updatePaths();
	updateImageLists();
	updateImageView();
}

void MainWindow::updateNameFilterR(QString filter)
{
	QStringList nameFilter = filter.split(" ");
	_sharedData->setNameFilter(nameFilter, dti::RIGHT);

	//Renew all paths and images!!
	updatePaths();
	updateImageLists();
	updateImageView();
}

void MainWindow::updateNameFilterHandEye(QString filter)
{
	QStringList nameFilter = filter.split(" ");
	_sharedData->setNameFilter(nameFilter, dti::HAND);

	//Renew all paths and images!!
	updatePaths();
	updateImageLists();
	updateImageView();
}

void MainWindow::updateBoardType(int index)
{
	if(tab == dti::TAB_HANDEYE){
		if(index == dti::AR_MARKER)
		{
			ui.spinBoxBoardSizeY_2->setEnabled(false);
			ui.spinBoxBoardSizeX_2->setEnabled(false);
			ui._doDetectCornersBtn_2->setText("Detect Marker");
			ui._doComputeCamPoseBtn_2->setEnabled(false);

		}else
		{
			ui.spinBoxBoardSizeY_2->setEnabled(true);
			ui.spinBoxBoardSizeX_2->setEnabled(true);
			ui._doDetectCornersBtn_2->setText("Detect corners");
			ui._doComputeCamPoseBtn_2->setEnabled(true);
		}
		_sharedData->setBoardTypeHandEye((dti::CalBoardTypes) index);
	}else
	{

		_sharedData->setBoardType((dti::CalBoardTypes) index);
	}
}

void MainWindow::updateSquareSize()
{
	QDoubleSpinBox* sp = (QDoubleSpinBox*) sender();
	double val = 1.0;
	if(sp == ui._squarePhysicalSize_SPINB) {
		 val= sp->value();
		_sharedData->setSquareSize(val);
	} else if(sp == ui._squarePhysicalSize_SPINB_2)
	{
		val = sp->value();
		_sharedData->setSquareSizeHandEye(val);
	}else if(sp == NULL)
	{
		val = ui._squarePhysicalSize_SPINB->value();
		_sharedData->setSquareSize(val);
	}
	else
        Q_EMIT consoleOutSig("unhandled sender in 'updateSquareSize'\n", dti::VerboseLevel::WARN);
}

void MainWindow::updateBoardSize(int val)
{
	QSpinBox* sp = (QSpinBox*) sender();
	if(sp == ui.spinBoxBoardSizeX)
		_sharedData->setBoardSizeX(val);
	else if (sp == ui.spinBoxBoardSizeY)
		_sharedData->setBoardSizeY(val);
	else if (sp == ui.spinBoxBoardSizeX_2)
		_sharedData->setBoardSizeXHandEye(val);
	else if (sp == ui.spinBoxBoardSizeY_2)
		_sharedData->setBoardSizeYHandEye(val);
    else Q_EMIT consoleOutSig("unhandled sender in 'updateBoardSize'\n", dti::VerboseLevel::WARN);
}

void MainWindow::showFlagsGroupBox(bool status)
{
	QGroupBox* box = (QGroupBox*) sender();
	int val = 15;
	if(status)
		val = 1000;
	box->setMaximumHeight(val);
}

void MainWindow::updateRoi() {
	QPushButton *btn = (QPushButton *) sender();
	QRect rect;
	if (btn == _annotationDialog->getAnnotationDialog()->btnOk) {
		if (_annotationDialog != NULL) {
			rect = _annotationDialog->getRoi();
			_sharedData->setRoi(rect);
			delete _annotationDialog;
			_annotationDialog = NULL;
		}
	} else if (btn == _annotationDialog->getAnnotationDialog()->btnSkip) {
		if (_annotationDialog != NULL) {
			rect = QRect(0, 0, 0, 0);
			_sharedData->setRoi(rect);
			delete _annotationDialog;
			_annotationDialog = NULL;
		}
	}
}

void MainWindow::annotateRoiInImage(QImage imgQ, QString title)
{
	// display in seperate window/dialog..
	if(_annotationDialog == NULL) {
		_annotationDialog = new dti::ImgAnnotation(this, _sharedData, title);
		// connect btns from annotation
		connect(_annotationDialog->getAnnotationDialog()->btnOk, SIGNAL(clicked()), this, SLOT(updateRoi()), Qt::UniqueConnection);
		// connect cancel btn
		connect(_annotationDialog->getAnnotationDialog()->btnSkip, SIGNAL(clicked()), this, SLOT(updateRoi()), Qt::UniqueConnection);
	}



	_annotationDialog->setVisible(true);
	_annotationDialog->updateImage(imgQ);
}

void MainWindow::updateAnnotationType(int index)
{
//	_sharedData->setCornerFlags()
}

void MainWindow::updateGuiIntrinsicVals()
{
	dti::IntrinsicGuessVals valsL =_sharedData->getIntrinsicGuessValsL();
	dti::IntrinsicGuessVals valsR =_sharedData->getIntrinsicGuessValsR();
	updateIntrinsicGuessValuesL(valsL);
	updateIntrinsicGuessValuesR(valsR);
}

void MainWindow::updateIntrinsicGuessValuesR(dti::IntrinsicGuessVals vals)
{
	ui._fixPrincipalPoint_VALx_CR->setText(QString::number(vals._principalPoint[0], 'g', 20));
	ui._fixPrincipalPoint_VALy_CR->setText(QString::number(vals._principalPoint[1], 'g', 20));

	ui._fixFocalLength_VALx_CR->setText(QString::number(vals._focalLength[0], 'g', 20));
	ui._fixFocalLength_VALy_CR->setText(QString::number(vals._focalLength[1], 'g', 20));

	ui._fix_K1_VAL_CR->setText(QString::number(vals._radiadistortion[0], 'g', 20));
	ui._fix_K2_VAL_CR->setText(QString::number(vals._radiadistortion[1], 'g', 20));
	ui._fix_K3_VAL_CR->setText(QString::number(vals._radiadistortion[2], 'g', 20));
	ui._fix_K4_VAL_CR->setText(QString::number(vals._radiadistortion[3], 'g', 20));
	ui._fix_K5_VAL_CR->setText(QString::number(vals._radiadistortion[4], 'g', 20));
	ui._fix_K6_VAL_CR->setText(QString::number(vals._radiadistortion[5], 'g', 20));

	ui._fix_P1_VAL_CR->setText(QString::number(vals._tangentialDistortion[0], 'g', 20));
	ui._fix_P2_VAL_CR->setText(QString::number(vals._tangentialDistortion[1], 'g', 20));
}

void MainWindow::updateIntrinsicGuessValuesL(dti::IntrinsicGuessVals vals)
{
	ui._fixPrincipalPoint_VALx_CL->setText(QString::number(vals._principalPoint[0], 'g', 20));
	ui._fixPrincipalPoint_VALy_CL->setText(QString::number(vals._principalPoint[1], 'g', 20));

	ui._fixFocalLength_VALx_CL->setText(QString::number(vals._focalLength[0], 'g', 20));
	ui._fixFocalLength_VALy_CL->setText(QString::number(vals._focalLength[1], 'g', 20));

	ui._fix_K1_VAL_CL->setText(QString::number(vals._radiadistortion[0], 'g', 20));
	ui._fix_K2_VAL_CL->setText(QString::number(vals._radiadistortion[1], 'g', 20));
	ui._fix_K3_VAL_CL->setText(QString::number(vals._radiadistortion[2], 'g', 20));
	ui._fix_K4_VAL_CL->setText(QString::number(vals._radiadistortion[3], 'g', 20));
	ui._fix_K5_VAL_CL->setText(QString::number(vals._radiadistortion[4], 'g', 20));
	ui._fix_K6_VAL_CL->setText(QString::number(vals._radiadistortion[5], 'g', 20));

	ui._fix_P1_VAL_CL->setText(QString::number(vals._tangentialDistortion[0], 'g', 20));
	ui._fix_P2_VAL_CL->setText(QString::number(vals._tangentialDistortion[1], 'g', 20));

}

void MainWindow::updateTangentDistortion(bool val){
	dti::IntrinsicGuessVals intrinsic;
	if(val == true)
	{

		//set felter = 0;
		ui._fix_P1_VAL_CL->setText(QString::number(0.0));
		ui._fix_P2_VAL_CL->setText(QString::number(0.0));
		ui._fix_P1_VAL_CR->setText(QString::number(0.0));
		ui._fix_P2_VAL_CR->setText(QString::number(0.0));
		ui._fix_P1_VAL_CL->setEnabled(false);
		ui._fix_P2_VAL_CL->setEnabled(false);
		ui._fix_P1_VAL_CR->setEnabled(false);
		ui._fix_P2_VAL_CR->setEnabled(false);
	} else
	{

		intrinsic = _sharedData->getIntrinsicGuessValsL();
		ui._fix_P1_VAL_CL->setEnabled(true);
		ui._fix_P2_VAL_CL->setEnabled(true);
		ui._fix_P1_VAL_CL->setText(QString::number(intrinsic._tangentialDistortion[0], 'g', 20));
		ui._fix_P2_VAL_CL->setText(QString::number(intrinsic._tangentialDistortion[1], 'g', 20));

		intrinsic = _sharedData->getIntrinsicGuessValsR();
		ui._fix_P1_VAL_CR->setEnabled(true);
		ui._fix_P2_VAL_CR->setEnabled(true);
		ui._fix_P1_VAL_CR->setText(QString::number(intrinsic._tangentialDistortion[0], 'g', 20));
		ui._fix_P2_VAL_CR->setText(QString::number(intrinsic._tangentialDistortion[1], 'g', 20));
	}
}

void MainWindow::updateRectificationGUI(int index)
{
	QComboBox* src;

	try {
		src = (QComboBox*) sender();
	} catch (std::exception& e) {
        Q_EMIT consoleOutSig("Error in CalTI::updateRectificationAlgo(int index):\n  ", dti::VerboseLevel::WARN);
        Q_EMIT consoleOutSig(QString(e.what()), dti::VerboseLevel::WARN);
		return;
	}

	bool val = true;
	if(src == ui._rectAlgorithm_Combo)
	{
		switch (index) {
			case dti::RE_Bougets:
				//				hide Fundamental; (and param1+2)
				val = false;
				ui._rectFundamentalLabel->setVisible(val);
				ui._rectFundamental_Combo->setVisible(val);
				ui._rectParam1_VAL->setVisible(val);
				ui._rectParam2_VAL->setVisible(val);
				ui._rectParam1_label->setVisible(val);
				ui._rectParam2_label->setVisible(val);
			break;

			case dti::RE_Hartleys:
//				show Fundamental; (and (maybe) param1+2)
				val = true;
				ui._rectFundamentalLabel->setVisible(val);
				ui._rectFundamental_Combo->setVisible(val);
				ui._rectParam1_VAL->setVisible(val);
				ui._rectParam2_VAL->setVisible(val);
				ui._rectParam1_label->setVisible(val);
				ui._rectParam2_label->setVisible(val);
				break;
			default:
                Q_EMIT consoleOutSig(" --> Unhandled index from 'ui._rectAlgorithm_Combo' in CalTI::updateRectificationGUI()", dti::VerboseLevel::ERROR);
				break;
		}
	} else if (src == ui._rectFundamental_Combo) {

		switch (index) {
			case dti::RE_FM_Points_7:
			case dti::RE_FM_Points_8:
				// show none of the params
				val = false;
				ui._rectParam1_VAL->setEnabled(val);
				ui._rectParam1_label->setEnabled(val);
				ui._rectParam2_VAL->setEnabled(val);
				ui._rectParam2_label->setEnabled(val);
				break;

			case dti::RE_FM_LeastMedian:
				// show param2 and not param2
				val = false;
				ui._rectParam1_VAL->setEnabled(false);
				ui._rectParam1_label->setEnabled(false);

				ui._rectParam2_VAL->setEnabled(true);
				ui._rectParam2_label->setEnabled(true);
				break;

			case dti::RE_FM_RANSAC:
				// show both params
				val = true;
				ui._rectParam1_VAL->setEnabled(val);
				ui._rectParam1_label->setEnabled(val);

				ui._rectParam2_VAL->setEnabled(val);
				ui._rectParam2_label->setEnabled(val);
				break;
			default:
                Q_EMIT consoleOutSig(" --> Unhandled index from ui._rectFundamental_Combo in CalTI::updateRectificationGUI()", dti::VerboseLevel::ERROR);
				break;
		}
	}
}

void MainWindow::updateRectSavePath()
{
	QString path;
	path = ui._rectificationSavePathTxtL->text();
	_sharedData->setRectSavePath(path, dti::LEFT);
	path = ui._rectificationSavePathTxtR->text();
	_sharedData->setRectSavePath(path, dti::RIGHT);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	writeSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::updateRobotPose(const tiv::pose& pose, int cam)
{
	_sharedData->setRobotPose(pose);
}

void MainWindow::updateImage(const QImage& img, int cam)
{
	if(onlineImages)
	{
		//Scale the image to fit the stream label
		QImage qImg = img.scaled(_previewSize, Qt::KeepAspectRatio, Qt::FastTransformation);
		if(cam == dti::LEFT || cam == dti::HAND )
				ui.imgLabelL->setPixmap(QPixmap::fromImage(qImg));
		else
				ui.imgLabelR->setPixmap(QPixmap::fromImage(qImg));

		if( saveNextImage[cam] )
		{

			saveNextImage[cam] = false;
			QStringList nameFilter = _sharedData->getNameFilter((dti::CameraID) cam);
			QRegExp rx("(\\*)");
			if( nameFilter.count() == 0)
			{
                Q_EMIT consoleOutSig("ERROR: No name filter specified", dti::VerboseLevel::WARN);
				return;
			}

			QStringList nameFilterSplit = nameFilter.at(0).split(rx);
			if( nameFilterSplit.count() != 2)
			{
                Q_EMIT consoleOutSig("ERROR: Wrong image filter. Use e.g. \"left*.jpg\"", dti::VerboseLevel::WARN);
				return;
			}
			QString prefix = nameFilterSplit.at(0);
			QString postfix = nameFilterSplit.at(1);

			QString imagePathString;
			if(cam == dti::LEFT){
				imagePathString = _sharedData->getImgPathL();
			std::cout << "left" << std::endl;
			}
			else if(cam == dti::HAND){
				imagePathString = _sharedData->getImgPathHandEyeMono();
				std::cout << "Hand" << std::endl;
			}
			else{
				imagePathString = _sharedData->getImgPathR();
				std::cout << "right" << std::endl;
			}

			QDir imgPath = imagePathString;
			if(imgPath.exists() == false)
			{
				QDir().mkdir(imagePathString);
			}
			imgPath.setNameFilters(nameFilter);
			QStringList fileList = imgPath.entryList();
			int lowestIndex = fileList.count()+1;

			for(int i = 0; i < fileList.count(); i++)
			{
				QString temp = fileList.at(i);
				if(temp.startsWith(prefix) && temp.endsWith(postfix))
				{
					temp = temp.remove(prefix,Qt::CaseSensitive);
					temp = temp.remove(postfix,Qt::CaseSensitive);
					bool conv;
					int number = temp.toInt(&conv, 10);
					if( (number != i+1) && (i+1 < lowestIndex) )
					{
						lowestIndex = i+1;
					}
				}
			}
			QString num = QString("%1").arg(lowestIndex, 3, 10, QChar('0')).toUpper();
			imagePathString.append("/");
			imagePathString.append(prefix);
			imagePathString.append(num);
			imagePathString.append(postfix);
			img.save(imagePathString);
            Q_EMIT consoleOutSig(imagePathString.prepend("Image added to list: ").append("\r\n"), dti::VerboseLevel::INFO);
			updateImageLists();
		}
	}
}

void MainWindow::updateImageLists()
{
	QStringList nameFilter;
	QDir imgPath;
	if(tab == dti::TAB_HANDEYE){
		nameFilter = _sharedData->getNameFilter(dti::HAND);
		imgPath = _sharedData->getImgPathHandEyeMono();
	}
	else{
		nameFilter = _sharedData->getNameFilter(dti::LEFT);
		imgPath = _sharedData->getImgPathL();
	}

	imgPath.setNameFilters(nameFilter);

	QStringList fileList = imgPath.entryList();
	int tempCount = ui.comboBoxImageCurrent->count();
	QString currentText = ui.comboBoxImageCurrent->currentText();
	int listSize = fileList.count();
	if(listSize == 0)
	{
		imageFolderEmpty = true;
        Q_EMIT consoleOutSig("No images in this folder with specified image filter.\r\n", dti::VerboseLevel::INFO);
		return;
	}
	else
	{
		imageFolderEmpty = false;
	}
	ui.comboBoxImageCurrent->setEnabled(false);
	ui.comboBoxImageCurrent->insertItems(tempCount,fileList);
	for(int i = 0; i < tempCount; i++)
		ui.comboBoxImageCurrent->removeItem(0);
	ui.comboBoxImageCurrent->setEnabled(true);

	int newIndex = ui.comboBoxImageCurrent->findText(currentText);
	if(newIndex != -1)
		ui.comboBoxImageCurrent->setCurrentIndex(newIndex);
	else if( ui.comboBoxImageCurrent->count() == 0 )
		ui.comboBoxImageCurrent->setCurrentIndex(-1);
	else
		ui.comboBoxImageCurrent->setCurrentIndex(0);
}

void MainWindow::updateImageView()
{
	if( onlineImages == false && imageFolderEmpty == false )
	{
		QString imgPath;
		if(tab == dti::TAB_HANDEYE){
			ui.imgLabelL->setVisible(true);
			ui.imgLabelR->setVisible(false);
			imgPath= ui.comboBoxImageCurrent->currentText().prepend(_sharedData->getImgPathHandEyeMono() + "/");
		}else{
			imgPath= ui.comboBoxImageCurrent->currentText().prepend(_sharedData->getImgPathL() + "/");
		}


		QImage imgL(imgPath);
		if(!imgL.isNull())
		{
			imgL = imgL.scaled(_previewSize,  Qt::KeepAspectRatio, Qt::FastTransformation);
			ui.imgLabelL->setPixmap(QPixmap::fromImage(imgL));

			if(calType == dti::StereoCalibration_offline || calType == dti::StereoCalibration_online)
			{
				QStringList nameFilter = _sharedData->getNameFilter(dti::RIGHT);
				QDir imgDir = _sharedData->getImgPathR();
				imgDir.setNameFilters(nameFilter);

				QStringList fileList = imgDir.entryList();
				int index = ui.comboBoxImageCurrent->currentIndex();
				if( index < fileList.size())
				{
					imgPath = fileList[ui.comboBoxImageCurrent->currentIndex()].prepend(_sharedData->getImgPathR() + "/");
					QImage imgR(imgPath);
					if(!imgR.isNull())
					{
						imgR = imgR.scaled(_previewSize,  Qt::KeepAspectRatio, Qt::FastTransformation);
						ui.imgLabelR->setPixmap(QPixmap::fromImage(imgR));
					}
				}
			}
		}

	}
}

void MainWindow::on_comboBoxImageCurrent_currentIndexChanged(int i)
{
	updateImageView();
}

void MainWindow::on_pushButtonImageDelete_clicked()
{
	if( imageFolderEmpty == false )
	{
		QString imgPath = ui.comboBoxImageCurrent->currentText().prepend(_sharedData->getImgPathL() + "/");
		remove(imgPath.toUtf8().constData());
        Q_EMIT consoleOutSig(imgPath.prepend("Image removed from list: ").append("\r\n"), dti::VerboseLevel::INFO);

		if( calType == dti::StereoCalibration_online || calType == dti::StereoCalibration_offline )
		{
				QStringList nameFilter = _sharedData->getNameFilter(dti::RIGHT);
				QDir imgDir = _sharedData->getImgPathR();
				imgDir.setNameFilters(nameFilter);

				QStringList fileList = imgDir.entryList();
				imgPath = fileList[ui.comboBoxImageCurrent->currentIndex()].prepend(_sharedData->getImgPathR() + "/");
				remove(imgPath.toUtf8().constData());
                Q_EMIT consoleOutSig(imgPath.prepend("Image removed from list: ").append("\r\n"),dti::VerboseLevel::INFO);
		}
		updateImageLists();
	}
}

void MainWindow::on_pushButtonStreamStart_clicked()
{
#ifdef USE_ROS
	onlineImages = true;
	std::string leftTopic = ui.lineEditImgTopicL->text().toStdString();

	if( calType == dti::StereoCalibration_online)
	{
		std::cout << "Starting stereo stream!" << std::endl;
		std::string rightTopic = ui.lineEditImgTopicR->text().toStdString();
		qnode.startStream(leftTopic,rightTopic);
	}else if( calType == dti::MonoCalibration_online)
	{
		std::cout << "Starting mono stream!" << std::endl;
		qnode.startStream(leftTopic);
	}else
	{	std::cout << "Starting mono and tcp  stream!" << std::endl;
		std::string tcpTopic = ui.lineEditImgTopicR->text().toStdString();
		qnode.startStreamWithTCP(leftTopic,tcpTopic);
	}

	ui.pushButtonStreamStop->setEnabled(true);
	ui.pushButtonStreamStart->setEnabled(false);
//#else
  //  QMessageBox::Yes reply;
  //  reply = QMessageBox::question(this, "Cannot stream images without ROS installed!", "No ROS",QMessageBox::Ok);
//    std::cout << "Cannot stream images without ROS installed!" << std::endl;

#endif
}

void MainWindow::on_pushButtonStreamStop_clicked()
{
#ifdef USE_ROS
	onlineImages = false;
	qnode.stopStream();
	ui.pushButtonStreamStop->setEnabled(false);
	ui.pushButtonStreamStart->setEnabled(true);
	updateImageView();
 //#else
    //QMessageBox::Yes reply;
    //  reply = QMessageBox::question(this, "Cannot stream images without ROS installed!", "No ROS",QMessageBox::Ok);
  //  std::cout << "Cannot stream images without ROS installed!" << std::endl;
#endif

}

void MainWindow::on_pushButtonStreamSaveImage_clicked()
{
	if(tab == dti::TAB_HANDEYE){
		if(calType == dti::HandEyeCalibration_online){
			saveNextImage[dti::HAND] = true;
			tiv::pose p = _sharedData->getRobotPose();

			std::cout << "X: " <<  p.t.x << "Y: " << p.t.y << "Z: " << p.t.z << std::endl;
			_sharedData->addRobotPose2Array(_sharedData->getRobotPose());
			updateRobotPoseCount();
		}
	}else{
		saveNextImage[dti::LEFT] = true;
		if(calType == dti::StereoCalibration_online)
			saveNextImage[dti::RIGHT] = true;
	}
}

void MainWindow::on_lineEditNarrowIntrinsicLeft_editingFinished()
{
	_sharedData->setNarrowIntrinsicLeftPath(ui.lineEditNarrowIntrinsicLeft->text());
}

void MainWindow::on_lineEditNarrowIntrinsicRight_editingFinished()
{
	_sharedData->setNarrowIntrinsicRightPath(ui.lineEditNarrowIntrinsicRight->text());
}

void MainWindow::on_lineEditNarrowImageLeft_editingFinished()
{
	_sharedData->setNarrowImageLeftPath(ui.lineEditNarrowImageLeft->text());
}

void MainWindow::on_lineEditNarrowImageRight_editingFinished()
{
	_sharedData->setNarrowImageRightPath(ui.lineEditNarrowImageRight->text());
}

/*void MainWindow::on_spinBoxNarrowBoardSizeX_valueChanged(int i)
{
	_sharedData->setNarrowBoardSizeX(i);
}

void MainWindow::on_spinBoxNarrowBoardSizeY_valueChanged(int i)
{
	_sharedData->setNarrowBoardSizeY(i);
}
*/
void MainWindow::on_lineEditNarrowOutputLeft_editingFinished()
{
	_sharedData->setNarrowOutputLeftPath(ui.lineEditNarrowOutputLeft->text());
}

void MainWindow::on_lineEditNarrowOutputRight_editingFinished()
{
	_sharedData->setNarrowOutputRightPath(ui.lineEditNarrowOutputRight->text());
}

void MainWindow::readNarrowStereoParameters()
{
	on_lineEditNarrowIntrinsicLeft_editingFinished();
	on_lineEditNarrowIntrinsicRight_editingFinished();
	on_lineEditNarrowImageLeft_editingFinished();
	on_lineEditNarrowImageRight_editingFinished();
	//on_spinBoxNarrowBoardSizeX_valueChanged(ui.spinBoxNarrowBoardSizeX->value());
	//on_spinBoxNarrowBoardSizeY_valueChanged(ui.spinBoxNarrowBoardSizeY->value());
	on_lineEditNarrowOutputLeft_editingFinished();
	on_lineEditNarrowOutputRight_editingFinished();
}

void MainWindow::on_pushButtonNarrowCompute_clicked()
{
	readFlags();

	QString leftPath = _sharedData->getNarrowImageLeftPath();
	QString rightPath = _sharedData->getNarrowImageRightPath();

	int i = leftPath.lastIndexOf("/",-1,Qt::CaseInsensitive);
	QString leftName = leftPath.right(leftPath.length()-1 -i);

	int index = ui.comboBoxImageCurrent->findText(leftName);
	ui.comboBoxImageCurrent->setCurrentIndex(index);
	updateImageView();

	_calThread->doNarrowStereoCalib();

	// disable all but cancel btn
	ui._doAllBtn->setEnabled(false);
	ui._doDetectCornersBtn->setEnabled(false);
	ui._doCalibrateBtn->setEnabled(false);
	ui._doRectifyBtn->setEnabled(false);
	ui._doSaveResultsBtn->setEnabled(false);

	_calThread->activate();
}

}  // namespace CalTI

