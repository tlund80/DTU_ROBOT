/**
 * @file /include/CalTI/main_window.hpp
 *
 * @brief Qt based gui for CalTI.
 *
 * @date November 2010
 **/
#ifndef CalTI_MAIN_WINDOW_H
#define CalTI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGlobal>

#if QT_VERSION >= 0x050000
    #include <QMainWindow>
#else
    #include <QtGui/QMainWindow>
#endif

#include "ui_main_window.h"
#ifdef USE_ROS
    #include "qnode.hpp"
#endif
#include "CalibThread.hpp"
#include "../../include/cal_ti/config.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace CalTI {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
    void consoleOut(QString msg, dti::VerboseLevel level);
	void consoleClear();
	void clearAllData();
	void commitROS_clicked();
	void commitHandEyeROS_clicked();
	void updateImgs();
	void updateGuiIntrinsicVals();
	void clearIntrinsicValsSlot();
	void updateImage(const QImage& img, int cam);

private Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	/* Narrow Stereo tab */
	void on_lineEditNarrowIntrinsicLeft_editingFinished();
	void on_lineEditNarrowIntrinsicRight_editingFinished();
	void on_lineEditNarrowImageLeft_editingFinished();
	void on_lineEditNarrowImageRight_editingFinished();
	//void on_spinBoxNarrowBoardSizeX_valueChanged(int i);
	//void on_spinBoxNarrowBoardSizeY_valueChanged(int i);
	void on_lineEditNarrowOutputLeft_editingFinished();
	void on_lineEditNarrowOutputRight_editingFinished();
	void on_pushButtonNarrowCompute_clicked();

	/* Image view */
	void on_comboBoxImageCurrent_currentIndexChanged(int i);
	void on_pushButtonImageDelete_clicked();

	/* Live stream */
	void on_pushButtonStreamStart_clicked();
	void on_pushButtonStreamStop_clicked();
	void on_pushButtonStreamSaveImage_clicked();

	/******************************************
	** Manual connections
	*******************************************/
	void updateRectSavePath();
	void updateRectificationGUI(int);
	void updateTangentDistortion(bool);
	void updateUseIntrinsic(bool);
	void updateIntrinsicGuessPath();
	void updateTab(int index);
	void updateRobotPoseCount();
//	void updateIntrinsicGuessPathL();
//	void updateIntrinsicGuessPathR();
	void updateTermCrit();
	void updateAnnotationType(int index);
	void updateSquareSize();
	void calThreadFinished();
	void selectPath();
	void updatePaths();
	void updateImgL();
	void updateImgR();
	void updateImgHandEye();
	void activateCalibrate();
	void processDoAllBtn();
	void processDoDetectCornersBtn();
	void processDoDetectCornersHandEyeBtn();
	void processDoCalibrateHandEyeBtn();
	void processDoCalibrateBtn();
	void processDoRectifyBtn();
	void processDoSaveBtn();
	void processDoSaveHandEyeBtn();
	void processDoCancelBtn();
	void processDoComputeCameraPosesBtn();
	void processDoClearRobotPoseBtn();
	void processDoSaveRobotPoseBtn();
	void processDoShowRobotPoseBtn();
	void updatePreviewSizeWidth(QString val);
	void updatePreviewSizeHeight(QString val);
	void updatePreviewSizeFields(int index);
	void updateNameFilterL(QString filter);
	void updateNameFilterR(QString filter);
	void updateNameFilterHandEye(QString filter);
	void updateCalType(int index);
	void updateHandEyeCalType(int index);
	void updateBoardType(int index);
	void updateBoardSize(int val);
	void showFlagsGroupBox(bool);
	void annotateRoiInImage(QImage imgQ, QString);
	void updateRoi();
	void updateImageView();
	void updateImageLists();
	void updateRobotPose(const tiv::pose& pose, int cam);
//	void processDeleteImage();
//	void processStartStream();
//	void processStopStream();
//	void processSaveImage();

	Q_SIGNALS:
    void consoleOutSig(QString msg, dti::VerboseLevel);
	void intrinsicGuessPath(dti::CameraID CAM, QString intrinsicMatName, QString distortionMatName);

private:
	void writeSettings();
	void readSettings();
	void init();
	void initGui();
	void clearIntrinsicVals(dti::CameraID CAM);
	void updateIntrinsicGuessValuesR(dti::IntrinsicGuessVals vals);
	void updateIntrinsicGuessValuesL(dti::IntrinsicGuessVals vals);
	void readFlags();
	void readCornerDetectionFlags();
	void readHandEyeCornerDetectionFlags();
	void readRobotFlags();
	void readCalFlags();
	void readStereoFlags();
	void readRectificationFlags();
	void readIntrinsicGuessVals();
	void readNarrowStereoParameters();


    Ui::MainWindowDesign ui;
#ifdef USE_ROS
	QNode qnode;
#endif

	dti::SharedData *_sharedData;
	dti::CalibThread *_calThread;
	dti::Calibration *_calibMono;
	dti::Calibration *_calibStereo;
	dti::Calibration *_calibHandEye;
    dti::ImgAnnotation  *_annotationDialog;

	dti::CalibrationTypes calType;
	QDir imgPathL, imgPathR;
	QSize _previewSize;
	bool imageFolderEmpty;
	bool onlineImages;
	bool saveNextImage[3];
	int tab;

};

}  // namespace CalTI

#endif // CalTI_MAIN_WINDOW_H
