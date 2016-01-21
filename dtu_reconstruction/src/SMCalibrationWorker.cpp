#include "SMCalibrationWorker.h"
#include "SMCalibrationParameters.h"

#include "cvtools.h"

#include <QSettings>

void SMCalibrationWorker::performCalibration(std::vector<SMCalibrationSet> calibrationData, std::string file_path){

    QSettings settings;

    // Number of saddle points on calibration pattern
    int checkerCountX = settings.value("calibration/checkerCountX", 12).toInt();
    int checkerCountY = settings.value("calibration/checkerCountY", 11).toInt();
    cv::Size checkerCount(checkerCountX, checkerCountY);
    std::cout << "checkerCountX: " << checkerCountX << " checkerCountY: " << checkerCountY << std::endl;
    int nSets = calibrationData.size();

    std::vector< std::vector<cv::Point2f> > qc0, qc1;
    std::vector< std::vector<cv::Point2f> > qc0Stereo, qc1Stereo;

    std::vector<float> angles;

    // Loop through calibration sets
    for(int i=0; i<nSets; i++){

        SMCalibrationSet SMCalibrationSetI = calibrationData[i];

        if(!SMCalibrationSetI.checked)
            continue;

        // Camera 0
        std::vector<cv::Point2f> qci0;

        // Convert to grayscale
        cv::Mat gray;
        if(SMCalibrationSetI.frame0.channels() == 1){
            cv::cvtColor(SMCalibrationSetI.frame0, gray,  CV_BayerBG2GRAY);
       } else
            cv::cvtColor(SMCalibrationSetI.frame0, gray, CV_RGB2GRAY);


        // Extract checker corners
        bool success0 = cv::findChessboardCorners(gray, checkerCount, qci0, cv::CALIB_CB_ADAPTIVE_THRESH); //+ cv::CALIB_CB_FAST_CHECK
        if(success0){
            std::cout << "Found chessboard corners for left image " << i << std::endl;
            cv::cornerSubPix(gray, qci0, cv::Size(6, 6), cv::Size(1, 1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 20, 0.0001));
            // Draw colored chessboard
            cv::Mat color;
            if(SMCalibrationSetI.frame0.channels() == 1)
                cv::cvtColor(SMCalibrationSetI.frame0, color, CV_BayerBG2RGB);
            else
                color = SMCalibrationSetI.frame0.clone();

            cvtools::drawChessboardCorners(color, checkerCount, qci0, success0, 10);
            SMCalibrationSetI.frame0Result = color;
        }else
            std::cerr << "Could not find chessboard corners!!" << std::endl;

        emit newFrameResult(i, 0, success0, SMCalibrationSetI.frame0Result);

        // Camera 1
        std::vector<cv::Point2f> qci1;

        // Convert to grayscale
        if(SMCalibrationSetI.frame1.channels() == 1)
            cv::cvtColor(SMCalibrationSetI.frame1, gray, CV_BayerBG2GRAY);
        else
            cv::cvtColor(SMCalibrationSetI.frame1, gray, CV_RGB2GRAY);

        // Extract checker corners
        bool success1 = cv::findChessboardCorners(gray, checkerCount, qci1, cv::CALIB_CB_ADAPTIVE_THRESH); //+ cv::CALIB_CB_FAST_CHECK cv::CALIB_CB_ADAPTIVE_THRESH +
        if(success1){
            std::cout << "Found chessboard corners for right image " << i << std::endl;
            cv::cornerSubPix(gray, qci1, cv::Size(6, 6), cv::Size(1, 1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 20, 0.0001));
            // Draw colored chessboard
            cv::Mat color;
            if(SMCalibrationSetI.frame1.channels() == 1)
                cv::cvtColor(SMCalibrationSetI.frame1, color, CV_BayerBG2RGB);
            else
                color = SMCalibrationSetI.frame1.clone();

            cvtools::drawChessboardCorners(color, checkerCount, qci1, success1, 10);
            SMCalibrationSetI.frame1Result = color;
        }else
            std::cerr << "Could not find chessboard corners!!" << std::endl;

        emit newFrameResult(i, 1, success1, SMCalibrationSetI.frame1Result);

        if(success0)
            qc0.push_back(qci0);

        if(success1)
            qc1.push_back(qci1);

        if(success0 && success1){
            qc0Stereo.push_back(qci0);
            qc1Stereo.push_back(qci1);
            angles.push_back(SMCalibrationSetI.rotationAngle);
        }

        // Show progress
        emit newSetProcessed(i);
    }

    int nValidSets = angles.size();
    if(nValidSets < 2){
        std::cerr << "Not enough valid calibration sequences!" << std::endl;
        emit done();
        return;
    }

    // Generate world object coordinates [mm]
    float checkerSize = settings.value("calibration/checkerSize", 10.0).toFloat(); // width and height of one field in mm
    std::vector<cv::Point3f> Qi;
    for (int h=0; h<checkerCount.height; h++)
        for (int w=0; w<checkerCount.width; w++)
            Qi.push_back(cv::Point3f(checkerSize * w, checkerSize* h, 0.0));

    std::vector< std::vector<cv::Point3f> > Q0, Q1, QStereo;
    for(unsigned int i=0; i<qc0.size(); i++)
        Q0.push_back(Qi);
    for(unsigned int i=0; i<qc1.size(); i++)
        Q1.push_back(Qi);
    for(int i=0; i<nValidSets; i++)
        QStereo.push_back(Qi);

    // calibrate the cameras
    SMCalibrationParameters cal;
    cal.frameWidth = calibrationData[0].frame0.cols;
    cal.frameHeight = calibrationData[0].frame0.rows;
    cv::Size frameSize(cal.frameWidth, cal.frameHeight);

    // determine only k1, k2 for lens distortion
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    // Note: several of the output arguments below must be cv::Mat, otherwise segfault
    std::vector<cv::Mat> cam_rvecs0, cam_tvecs0;
    cal.cam0_error = cv::calibrateCamera(Q0, qc0, frameSize, cal.K0, cal.k0, cam_rvecs0, cam_tvecs0, flags,
                                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON));
//std::cout << cal.k0 << std::endl;
//    // refine extrinsics for camera 0
//    for(int i=0; i<Q.size(); i++)
//        cv::solvePnPRansac(Q[i], qc0[i], cal.K0, cal.k0, cam_rvecs0[i], cam_tvecs0[i], true, 100, 0.05, 100, cv::noArray(), CV_ITERATIVE);

    std::vector<cv::Mat> cam_rvecs1, cam_tvecs1;
    cal.cam1_error = cv::calibrateCamera(Q1, qc1, frameSize, cal.K1, cal.k1, cam_rvecs1, cam_tvecs1, flags,
                                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON));
//std::cout << cal.k1 << std::endl;
    // stereo calibration
    int flags_stereo = cv::CALIB_FIX_INTRINSIC;// + cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_ASPECT_RATIO;
    cv::Mat E, F, R1, T1;
    cal.stereo_error = cv::stereoCalibrate(QStereo, qc0Stereo, qc1Stereo, cal.K0, cal.k0, cal.K1, cal.k1,
                                              frameSize, R1, T1, E, F,
                                              cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, DBL_EPSILON),
                                              flags_stereo);

    cal.R1 = R1;
    cal.T1 = T1;
    cal.E = E;
    cal.F = F;




    // Direct rotation axis calibration //
    // full camera matrices
 /*   cv::Matx34f P0 = cv::Matx34f::eye();
    cv::Mat RT1(3, 4, CV_32F);
    cv::Mat(cal.R1).copyTo(RT1(cv::Range(0, 3), cv::Range(0, 3)));
    cv::Mat(cal.T1).copyTo(RT1(cv::Range(0, 3), cv::Range(3, 4)));
    cv::Matx34f P1 = cv::Matx34f(RT1);

    // calibration points in camera 0 frame
    std::vector< std::vector<cv::Point3f> > Qcam;

    for(int i=0; i<nValidSets; i++){
        std::vector<cv::Point2f> qc0i, qc1i;

        cv::undistortPoints(qc0[i], qc0i, cal.K0, cal.k0);
        cv::undistortPoints(qc1[i], qc1i, cal.K1, cal.k1);
//        qc0i = qc0[i];
//        qc1i = qc1[i];

        cv::Mat Qhom, Qcami;
        cv::triangulatePoints(P0, P1, qc0i, qc1i, Qhom);
        cvtools::convertMatFromHomogeneous(Qhom, Qcami);
        std::vector<cv::Point3f> QcamiPoints;
        cvtools::matToPoints3f(Qcami, QcamiPoints);

        Qcam.push_back(QcamiPoints);
    }

    cv::Vec3f axis, point;
    cvtools::rotationAxisCalibration(Qcam, Qi, axis, point);

    // construct transformation matrix
    cv::Vec3f ex = axis.cross(cv::Vec3f(0,0,1.0));
    ex = cv::normalize(ex);
    cv::Vec3f ez = ex.cross(axis);
    ez = cv::normalize(ez);

    cv::Mat RrMat(3, 3, CV_32F);
    cv::Mat(ex).copyTo(RrMat.col(0));
    cv::Mat(axis).copyTo(RrMat.col(1));
    cv::Mat(ez).copyTo(RrMat.col(2));

    cal.Rr = cv::Matx33f(RrMat).t();
    cal.Tr = -cv::Matx33f(RrMat).t()*point;
*/
    // Print to std::cout
    cal.print();

    cal.exportToXML(QString::fromStdString(file_path));

    // save to (reentrant qsettings object)
    settings.setValue("calibration/parameters", QVariant::fromValue(cal));


    emit done();

}

