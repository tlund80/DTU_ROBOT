#include "SMColorCalibrationParameters.h"

//SMColorCalibrationParameters::SMColorCalibrationParameters()
//{
//}


void SMColorCalibrationParameters::print(){

    std::cout << "Color Calibration Parameters:" << std::endl;
    std::cout << "Color Calibration Matrix Left: " << std::endl;
    std::cout << calibration_matrixL << std::endl;
    std::cout << "Color Calibration Matrix Right: " << std::endl;
    std::cout << calibration_matrixR << std::endl;
    std::cout << "Color centers Left: " << std::endl;
    std::cout << colorCentersL << std::endl;
    std::cout << "Color centers Right: " << std::endl;
    std::cout << colorCentersR << std::endl;

}

void SMColorCalibrationParameters::exportToXML(QString fileName){

    cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::WRITE);
    if (!fs.isOpened())
        std::cerr << "SMColorCalibrationParameters: could not save file!" << std::endl;

    fs << "CC_Left" << calibration_matrixL << "CC_Right" << calibration_matrixR
          << "CP_Left" << colorCentersL << "CP_Right" << colorCentersR;


    fs.release();

}

void SMColorCalibrationParameters::importFromXML(QString fileName){

    cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened())
        std::cerr << "SMColorCalibrationParameters: could not open file!" << std::endl;

    cv::Mat temp;
    fs["CC_Left"] >> temp; calibration_matrixL = temp;
    fs["CC_Right"] >> temp; calibration_matrixR = temp;
    fs["CP_Left"] >> temp; colorCentersL = temp;
    fs["CP_Right"] >> temp; colorCentersR = temp;

    fs.release();
}

// QStreamtypes for qDebug() and QSettings
QDataStream& operator>>(QDataStream& in, SMColorCalibrationParameters& data){
//    std::cout << "Deserializing color calibration parameters!" << std::endl;
    in >> data.calibration_matrixL.at<double>(0,0) >> data.calibration_matrixL.at<double>(0,1) >> data.calibration_matrixL.at<double>(0,2) >> data.calibration_matrixL.at<double>(1,0) >> data.calibration_matrixL.at<double>(1,1)
       >> data.calibration_matrixL.at<double>(1,2) >> data.calibration_matrixL.at<double>(2,0) >> data.calibration_matrixL.at<double>(2,1) >> data.calibration_matrixL.at<double>(2,2);

    in >> data.calibration_matrixR.at<double>(0,0) >> data.calibration_matrixR.at<double>(0,1) >> data.calibration_matrixR.at<double>(0,2) >> data.calibration_matrixR.at<double>(1,0) >> data.calibration_matrixR.at<double>(1,1)
       >> data.calibration_matrixR.at<double>(1,2) >> data.calibration_matrixR.at<double>(2,0) >> data.calibration_matrixR.at<double>(2,1) >> data.calibration_matrixR.at<double>(2,2);

    in >> data.colorCentersL.at<double>(0,0) >> data.colorCentersL.at<double>(1,0) >> data.colorCentersL.at<double>(2,0)
       >> data.colorCentersL.at<double>(0,1) >> data.colorCentersL.at<double>(1,1) >> data.colorCentersL.at<double>(2,1)
       >> data.colorCentersL.at<double>(0,2) >> data.colorCentersL.at<double>(1,2) >> data.colorCentersL.at<double>(2,2)
       >> data.colorCentersL.at<double>(0,3) >> data.colorCentersL.at<double>(1,3) >> data.colorCentersL.at<double>(2,3)
       >> data.colorCentersL.at<double>(0,4) >> data.colorCentersL.at<double>(1,4) >> data.colorCentersL.at<double>(2,4)
       >> data.colorCentersL.at<double>(0,5) >> data.colorCentersL.at<double>(1,5) >> data.colorCentersL.at<double>(2,5)
       >> data.colorCentersL.at<double>(0,6) >> data.colorCentersL.at<double>(1,6) >> data.colorCentersL.at<double>(2,6)
       >> data.colorCentersL.at<double>(0,7) >> data.colorCentersL.at<double>(1,7) >> data.colorCentersL.at<double>(2,7)
       >> data.colorCentersL.at<double>(0,8) >> data.colorCentersL.at<double>(1,8) >> data.colorCentersL.at<double>(2,8)
       >> data.colorCentersL.at<double>(0,9) >> data.colorCentersL.at<double>(1,9) >> data.colorCentersL.at<double>(2,9)
       >> data.colorCentersL.at<double>(0,10) >> data.colorCentersL.at<double>(1,10) >> data.colorCentersL.at<double>(2,10)
       >> data.colorCentersL.at<double>(0,11) >> data.colorCentersL.at<double>(1,11) >> data.colorCentersL.at<double>(2,11)
       >> data.colorCentersL.at<double>(0,12) >> data.colorCentersL.at<double>(1,12) >> data.colorCentersL.at<double>(2,12)
       >> data.colorCentersL.at<double>(0,13) >> data.colorCentersL.at<double>(1,13) >> data.colorCentersL.at<double>(2,13)
       >> data.colorCentersL.at<double>(0,14) >> data.colorCentersL.at<double>(1,14) >> data.colorCentersL.at<double>(2,14)
       >> data.colorCentersL.at<double>(0,15) >> data.colorCentersL.at<double>(1,15) >> data.colorCentersL.at<double>(2,15)
       >> data.colorCentersL.at<double>(0,16) >> data.colorCentersL.at<double>(1,16) >> data.colorCentersL.at<double>(2,16)
       >> data.colorCentersL.at<double>(0,17) >> data.colorCentersL.at<double>(1,17) >> data.colorCentersL.at<double>(2,17)
       >> data.colorCentersL.at<double>(0,18) >> data.colorCentersL.at<double>(1,18) >> data.colorCentersL.at<double>(2,18)
       >> data.colorCentersL.at<double>(0,19) >> data.colorCentersL.at<double>(1,19) >> data.colorCentersL.at<double>(2,19)
       >> data.colorCentersL.at<double>(0,20) >> data.colorCentersL.at<double>(1,20) >> data.colorCentersL.at<double>(2,20)
       >> data.colorCentersL.at<double>(0,21) >> data.colorCentersL.at<double>(1,21) >> data.colorCentersL.at<double>(2,21)
       >> data.colorCentersL.at<double>(0,22) >> data.colorCentersL.at<double>(1,22) >> data.colorCentersL.at<double>(2,22)
       >> data.colorCentersL.at<double>(0,23) >> data.colorCentersL.at<double>(1,23) >> data.colorCentersL.at<double>(2,23);

    in >> data.colorCentersR.at<double>(0,0) >> data.colorCentersR.at<double>(1,0) >> data.colorCentersR.at<double>(2,0)
       >> data.colorCentersR.at<double>(0,1) >> data.colorCentersR.at<double>(1,1) >> data.colorCentersR.at<double>(2,1)
       >> data.colorCentersR.at<double>(0,2) >> data.colorCentersR.at<double>(1,2) >> data.colorCentersR.at<double>(2,2)
       >> data.colorCentersR.at<double>(0,3) >> data.colorCentersR.at<double>(1,3) >> data.colorCentersR.at<double>(2,3)
       >> data.colorCentersR.at<double>(0,4) >> data.colorCentersR.at<double>(1,4) >> data.colorCentersR.at<double>(2,4)
       >> data.colorCentersR.at<double>(0,5) >> data.colorCentersR.at<double>(1,5) >> data.colorCentersR.at<double>(2,5)
       >> data.colorCentersR.at<double>(0,6) >> data.colorCentersR.at<double>(1,6) >> data.colorCentersR.at<double>(2,6)
       >> data.colorCentersR.at<double>(0,7) >> data.colorCentersR.at<double>(1,7) >> data.colorCentersR.at<double>(2,7)
       >> data.colorCentersR.at<double>(0,8) >> data.colorCentersR.at<double>(1,8) >> data.colorCentersR.at<double>(2,8)
       >> data.colorCentersR.at<double>(0,9) >> data.colorCentersR.at<double>(1,9) >> data.colorCentersR.at<double>(2,9)
       >> data.colorCentersR.at<double>(0,10) >> data.colorCentersR.at<double>(1,10) >> data.colorCentersR.at<double>(2,10)
       >> data.colorCentersR.at<double>(0,11) >> data.colorCentersR.at<double>(1,11) >> data.colorCentersR.at<double>(2,11)
       >> data.colorCentersR.at<double>(0,12) >> data.colorCentersR.at<double>(1,12) >> data.colorCentersR.at<double>(2,12)
       >> data.colorCentersR.at<double>(0,13) >> data.colorCentersR.at<double>(1,13) >> data.colorCentersR.at<double>(2,13)
       >> data.colorCentersR.at<double>(0,14) >> data.colorCentersR.at<double>(1,14) >> data.colorCentersR.at<double>(2,14)
       >> data.colorCentersR.at<double>(0,15) >> data.colorCentersR.at<double>(1,15) >> data.colorCentersR.at<double>(2,15)
       >> data.colorCentersR.at<double>(0,16) >> data.colorCentersR.at<double>(1,16) >> data.colorCentersR.at<double>(2,16)
       >> data.colorCentersR.at<double>(0,17) >> data.colorCentersR.at<double>(1,17) >> data.colorCentersR.at<double>(2,17)
       >> data.colorCentersR.at<double>(0,18) >> data.colorCentersR.at<double>(1,18) >> data.colorCentersR.at<double>(2,18)
       >> data.colorCentersR.at<double>(0,19) >> data.colorCentersR.at<double>(1,19) >> data.colorCentersR.at<double>(2,19)
       >> data.colorCentersR.at<double>(0,20) >> data.colorCentersR.at<double>(1,20) >> data.colorCentersR.at<double>(2,20)
       >> data.colorCentersR.at<double>(0,21) >> data.colorCentersR.at<double>(1,21) >> data.colorCentersR.at<double>(2,21)
       >> data.colorCentersR.at<double>(0,22) >> data.colorCentersR.at<double>(1,22) >> data.colorCentersR.at<double>(2,22)
       >> data.colorCentersR.at<double>(0,23) >> data.colorCentersR.at<double>(1,23) >> data.colorCentersR.at<double>(2,23);

    return in;
}

QDataStream& operator<<(QDataStream& out, const SMColorCalibrationParameters& data){
//    std::cout << "Serializing color calibration parameters!" << std::endl;
    out << data.calibration_matrixL.at<double>(0,0) << data.calibration_matrixL.at<double>(0,1) << data.calibration_matrixL.at<double>(0,2) << data.calibration_matrixL.at<double>(1,0) << data.calibration_matrixL.at<double>(1,1)
       << data.calibration_matrixL.at<double>(1,2) << data.calibration_matrixL.at<double>(2,0) << data.calibration_matrixL.at<double>(2,1) << data.calibration_matrixL.at<double>(2,2);

    out << data.calibration_matrixR.at<double>(0,0) << data.calibration_matrixR.at<double>(0,1) << data.calibration_matrixR.at<double>(0,2) << data.calibration_matrixR.at<double>(1,0) << data.calibration_matrixR.at<double>(1,1)
       << data.calibration_matrixR.at<double>(1,2) << data.calibration_matrixR.at<double>(2,0) << data.calibration_matrixR.at<double>(2,1) << data.calibration_matrixR.at<double>(2,2);

    out << data.colorCentersL.at<double>(0,0) << data.colorCentersL.at<double>(1,0) << data.colorCentersL.at<double>(2,0)
       << data.colorCentersL.at<double>(0,1) << data.colorCentersL.at<double>(1,1) << data.colorCentersL.at<double>(2,1)
       << data.colorCentersL.at<double>(0,2) << data.colorCentersL.at<double>(1,2) << data.colorCentersL.at<double>(2,2)
       << data.colorCentersL.at<double>(0,3) << data.colorCentersL.at<double>(1,3) << data.colorCentersL.at<double>(2,3)
       << data.colorCentersL.at<double>(0,4) << data.colorCentersL.at<double>(1,4) << data.colorCentersL.at<double>(2,4)
       << data.colorCentersL.at<double>(0,5) << data.colorCentersL.at<double>(1,5) << data.colorCentersL.at<double>(2,5)
       << data.colorCentersL.at<double>(0,6) << data.colorCentersL.at<double>(1,6) << data.colorCentersL.at<double>(2,6)
       << data.colorCentersL.at<double>(0,7) << data.colorCentersL.at<double>(1,7) << data.colorCentersL.at<double>(2,7)
       << data.colorCentersL.at<double>(0,8) << data.colorCentersL.at<double>(1,8) << data.colorCentersL.at<double>(2,8)
       << data.colorCentersL.at<double>(0,9) << data.colorCentersL.at<double>(1,9) << data.colorCentersL.at<double>(2,9)
       << data.colorCentersL.at<double>(0,10) << data.colorCentersL.at<double>(1,10) << data.colorCentersL.at<double>(2,10)
       << data.colorCentersL.at<double>(0,11) << data.colorCentersL.at<double>(1,11) << data.colorCentersL.at<double>(2,11)
       << data.colorCentersL.at<double>(0,12) << data.colorCentersL.at<double>(1,12) << data.colorCentersL.at<double>(2,12)
       << data.colorCentersL.at<double>(0,13) << data.colorCentersL.at<double>(1,13) << data.colorCentersL.at<double>(2,13)
       << data.colorCentersL.at<double>(0,14) << data.colorCentersL.at<double>(1,14) << data.colorCentersL.at<double>(2,14)
       << data.colorCentersL.at<double>(0,15) << data.colorCentersL.at<double>(1,15) << data.colorCentersL.at<double>(2,15)
       << data.colorCentersL.at<double>(0,16) << data.colorCentersL.at<double>(1,16) << data.colorCentersL.at<double>(2,16)
       << data.colorCentersL.at<double>(0,17) << data.colorCentersL.at<double>(1,17) << data.colorCentersL.at<double>(2,17)
       << data.colorCentersL.at<double>(0,18) << data.colorCentersL.at<double>(1,18) << data.colorCentersL.at<double>(2,18)
       << data.colorCentersL.at<double>(0,19) << data.colorCentersL.at<double>(1,19) << data.colorCentersL.at<double>(2,19)
       << data.colorCentersL.at<double>(0,20) << data.colorCentersL.at<double>(1,20) << data.colorCentersL.at<double>(2,20)
       << data.colorCentersL.at<double>(0,21) << data.colorCentersL.at<double>(1,21) << data.colorCentersL.at<double>(2,21)
       << data.colorCentersL.at<double>(0,22) << data.colorCentersL.at<double>(1,22) << data.colorCentersL.at<double>(2,22)
       << data.colorCentersL.at<double>(0,23) << data.colorCentersL.at<double>(1,23) << data.colorCentersL.at<double>(2,23);

    out << data.colorCentersR.at<double>(0,0) << data.colorCentersR.at<double>(1,0) << data.colorCentersR.at<double>(2,0)
       << data.colorCentersR.at<double>(0,1) << data.colorCentersR.at<double>(1,1) << data.colorCentersR.at<double>(2,1)
       << data.colorCentersR.at<double>(0,2) << data.colorCentersR.at<double>(1,2) << data.colorCentersR.at<double>(2,2)
       << data.colorCentersR.at<double>(0,3) << data.colorCentersR.at<double>(1,3) << data.colorCentersR.at<double>(2,3)
       << data.colorCentersR.at<double>(0,4) << data.colorCentersR.at<double>(1,4) << data.colorCentersR.at<double>(2,4)
       << data.colorCentersR.at<double>(0,5) << data.colorCentersR.at<double>(1,5) << data.colorCentersR.at<double>(2,5)
       << data.colorCentersR.at<double>(0,6) << data.colorCentersR.at<double>(1,6) << data.colorCentersR.at<double>(2,6)
       << data.colorCentersR.at<double>(0,7) << data.colorCentersR.at<double>(1,7) << data.colorCentersR.at<double>(2,7)
       << data.colorCentersR.at<double>(0,8) << data.colorCentersR.at<double>(1,8) << data.colorCentersR.at<double>(2,8)
       << data.colorCentersR.at<double>(0,9) << data.colorCentersR.at<double>(1,9) << data.colorCentersR.at<double>(2,9)
       << data.colorCentersR.at<double>(0,10) << data.colorCentersR.at<double>(1,10) << data.colorCentersR.at<double>(2,10)
       << data.colorCentersR.at<double>(0,11) << data.colorCentersR.at<double>(1,11) << data.colorCentersR.at<double>(2,11)
       << data.colorCentersR.at<double>(0,12) << data.colorCentersR.at<double>(1,12) << data.colorCentersR.at<double>(2,12)
       << data.colorCentersR.at<double>(0,13) << data.colorCentersR.at<double>(1,13) << data.colorCentersR.at<double>(2,13)
       << data.colorCentersR.at<double>(0,14) << data.colorCentersR.at<double>(1,14) << data.colorCentersR.at<double>(2,14)
       << data.colorCentersR.at<double>(0,15) << data.colorCentersR.at<double>(1,15) << data.colorCentersR.at<double>(2,15)
       << data.colorCentersR.at<double>(0,16) << data.colorCentersR.at<double>(1,16) << data.colorCentersR.at<double>(2,16)
       << data.colorCentersR.at<double>(0,17) << data.colorCentersR.at<double>(1,17) << data.colorCentersR.at<double>(2,17)
       << data.colorCentersR.at<double>(0,18) << data.colorCentersR.at<double>(1,18) << data.colorCentersR.at<double>(2,18)
       << data.colorCentersR.at<double>(0,19) << data.colorCentersR.at<double>(1,19) << data.colorCentersR.at<double>(2,19)
       << data.colorCentersR.at<double>(0,20) << data.colorCentersR.at<double>(1,20) << data.colorCentersR.at<double>(2,20)
       << data.colorCentersR.at<double>(0,21) << data.colorCentersR.at<double>(1,21) << data.colorCentersR.at<double>(2,21)
       << data.colorCentersR.at<double>(0,22) << data.colorCentersR.at<double>(1,22) << data.colorCentersR.at<double>(2,22)
       << data.colorCentersR.at<double>(0,23) << data.colorCentersR.at<double>(1,23) << data.colorCentersR.at<double>(2,23);

    return out;

}
