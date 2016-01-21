#include "SMCalibrationParameters.h"

SMCalibrationParameters::SMCalibrationParameters () : frameHeight(0), frameWidth(0),
                          K0(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), k0(0.0), cam0_error(0.0),
                          K1(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), k1(0.0), cam1_error(0.0),
                          R1(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), T1(0.0), stereo_error(0.0),
                          //Rr(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), Tr(0.0),
                          E(0.0), F(0.0){

    qRegisterMetaType<SMCalibrationParameters>("SMCalibrationParameters");
    qRegisterMetaTypeStreamOperators<SMCalibrationParameters>("SMCalibrationParameters");
}

SMCalibrationParameters::~SMCalibrationParameters(){

}

void SMCalibrationParameters::print(){

    std::cout << "Calibration Parameters:" << std::endl;
    std::cout << "K0: " << std::endl;
    std::cout << K0 << std::endl;
    std::cout << "k0: " << std::endl;
    std::cout << k0 << std::endl;
    std::cout << "K1: " << std::endl;
    std::cout << K1 << std::endl;
    std::cout << "k1: " << std::endl;
    std::cout << k1 << std::endl;
    std::cout << "R1: " << std::endl;
    std::cout << R1 << std::endl;
    std::cout << "T1: " << std::endl;
    std::cout << T1 << std::endl;
    //std::cout << "Rr: " << std::endl;
    //std::cout << Rr << std::endl;
    //std::cout << "Tr: " << std::endl;
    //std::cout << Tr << std::endl;
    std::cout << "cam0_error: " << std::endl;
    std::cout << cam0_error << std::endl;
    std::cout << "cam1_error: " << std::endl;
    std::cout << cam1_error << std::endl;
    std::cout << "stereo_error: " << std::endl;
    std::cout << stereo_error << std::endl;
}

void SMCalibrationParameters::exportToXML(QString fileName){

    cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::WRITE);
    if (!fs.isOpened())
        std::cerr << "SMCalibrationParameters: could not save file!" << std::endl;

    fs << "K0" << cv::Mat(K0) << "k0" << cv::Mat(k0) << "cam0_error" << cam0_error
       << "K1" << cv::Mat(K1) << "k1" << cv::Mat(k1) << "cam1_error" << cam1_error
       << "R1" << cv::Mat(R1) << "T1" << cv::Mat(T1) << "stereo_error" << stereo_error
       //<< "Rr" << cv::Mat(Rr) << "Tr" << cv::Mat(Tr)
       << "E" << cv::Mat(E) << "F" << cv::Mat(F);

    fs.release();

}

void SMCalibrationParameters::importFromXML(QString fileName){

    cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened())
        std::cerr << "SMCalibrationParameters: could not open file!" << std::endl;

    cv::Mat temp;
    fs["K0"] >> temp; K0 = temp;
    fs["k0"] >> temp; k0 = temp;
    fs["cam0_error"] >> cam0_error;
    fs["K1"] >> temp; K1 = temp;
    fs["k1"] >> temp; k1 = temp;
    fs["cam1_error"] >> cam1_error;
    fs["R1"] >> temp; R1 = temp;
    fs["T1"] >> temp; T1 = temp;
    fs["stereo_error"] >> stereo_error;
    //fs["Rr"] >> temp; Rr = temp;
    //fs["Tr"] >> temp; Tr = temp;
    fs["E"] >> temp; E = temp;
    fs["F"] >> temp; F = temp;

    fs.release();
}

// QStreamtypes for qDebug() and QSettings
QDataStream& operator>>(QDataStream& in, SMCalibrationParameters& data){
//    std::cout << "Deserializing calibration parameters!" << std::endl;
    in >> data.K0(0,0) >> data.K0(0,1) >> data.K0(0,2) >> data.K0(1,0) >> data.K0(1,1) >> data.K0(1,2) >> data.K0(2,0) >> data.K0(2,1) >> data.K0(2,2);
    in >> data.k0(0) >> data.k0(1) >> data.k0(2) >> data.k0(3) >> data.k0(4);
    in >> data.cam0_error;

    in >> data.K1(0,0) >> data.K1(0,1) >> data.K1(0,2) >> data.K1(1,0) >> data.K1(1,1) >> data.K1(1,2) >> data.K1(2,0) >> data.K1(2,1) >> data.K1(2,2);
    in >> data.k1(0) >> data.k1(1) >> data.k1(2) >> data.k1(3) >> data.k1(4);
    in >> data.cam1_error;

    in >> data.R1(0,0) >> data.R1(0,1) >> data.R1(0,2) >> data.R1(1,0) >> data.R1(1,1) >> data.R1(1,2) >> data.R1(2,0) >> data.R1(2,1) >> data.R1(2,2);
    in >> data.T1(0) >> data.T1(1) >> data.T1(2);
    in >> data.stereo_error;

    in >> data.E(0,0) >> data.E(0,1) >> data.E(0,2) >> data.E(1,0) >> data.E(1,1) >> data.E(1,2) >> data.E(2,0) >> data.E(2,1) >> data.E(2,2);
    in >> data.F(0,0) >> data.F(0,1) >> data.F(0,2) >> data.F(1,0) >> data.F(1,1) >> data.F(1,2) >> data.F(2,0) >> data.F(2,1) >> data.F(2,2);

   // in >> data.Rr(0,0) >> data.Rr(0,1) >> data.Rr(0,2) >> data.Rr(1,0) >> data.Rr(1,1) >> data.Rr(1,2) >> data.Rr(2,0) >> data.Rr(2,1) >> data.Rr(2,2);
   // in >> data.Tr(0) >> data.Tr(1) >> data.Tr(2);

    return in;
}

QDataStream& operator<<(QDataStream& out, const SMCalibrationParameters& data){
//    std::cout << "Serializing calibration parameters!" << std::endl;
    out << data.K0(0,0) << data.K0(0,1) << data.K0(0,2) << data.K0(1,0) << data.K0(1,1) << data.K0(1,2) << data.K0(2,0) << data.K0(2,1) << data.K0(2,2);
    out << data.k0(0) << data.k0(1) << data.k0(2) << data.k0(3) << data.k0(4);
    out << data.cam0_error;

    out << data.K1(0,0) << data.K1(0,1) << data.K1(0,2) << data.K1(1,0) << data.K1(1,1) << data.K1(1,2) << data.K1(2,0) << data.K1(2,1) << data.K1(2,2);
    out << data.k1(0) << data.k1(1) << data.k1(2) << data.k1(3) << data.k1(4);
    out << data.cam1_error;

    out << data.R1(0,0) << data.R1(0,1) << data.R1(0,2) << data.R1(1,0) << data.R1(1,1) << data.R1(1,2) << data.R1(2,0) << data.R1(2,1) << data.R1(2,2);
    out << data.T1(0) << data.T1(1) << data.T1(2);
    out << data.stereo_error;

    out << data.E(0,0) << data.E(0,1) << data.E(0,2) << data.E(1,0) << data.E(1,1) << data.E(1,2) << data.E(2,0) << data.E(2,1) << data.E(2,2);
    out << data.F(0,0) << data.F(0,1) << data.F(0,2) << data.F(1,0) << data.F(1,1) << data.F(1,2) << data.F(2,0) << data.F(2,1) << data.F(2,2);

 //   out << data.Rr(0,0) << data.Rr(0,1) << data.Rr(0,2) << data.Rr(1,0) << data.Rr(1,1) << data.Rr(1,2) << data.Rr(2,0) << data.Rr(2,1) << data.Rr(2,2);
 //   out << data.Tr(0) << data.Tr(1) << data.Tr(2);

    return out;

}



