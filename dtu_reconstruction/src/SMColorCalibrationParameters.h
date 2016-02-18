#ifndef SMCOLORCALIBRATIONPARAMETERS_H
#define SMCOLORCALIBRATIONPARAMETERS_H

#include <QObject>
#include <QDataStream>
#include <QMetaType>
#include <opencv2/opencv.hpp>

class SMColorCalibrationParameters
{


    public:
        SMColorCalibrationParameters(): colorCentersL(3, 24, CV_64F), colorCentersR(3, 24, CV_64F){

        }

        void print();
        void exportToXML(QString fileName);
        void importFromXML(QString fileName);

        //Output
        cv::Mat calibration_matrixL;
        cv::Mat calibration_matrixR;
        cv::Mat colorCentersL;
        cv::Mat colorCentersR;


};



QDataStream& operator>>(QDataStream& in, SMColorCalibrationParameters& data);
QDataStream& operator<<(QDataStream& out, const SMColorCalibrationParameters& data);

Q_DECLARE_METATYPE(SMColorCalibrationParameters)

#endif // SMCOLORCALIBRATIONPARAMETERS_H
