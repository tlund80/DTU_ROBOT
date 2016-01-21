#ifndef SMCALIBRATIONPARAMETERS_H
#define SMCALIBRATIONPARAMETERS_H

#include <QObject>
#include <QDataStream>
#include <QMetaType>
#include <opencv2/opencv.hpp>

class SMCalibrationParameters
{
    public:

        SMCalibrationParameters();

        ~SMCalibrationParameters();

        unsigned int frameHeight;
        unsigned int frameWidth;

        cv::Matx33f K0; // intrinsic camera matrix
        cv::Vec<float, 5> k0; // distortion coefficients
        double cam0_error; // overall reprojection error

        cv::Matx33f K1;
        cv::Vec<float, 5> k1;
        double cam1_error;

        cv::Matx33f R1; // extrinsic rotation matrix camera 1
        cv::Vec3f   T1; // extrinsic translation vector camera 1
        double stereo_error; // stereo calibration reprojection error

    //    cv::Matx33f Rr; // extrinsic rotation rotation stage
    //    cv::Vec3f   Tr; // extrinsic translation vector rotation stage

        cv::Matx33f E; // essential matrix
        cv::Matx33f F; // fundamental matrix

        void print();
        void exportToXML(QString fileName);
        void importFromXML(QString fileName);
};

QDataStream& operator>>(QDataStream& in, SMCalibrationParameters& data);
QDataStream& operator<<(QDataStream& out, const SMCalibrationParameters& data);

Q_DECLARE_METATYPE(SMCalibrationParameters)


#endif // SMCALIBRATIONPARAMETERS_H
