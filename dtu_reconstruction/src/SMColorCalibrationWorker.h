#ifndef SMCOLORCALIBRATIONWORKER_H
#define SMCOLORCALIBRATIONWORKER_H

#include <QObject>
#include "SMTypes.h"

namespace Ui {
    class SMColorCalibrationWorker;
}

class SMColorCalibrationWorker : public QObject{

      Q_OBJECT
public:
    SMColorCalibrationWorker(){}
    ~SMColorCalibrationWorker(){}


public slots:
    void performCalibration(std::vector< SMCalibrationSet > calibrationData);

signals:
    void newFrameResult(int idx, int camID, bool success, cv::Mat frameResult);
    void newSetProcessed(int idx);
    void done();
};

#endif // SMCOLORCALIBRATIONWORKER_H
