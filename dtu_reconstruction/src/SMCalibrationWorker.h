#ifndef SMCalibrationWorker_H
#define SMCalibrationWorker_H

#include <QObject>

#include "SMTypes.h"

namespace Ui {
    class SMCalibrationWorker;
}

class SMCalibrationWorker : public QObject{
    Q_OBJECT

    public:
        SMCalibrationWorker(){}
        ~SMCalibrationWorker(){}
    public slots:
        void performCalibration(std::vector< SMCalibrationSet > calibrationData, std::string file_path = "Calibration.xml");

    private slots:
    signals:
        void newFrameResult(int idx, int camID, bool success, cv::Mat frameResult);
        void newSetProcessed(int idx);
        void done();

    private:


};

#endif // SMCalibrationWorker_H
