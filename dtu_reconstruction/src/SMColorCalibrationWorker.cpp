#include "SMColorCalibrationWorker.h"
#include "MacBethColorCalibrate.h"
#include "SMColorCalibrationParameters.h"
#include <vector>
#include <QSettings>

void SMColorCalibrationWorker::performCalibration(std::vector< SMCalibrationSet > calibrationData)
{
    QSettings settings;
    MacBethColorCalibrate mac;
    SMColorCalibrationParameters params;
    std::vector<SMColorCalibrationParameters> calib;
    for(std::vector<SMCalibrationSet>::iterator it = calibrationData.begin(); it != calibrationData.end(); ++it) {
        SMCalibrationSet cs = *it;
        mac.calibrate(cs.colorCalibration0);
        mac.calibrate(cs.colorCalibration1);
        params.calibration_matrixL = cs.colorCalibration0.calibration_matrix;
        params.calibration_matrixR = cs.colorCalibration1.calibration_matrix;
        params.colorCentersL = cs.colorCalibration0.colorCenters;
        params.colorCentersR = cs.colorCalibration1.colorCenters;
        calib.push_back(params);
    }


    //Compute global calibration matrix and store in settings

    // Print to std::cout
    params.print();
    // save to (reentrant qsettings object)
    settings.setValue("colorCalibration/parameters", QVariant::fromValue(params));


    emit done();
}
