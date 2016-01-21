#-------------------------------------------------
#
# Project created by QtCreator 2015-11-06T13:53:29
#
#-------------------------------------------------

QT       += core

QT       -= gui

greaterThan(QT_MAJOR_VERSION, 4){
CONFIG += c++11
} else {
QMAKE_CXXFLAGS += -std=c++0x
}

TARGET = dtu_reconstruction
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES +=  src/viewdata.cpp \
            src/stereo_match.cpp \
            src/SMColorCalibrationWorker.cpp \
            src/SMCalibrationWorker.cpp \
            src/SMCalibrationParameters.cpp \
            src/scenedata.cpp \
            src/main.cpp \
            src/cvtools.cpp \
            src/AlgorithmGrayCode.cpp \
            #src/MacBethDetector.cpp \
            src/MacBethColorCalibrate.cpp \
    src/SMColorCalibrationParameters.cpp

HEADERS += src/viewdata.h \
            src/stereo_match.h \
            src/SMTypes.h \
            src/SMColorCalibrationWorker.h \
            src/SMCalibrationWorker.h \
            src/SMCalibrationParameters.h \
            src/scenedata.h \
            src/cvtools.h \
            src/AlgorithmGrayCode.h \
            src/Algorithm.h \
            #src/MacBethDetector.h \
            src/MacBethColorCalibrate.h \
    src/SMColorCalibrationParameters.h


# Operating System dependant linking and including
# Linux
unix:!macx {

    CONFIG += link_pkgconfig
    # Link VTK and Boost (no pkg-config)
    INCLUDEPATH += /usr/include/vtk-5.8/
    LIBS += -lQVTK -lvtkCommon -lvtkFiltering -lvtkRendering -lvtkIO -lvtkGraphics -lvtkHybrid
    # PCL pkg-config workaround
    LIBS += -lboost_system -lpcl_visualization -lpcl_common -lpcl_io -lpcl_search -lpcl_surface
    # PKG-config libs
    INCLUDEPATH += /usr/local/include/pcl-1.8 /usr/include/eigen3/
    PKGCONFIG += gl glu x11 /opt/ros/hydro/lib/pkgconfig/opencv.pc eigen3
 #   PKGCONFIG += pcl_registration-1.8 pcl_visualization-1.8 pcl_surface-1.8 pcl_search-1.8 pcl_filters-1.8 pcl_kdtree-1.8 pcl_tracking-1.8
  #  QMAKE_CXXFLAGS += -fopenmp
  #  QMAKE_LFLAGS *= -fopenmp

}
