#-------------------------------------------------
#
# Project created by QtCreator 2015-05-27T13:57:44
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = DTIColorCalibration
CONFIG   += console
CONFIG   -= app_bundle

# With C++11 support
greaterThan(QT_MAJOR_VERSION, 4){
CONFIG += c++11
} else {
QMAKE_CXXFLAGS += -std=c++0x
}

#win32 {
    CONFIG(debug, debug|release) {
        DESTDIR = build/debug
    } else {
        DESTDIR = build/release
    }

    OBJECTS_DIR = $$DESTDIR/obj
    MOC_DIR = $$DESTDIR/moc
    RCC_DIR = $$DESTDIR/qrc
    UI_DIR = $$DESTDIR/ui
#}

TEMPLATE = app

SOURCES += src/main.cpp \
    src/MacBethColorCalibrate.cpp \
    src/MacBethDetector.cpp

HEADERS += \
    src/MacBethColorCalibrate.h \
    src/MacBethDetector.h

# Operating System dependant linking and including
# Linux
unix:!macx {
    CONFIG += link_pkgconfig
    # opencv
    PKGCONFIG += opencv
}

# Windows
win32 {
    # opencv
    INCLUDEPATH += "$$(OPENCV_INCLUDE_DIR)/" #C:\opencv\build\include

    CONFIG(debug,debug|release){
    #debug
   LIBS += -L"$$(OPENCV_DIR)" \
            -lopencv_core2411d \
            -lopencv_highgui2411d \
            -lopencv_imgproc2411d \
            -lopencv_calib3d2411d \
            -lopencv_contrib2411d \
            -lopencv_features2d2411d \
            -lopencv_flann2411d
   } else {
    #release
    LIBS += -L"$$(OPENCV_DIR)" \
            -lopencv_core2411 \
            -lopencv_highgui2411 \
            -lopencv_imgproc2411 \
            -lopencv_calib3d2411 \
            -lopencv_contrib2411 \
            -lopencv_features2d2411 \
            -lopencv_flann2411
    }
}


