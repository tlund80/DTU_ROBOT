#-------------------------------------------------
#
# Project created by QtCreator 2015-11-17T11:47:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

greaterThan(QT_MAJOR_VERSION, 4){
CONFIG += c++11
} else {
QMAKE_CXXFLAGS += -std=c++0x
}

TARGET = cal_ti
TEMPLATE = app

#DEFINES += USE_ROS

SOURCES += \
    src/main.cpp \
    src/marker/markerdetector.cpp \
    src/marker/marker.cpp \
    src/marker/cvdrawingutils.cpp \
    src/marker/cameraparameters.cpp \
    src/marker/boarddetector.cpp \
    src/marker/board.cpp \
    src/marker/arucofidmarkers.cpp \
    src/gui/main_window.cpp \
    src/gui/ImgAnnotation.cpp \
    src/calibration/tiv_types.cpp \
    src/calibration/tiv_test.cpp \
    src/calibration/StereoCalibration.cpp \
    src/calibration/rot.cpp \
    src/calibration/MonoCalibration.cpp \
    src/calibration/HandEyeCalibration.cpp \
    src/calibration/dornaika.cpp \
    src/calibration/CalibThread.cpp \
    src/calibration/Calibration.cpp

HEADERS  += \
    src/marker/markerdetector.h \
    src/marker/marker.h \
    src/marker/exports.h \
    src/marker/cvdrawingutils.h \
    src/marker/cameraparameters.h \
    src/marker/boarddetector.h \
    src/marker/board.h \
    src/marker/arucofidmarkers.h \
    src/marker/aruco.h \
    include/cal_ti/ui_AnnotateDialog.hpp \
    include/cal_ti/tiv_types.hpp \
    include/cal_ti/StereoCalibration.hpp \
    include/cal_ti/SharedData.hpp \
    include/cal_ti/rot.hpp \
    include/cal_ti/MonoCalibration.hpp \
    include/cal_ti/main_window.hpp \
    include/cal_ti/ImgAnnotation.hpp \
    include/cal_ti/HandEyeCalibration.hpp \
    include/cal_ti/dornaika.hpp \
    include/cal_ti/config.hpp \
    include/cal_ti/CalibThread.hpp \
    include/cal_ti/Calibration.hpp

FORMS    += ui/main_window.ui

use_ros{
    SOURCES += src/qnode.cpp \
    HEADERS +=  include/cal_ti/qnode.hpp \
}

SUBDIRS += ext src include ui

# Operating System dependant linking and including
# Linux
unix:!macx {

    #Link VTK and Boost (no pkg-config)
    INCLUDEPATH += /usr/local/include/vtk-6.3/
    LIBS += \
            -lvtkGUISupportQt-6.3\
            -lvtkCommonCore-6.3 \
            -lvtkFiltersCore-6.3 \
            -lvtkRenderingCore-6.3 \
            -lvtkRenderingLOD-6.3 \
            -lvtkCommonDataModel-6.3 \
            -lvtkCommonMath-6.3 \
            -lvtkIOCore-6.3 \
            -lvtkpng-6.3 \
            -lvtksys-6.3 \
            -lvtktiff-6.3 \
            -lvtkjpeg-6.3 \
            -lvtkexpat-6.3 \
            -lvtkzlib-6.3

    INCLUDEPATH += $$PWD/ext/levmar-2.6
    DEPENDPATH += $$PWD/ext/levmar-2.6/build
    LIBS += -L$$PWD/ext/levmar-2.6/build/ -llevmar

    #Blas
    INCLUDEPATH += /usr/include
    LIBS += -L/usr/lib -lblas

    #Lapack
    LIBS += -L/usr/lib -llapack

    #Boost pkg-config workaround
    LIBS += -lboost_system

    INCLUDEPATH += /usr/include/pcl-1.7/ #/usr/local/include/pcl-1.8
    LIBS += -lpcl_visualization \
            -lpcl_common \
            -lpcl_io \
            -lpcl_search \
            -lpcl_surface \
            -lpcl_filters \
            -lpcl_kdtree \
            -lpcl_features \
            -lpcl_keypoints \
            -lpcl_segmentation \
            -lpcl_sample_consensus \
            -lpcl_registration \
            -lpcl_recognition

    # PKG-config libs
    CONFIG += link_pkgconfig

    INCLUDEPATH += /usr/include/eigen3/
    PKGCONFIG += gl glu x11 /usr/lib/x86_64-linux-gnu/pkgconfig/opencv.pc eigen3 #/usr/local/lib/pkgconfig/opencv.pc

 #   PKGCONFIG += pcl_registration-1.8 pcl_visualization-1.8 pcl_surface-1.8 pcl_search-1.8 pcl_filters-1.8 pcl_kdtree-1.8 pcl_tracking-1.8
  #  QMAKE_CXXFLAGS += -fopenmp
  #  QMAKE_LFLAGS *= -fopenmp

}


# Windows
win32 {
    # Boost join
    DEFINES += DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED

    # opencv
    INCLUDEPATH += "$$(OPENCV_INCLUDE_DIR)/" #C:\opencv\build\include

    CONFIG(debug,debug|release){
    #debug
    LIBS += -L"$$(OPENCV_DIR)" \ #C:\opencv\build\x64\vc10\lib
            -lopencv_core2411d \
            -lopencv_highgui2411d \
            -lopencv_imgproc2411d \
            -lopencv_calib3d2411d \
            -lopencv_features2d2411d \
            -lopencv_flann2411
    } else {
   #release
    LIBS += -L"$$(OPENCV_DIR)" \
            -lopencv_core2411 \
            -lopencv_highgui2411 \
            -lopencv_imgproc2411 \
            -lopencv_calib3d2411 \
            -lopencv_features2d2411 \
            -lopencv_flann2411
    }


 #   INCLUDEPATH += $$PWD/ext/levmar-2.6
 #   DEPENDPATH += $$PWD/ext/levmar-2.6
    #LevMar
 #   CONFIG(release, debug|release){
 #    LIBS += -L$$PWD/ext/levmar-2.6/build/release/ -llevmar
 #   }else{
 #      LIBS += -L$$PWD/ext/levmar-2.6/build/debug/ -llevmar
 #   }

    # pcl
    INCLUDEPATH += "$$(PCL_INCLUDE_DIR)/" #C:\Program Files\PCL\include\pcl-1.7

    CONFIG(debug,debug|release){
    #debug
    LIBS += -L"$$(PCL_DIR)" \ #C:\Program Files\PCL\lib
            -lpcl_common_release \
            -lpcl_features_release \
            -lpcl_filters_release \
            -lpcl_io_ply_release \
            -lpcl_io_release \
            -lpcl_kdtree_release \
            -lpcl_keypoints_release \
            -lpcl_ml_release \
            -lpcl_octree_release \
            -lpcl_outofcore_release \
            -lpcl_people_release \
            -lpcl_recognition_release \
            -lpcl_registration_release \
            -lpcl_sample_consensus_release \
            -lpcl_search_release \
            -lpcl_segmentation_release \
            -lpcl_surface_release \
            -lpcl_tracking_release \
            -lpcl_visualization_release

    } else {
    # release
    LIBS += -L"$$(PCL_DIR)" \
            -lpcl_common_release \
            -lpcl_features_release \
            -lpcl_filters_release \
            -lpcl_io_ply_release \
            -lpcl_io_release \
            -lpcl_kdtree_release \
            -lpcl_keypoints_release \
            -lpcl_ml_release \
            -lpcl_octree_release \
            -lpcl_outofcore_release \
            -lpcl_people_release \
            -lpcl_recognition_release \
            -lpcl_registration_release \
            -lpcl_sample_consensus_release \
            -lpcl_search_release \
            -lpcl_segmentation_release \
            -lpcl_surface_release \
            -lpcl_tracking_release \
            -lpcl_visualization_release

    }

    # pcl dependencies
    INCLUDEPATH += "$$(BOOST_ROOT)/include" \
                   "$$(EIGEN_ROOT)/include" \
                   "$$(FLANN_ROOT)/include"

    # vtk
    INCLUDEPATH += "$$(VTK_INCLUDE_DIR)" #C:\Program Files\VTK\include\vtk-5.10

    CONFIG(debug,debug|release){
    #debug
    LIBS += -L"$$(BOOST_ROOT)/lib" -llibboost_system-vc120-mt-gd-1_59

    LIBS += -L"$$(VTK_DIR)" \ #C:\Program Files\VTK\lib\vtk-5.10
          #  -lvtkGraphics \
        #    -lQVTK \
            -lvtkGUISupportQt-6.1\
            -lvtkCommonCore-6.1 \
            -lvtkFiltersCore-6.1 \
            -lvtkRenderingCore-6.1 \
            -lvtkIOCore-6.1 \
            -lvtkpng-6.1 \
            -lvtksys-6.1 \
            -lvtktiff-6.1 \
            -lvtkjpeg-6.1 \
            -lvtkexpat-6.1 \
            -lvtkzlib-6.1
           # -lvtkGraphics-gd \
           # -lQVTK-gd \
      #      -lvtkCommon-gd \
      #      -lvtkFiltering-gd \
      #      -lvtkRendering-gd \
      #      -lvtkIO-gd \
      #      -lvtkpng-gd \
      #      -lvtksys-gd \
      #      -lvtktiff-gd \
      #      -lvtkjpeg-gd \
      #      -lvtkexpat-gd \
      #      -lvtkzlib-gd
    } else {
   # release
    LIBS += -L"$$(BOOST_ROOT)/lib" -llibboost_system-vc120-mt-1_59

    LIBS += -L"$$(VTK_DIR)" \
          #  -lvtkGraphics \
        #    -lQVTK \
            -lvtkGUISupportQt-6.1\
            -lvtkCommonCore-6.1 \
            -lvtkFiltersCore-6.1 \
            -lvtkRenderingCore-6.1 \
            -lvtkIOCore-6.1 \
            -lvtkpng-6.1 \
            -lvtksys-6.1 \
            -lvtktiff-6.1 \
            -lvtkjpeg-6.1 \
            -lvtkexpat-6.1 \
            -lvtkzlib-6.1
    }
}



#Mac OS X
macx {
    INCLUDEPATH += /opt/local/include/vtk-5.10/
  #  INCLUDEPATH += $$PWD/ext/levmar-2.6/build
    LIBS += -L/opt/local/lib/vtk-5.10/ -lQVTK -lvtkCommon -lvtkFiltering -lvtkRendering -lvtkIO -lvtkGraphics
    LIBS += -L/opt/local/lib/ -lboost_system-mt
  #  LIBS += -llevmar
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv pcl_visualization-1.7 pcl_filters-1.7 pcl_search-1.7 pcl_registration-1.7
    DEFINES += BOOST_TT_HAS_OPERATOR_HPP_INCLUDED

  #  DEPENDPATH += $$PWD/ext/levmar-2.6/build
}

RESOURCES += \
    resources/images.qrc

OTHER_FILES += \
    resources/images.qrc.depends \
    resources/images/stop-red.png \
    resources/images/save.png \
    resources/images/robot.jpg \
    resources/images/play1.png \
    resources/images/icon.png \
    resources/images/checker64.jpg
