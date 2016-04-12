#-------------------------------------------------
#
# Project created by QtCreator 2016-02-15T22:21:29
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

greaterThan(QT_MAJOR_VERSION, 4){
CONFIG += c++11
} else {
QMAKE_CXXFLAGS += -std=c++0x
}

TARGET = manual_registration_win
TEMPLATE = app


SOURCES += \
    src/manual_registration.cpp \
    build/release/moc_manual_registration.cpp

HEADERS  += \
    src/manual_registration.h \
    build/ui_manual_registration.h

FORMS    += \
    src/manual_registration.ui


# Windows
win32 {
    # Boost join
    DEFINES += DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED

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
    INCLUDEPATH += "$$(BOOST_INCLUDE_DIR)" \
                   "$$(EIGEN_ROOT)/include" \
                   "$$(FLANN_ROOT)/include"

    # vtk
    INCLUDEPATH += "$$(VTK_INCLUDE_DIR)"

    CONFIG(debug,debug|release){
    #debug
    LIBS += -L"$$(BOOST_ROOT)/lib" -llibboost_system-vc120-mt-gd-1_56

    LIBS += -L"$$(VTK_DIR)" \ #C:\Program Files\VTK\lib\vtk-5.10
            -lvtkCommonCore-6.1 \
            -lvtkCommonMath-6.1 \
            -lvtkCommonDataModel-6.1 \
            -lvtkCommonExecutionModel-6.1 \
            -lvtkFiltersCore-6.1 \
            -lvtkRenderingCore-6.1 \
            -lvtkRenderingLOD-6.1 \
            -lvtkGUISupportQt-6.1\
            -lvtkGUISupportQtOpenGL-6.1 \
            -lvtkInteractionStyle-6.1 \
            -lvtkFiltersSources-6.1
    } else {
   # release
    LIBS += -L"$$(BOOST_ROOT)/lib" -llibboost_system-vc120-mt-1_56

    LIBS += -L"$$(VTK_DIR)" \
            -lvtkCommonCore-6.1 \
            -lvtkCommonMath-6.1 \
            -lvtkCommonDataModel-6.1 \
            -lvtkCommonExecutionModel-6.1 \
            -lvtkFiltersCore-6.1 \
            -lvtkRenderingCore-6.1 \
            -lvtkRenderingLOD-6.1 \
            -lvtkGUISupportQt-6.1\
            -lvtkGUISupportQtOpenGL-6.1 \
            -lvtkInteractionStyle-6.1 \
            -lvtkFiltersSources-6.1








    }
}

OTHER_FILES += \
    config/config.xml \
    packages/package_manual_registration/meta/package.xml \
    packages/package_manual_registration/meta/installscript.qs

