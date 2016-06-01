#-------------------------------------------------
#
# Project created by QtCreator 2016-05-30T18:19:40
#
#-------------------------------------------------

QT       += core

QT       -= gui

greaterThan(QT_MAJOR_VERSION, 4){
CONFIG += c++11
} else {
QMAKE_CXXFLAGS += -std=c++0x
}

TARGET = generate_gt
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

# Operating System dependant linking and including
# Linux
unix:!macx {

    CONFIG += link_pkgconfig

 # Link VTK and Boost (no pkg-config)
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

    # PCL pkg-config workaround
    INCLUDEPATH += /usr/local/include/pcl-1.8/ /usr/include/eigen3/
    LIBS += -lboost_system -lpcl_visualization -lpcl_common -lpcl_io -lpcl_search -lpcl_surface -lpcl_filters -lpcl_features
    # PKG-config libs

    PKGCONFIG += gl glu x11 /usr/local/lib/pkgconfig/opencv.pc eigen3
 #   PKGCONFIG += pcl_registration-1.8 pcl_visualization-1.8 pcl_surface-1.8 pcl_search-1.8 pcl_filters-1.8 pcl_kdtree-1.8 pcl_tracking-1.8
  #  QMAKE_CXXFLAGS += -fopenmp
  #  QMAKE_LFLAGS *= -fopenmp

}
