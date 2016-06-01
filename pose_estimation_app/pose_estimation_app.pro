#-------------------------------------------------
#
# Project created by QtCreator 2016-01-25T09:37:35
#
#-------------------------------------------------

QT       += core

QT       -= gui

greaterThan(QT_MAJOR_VERSION, 4){
CONFIG += c++11
} else {
QMAKE_CXXFLAGS += -std=c++0x
}

QMAKE_CXXFLAGS+= -fopenmp
QMAKE_LFLAGS +=  -fopenmp

TARGET = pose_estimation_app
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += src/main.cpp \
    src/halcon/SurfaceModelDetector.cpp \
    src/halcon/SurfaceModelCreator.cpp \
    src/halcon/Create3DObjectModel.cpp \
    src/detect/correspondence_filter_ratio.cpp \
    src/detect/correspondence_filter_distance.cpp \
    src/core/macros.cpp \
    src/core/detection.cpp \
    src/core/correspondence.cpp \
    src/core/range.cpp \
    src/core/stat.cpp \
    src/core/random.cpp \ #\
    src/detect/halconestimation.cpp \
 #   src/FeatureEstimator.cpp \
#    src/feature_matching_tools.cpp \
#    src/feature_matching_metrics.cpp \
#    src/feature_matching_features.cpp
    #src/FeatureEstimator.cpp


HEADERS += \
   src/halcon/SurfaceModelDetector.h \
   src/halcon/SurfaceModelCreator.h \
   src/halcon/ObjectModelAttribute.hpp \
   src/halcon/ObjectModel3DSegmentation.h \
   src/halcon/ModelDetectors.hpp \
   src/halcon/ModelDetectionParameters.hpp \
   src/halcon/ModelCreators.hpp \
   src/halcon/ModelCreationParameters.hpp \
   src/halcon/Create3DObjectModel.h \
   src/halcon/CADModel.h \
    src/detect/search_base_impl.hpp \
    src/detect/search_base.h \
    src/detect/ransac_impl.hpp \
    src/detect/ransac.h \
    src/detect/pose_sampler.h \
    src/detect/point_search_impl.hpp \
    src/detect/point_search.h \
    src/detect/fit_evaluation_impl.hpp \
    src/detect/fit_evaluation.h \
    src/detect/feature_search_impl.hpp \
    src/detect/feature_search.h \
    src/detect/detect_base.h \
    src/detect/correspondence_voting_impl.hpp \
    src/detect/correspondence_voting.h \
    src/detect/correspondence_filter_transform_impl.hpp \
    src/detect/correspondence_filter_transform.h \
    src/detect/correspondence_filter_ratio.h \
    src/detect/correspondence_filter_distance.h \
    src/detect/correspondence_filter_base.h \
    src/core/transform.h \
    src/core/traits.h \
    src/core/macros.h \
    src/core/ids.h \
    src/core/detection.h \
    src/core/covis_base.h \
    src/core/correspondence.h \
    src/core/progress_display.h \
    src/core/io_impl.hpp \
    src/core/io.h \
    src/core/range.h \
    src/detect/detect.h \
    src/core/core.h \
    src/core/dist.h \
   src/core/stat.h \
   src/core/stat_impl.hpp \
    src/core/converters_impl.hpp \
    src/core/converters.h \
    src/core/random_impl.hpp \
    src/core/random.h \
    src/covis_config.h \
    src/detect/halconestimation.h \
 #   src/FeatureEstimator.h \
#    src/feature_matching_tools.h \
#    src/feature_matching_metrics.h \
#    src/feature_matching_features.h \
#    src/shot_lrf_mod.h \
#    src/rops_estimation_fix.h
    #src/FeatureEstimator.h




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
    LIBS += -lboost_system \
            -lboost_thread \
            -lboost_regex \
            -lboost_program_options \
            -lboost_filesystem

    INCLUDEPATH += /usr/local/include/pcl-1.8
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
    INCLUDEPATH += /usr/local/include/eigen3/
    PKGCONFIG += gl glu x11 /usr/local/lib/pkgconfig/opencv.pc eigen3

    # Link Covis
    LIBS += -L$$PWD/../../libs/covis/covis/build/lib/ -lcovis
    INCLUDEPATH += $$PWD/../../libs/covis/covis/src

    #Link Halcon
    INCLUDEPATH += $$(HALCONROOT)/include/
    LIBS += -L$$(HALCONROOT)/lib/$$(HALCONARCH)/ -lhalconcpp -lhalconc

}

OTHER_FILES += \
    src/detect/CMakeLists.txt
