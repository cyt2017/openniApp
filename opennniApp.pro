#-------------------------------------------------
#
# Project created by QtCreator 2017-09-10T10:36:52
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = opennniApp
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    util/opennilzo.cpp \
    util/bLib.c \
    util/rngLib.c \
    driver/openninet.cpp \
    driver/openniinterface.cpp \
    driver/opennilocal.cpp \
    qimagelabel.cpp \
    pcl/qpclviewer.cpp

HEADERS  += mainwindow.h \
    util/bLib.h \
    util/opennilzo.h \
    util/rngLib.h \
    driver/openninet.h \
    driver/openniinterface.h \
    driver/opennilocal.h \
    qimagelabel.h \
    pcl/qpclviewer.h

FORMS    += mainwindow.ui


INCLUDEPATH += /home/cyt/workspace/OpenNI2/Include\
                /home/cyt/myLibs/include/pcl-1.8  \
                /usr/include/eigen3  \
                /home/cyt/myLibs/include/vtk-8.1  \
                /usr/include/boost

LIBS +=  /home/cyt/workspace/OpenNI2/Bin/x64-Release/libOpenNI2.so


INCLUDEPATH += /home/cyt/myLibs/include
LIBS += /home/cyt/myLibs/lib/liblzo2.a

INCLUDEPATH += /usr/include

LIBS += /usr/lib/x86_64-linux-gnu/libopencv*.so
LIBS+=-L/home/cyt/myLibs/lib /home/cyt/myLibs/lib/libvtk*.so
LIBS+=-lpcl_apps \
-lpcl_common \
-lpcl_features \
-lpcl_filters \
-lpcl_io_ply \
-lpcl_io \
-lpcl_kdtree \
-lpcl_keypoints \
-lpcl_octree \
-lpcl_outofcore \
-lpcl_people \
-lpcl_recognition \
-lpcl_registration \
-lpcl_sample_consensus \
-lpcl_search \
-lpcl_segmentation \
-lpcl_surface \
-lpcl_tracking \
-lpcl_visualization \
-lboost_system
