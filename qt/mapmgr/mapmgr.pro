#-------------------------------------------------
#
# Project created by QtCreator 2023-08-29T14:17:14
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = mapmgr
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    main.cpp \
    dialog.cpp \
    data/fileinfo.cpp \
    data/mapcollection.cpp \
    data/mapgroup.cpp \
    data/semanticinfo.cpp \
    ros_helper/publishermanager.cpp

HEADERS += \
    dialog.h \
    data/fileinfo.h \
    data/mapcollection.h \
    data/mapgroup.h \
    data/semanticinfo.h \
    ros_helper/publishermanager.h

FORMS += \
    dialog.ui

INCLUDEPATH += \
    /opt/ros/melodic/include \
    /home/lx/catkin_ws/devel/include

LIBS += -lboost_filesystem -lboost_system -lyaml-cpp \
    -L/opt/ros/melodic/lib \
    -lroscpp -lrosconsole -lroscpp_serialization -lrostime
