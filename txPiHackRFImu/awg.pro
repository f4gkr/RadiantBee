#-------------------------------------------------
#
# Project created by QtCreator 2016-09-28T12:26:18
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG += c++11
TARGET = awg
TEMPLATE = app
LIBS += -lusb-1.0 -lliquid -lpthread -lgps
INCLUDEPATH += /usr/include/eigen3
#include( ./RTIMULib/RTIMULib.pri )
SOURCES += main.cpp\
        mainwindow.cpp \
    chirp.cpp \
    fsk.cpp \
    transmitter.cpp \
    hackrf.c \
    ahrs/ahrs.cpp \
    ahrs/I2CBus.cpp \
    ahrs/L3G.cpp \
    ahrs/LSM303.cpp \
    ahrs/MinIMU9.cpp

HEADERS  += mainwindow.h \
    chirp.h \
    datatypes.h \
    fsk.h \
    hackrf.h \
    transmitter.h \
    ahrs/exceptions.h \
    ahrs/ahrs.h \
    ahrs/I2CBus.h \
    ahrs/IMU.h \
    ahrs/L3G.h \
    ahrs/LSM303.h \
    ahrs/MinIMU9.h \
    ahrs/vector.h \
    ahrs/version.h

