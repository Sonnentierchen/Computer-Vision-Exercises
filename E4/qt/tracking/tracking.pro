QT += core
QT -= gui

CONFIG += c++11

TARGET = tracking
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    ../../TrackingEx/main.cpp \
    ../../TrackingEx/MotionModel.cpp \
    ../../TrackingEx/ObservationModel.cpp \
    ../../TrackingEx/Particle.cpp

HEADERS += \
    ../../TrackingEx/MotionModel.h \
    ../../TrackingEx/ObservationModel.h \
    ../../TrackingEx/Particle.h
