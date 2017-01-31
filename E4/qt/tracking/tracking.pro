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

win32 {
        INCLUDEPATH += C:\opencv\build\include
        debug: LIBS += -LC:\opencv\build\lib\Debug -lopencv_core247d -lopencv_highgui247d -lopencv_imgproc247d -lopencv_calib3d247d
        release: LIBS += -LC:\opencv\build\lib\Release -lopencv_core247 -lopencv_highgui247 -lopencv_imgproc247 -lopencv_calib3d247
}

unix: LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d
