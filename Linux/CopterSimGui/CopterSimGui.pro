#-------------------------------------------------
#
# Project created by QtCreator 2014-04-08T10:07:24
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

LIBS += -L../CopterSim -lCopterSim
INCLUDEPATH += ../CopterSim
DEPENDPATH += ../CopterSim

LIBS += -L../FaultCheck -lFaultCheck
INCLUDEPATH += ../FaultCheck
DEPENDPATH += ../FaultCheck

TARGET = CopterSimGui
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    joystick.cpp

HEADERS  += mainwindow.h \
    joystick.h

FORMS    += mainwindow.ui
