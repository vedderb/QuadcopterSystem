#-------------------------------------------------
#
# Project created by QtCreator 2013-09-23T15:41:25
#
#-------------------------------------------------

QT       += core gui
QT       += network
QT       += opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
greaterThan(QT_MAJOR_VERSION, 4): QT += printsupport

TARGET = QuadcopterTool
TEMPLATE = app

win32 {
    LIBS += -lws2_32
    LIBS += -L"C:\Program Files (x86)\Assimp\lib"
    INCLUDEPATH += "C:/Program Files/Assimp/include"
    DEPENDPATH += "C:/Program Files/Assimp/include"
}

LIBS += -lassimp

SOURCES += main.cpp\
    mainwindow.cpp \
    qcustomplot.cpp \
    perspectivepixmap.cpp \
    mapwidget.cpp \
    locpoint.cpp \
    packetinterface.cpp \
    utility.cpp \
    objectinfo.cpp \
    orientationwidget.cpp \
    comm.cpp \
    serialization.cpp \
    simviewer.cpp

unix:SOURCES += joystick.cpp
unix:SOURCES += serialport.cpp

win32:SOURCES += joystick_win.cpp
win32:SOURCES += serial_win.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    perspectivepixmap.h \
    mapwidget.h \
    locpoint.h \
    serialport.h \
    packetinterface.h \
    utility.h \
    objectinfo.h \
    orientationwidget.h \
    joystick.h \
    comm.h \
    datatypes.h \
    serialization.h \
    simviewer.h

FORMS    += mainwindow.ui \
    simviewer.ui
