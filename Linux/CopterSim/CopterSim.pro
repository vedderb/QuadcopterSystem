#-------------------------------------------------
#
# Project created by QtCreator 2014-04-04T16:15:29
#
#-------------------------------------------------

QT       += network
QT       -= gui

TARGET = CopterSim
TEMPLATE = lib

LIBS += -L../FaultCheck -lFaultCheck
INCLUDEPATH += ../FaultCheck
DEPENDPATH += ../FaultCheck

DEFINES += COPTERSIM_LIBRARY

SOURCES += coptersim.cpp \
    coptermodel.cpp \
    itsstation.cpp \
    utility.cpp \
    coptersim_wrapper.cpp

HEADERS += coptersim.h\
        coptersim_global.h \
    coptermodel.h \
    itsstation.h \
    datatypes.h \
    utility.h \
    coptersim_wrapper.h

unix {
    target.path = /usr/local/lib
    INSTALLS += target
}
