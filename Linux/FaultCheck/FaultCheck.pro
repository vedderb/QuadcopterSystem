#-------------------------------------------------
#
# Project created by QtCreator 2013-03-27T09:27:43
#
#-------------------------------------------------

QT       += network

QT       -= gui

TARGET = FaultCheck
TEMPLATE = lib

DEFINES += FAULTCHECK_LIBRARY

SOURCES += \
    faultinfo.cpp \
    faultcheck.cpp \
    faultcheck_wrapper.cpp \
    packettool.cpp \
    trigger.cpp \
    utils.cpp \
    packetfaultinfo.cpp \
    packetinfo.cpp \
    faultcheck_packet_wrapper.cpp

HEADERS +=\
    faultinfo.h \
    faultcheck_types.h \
    faultcheck.h \
    FaultCheck_global.h \
    faultcheck_wrapper.h \
    packettool.h \
    trigger.h \
    utils.h \
    packetfaultinfo.h \
    packetinfo.h \
    faultcheck_packet_wrapper.h

symbian {
    MMP_RULES += EXPORTUNFROZEN
    TARGET.UID3 = 0xE0AC527F
    TARGET.CAPABILITY = 
    TARGET.EPOCALLOWDLLDATA = 1
    addFiles.sources = FaultLib.dll
    addFiles.path = !:/sys/bin
    DEPLOYMENT += addFiles
}

unix {
    target.path = /usr/local/lib
    INSTALLS += target
}
