TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += main.cpp \
    settings.cpp \
    uart_gestion.cpp \
    write_file.cpp \
    calculateTimestamp.cpp

#include(deployment.pri)
#qtcAddDeployment()

HEADERS += \
    settings.h \
    uart_gestion.h \
    write_file.h \
    VCD_definitions.h \
    calculateTimestamp.h

