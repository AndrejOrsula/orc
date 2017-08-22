QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = orc

TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
        main.cpp \
        orc.cpp \
    spatialrotationconversions.cpp

HEADERS += \
        orc.h \
    spatialrotationconversions.h

FORMS += \
        orc.ui

RESOURCES += \
    graphics.qrc
