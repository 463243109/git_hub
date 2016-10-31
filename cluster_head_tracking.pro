TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    corner_point.cpp \
    drawfunction.cpp

INCLUDEPATH += D:/opencv/include \

LIBS+= \
        D:/opencv/lib/libopencv_* \

HEADERS += \
    corner_point.h \
    main.h

