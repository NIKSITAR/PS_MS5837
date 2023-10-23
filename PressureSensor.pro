QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp \
        ms5837.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
#target.path = /home/hydro/PressureSensor/bin
#INSTALLS += target

LIBS += -L"/home/hydronautics/rpi/sysroot/usr/lib"
LIBS += -lwiringPi
LIBS += -L"/home/hydronautics/rpi/sysroot/usr/lib/arm-linux-gnueabihf"
#LIBS += -larm-linux-gnueabihf
LIBS += -li2c

HEADERS += \
    ms5837.h
