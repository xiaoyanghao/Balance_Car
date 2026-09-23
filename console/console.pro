#QT       += core gui
QT       *= core gui serialport
QT       *= core gui network
QT       += quick
QT       += quickwidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

RC_ICONS = $$PWD/balance.ico

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    console_protocol.cpp \
    console_setup.cpp \
    main.cpp

HEADERS += \
    console_protocol.h \
    console_setup.h\

FORMS += \
    ui_parm.ui

#一次性引入自定义控件的所有头文件 懒得一个个拷贝
INCLUDEPATH += $$PWD/quc/include

#不同的构建套件 debug release 依赖不同的链接库
CONFIG(debug, debug|release){
LIBS += -L$$PWD/quc/ -lqucd
} else {
LIBS += -L$$PWD/quc/ -lquc
}




# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    qml.qrc \
    res.qrc
