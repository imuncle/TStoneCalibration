QT       += core gui
QT += multimedia multimediawidgets
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

RC_ICONS = camera.ico

SOURCES += \
    chessboard.cpp \
    dialog.cpp \
    double_capture.cpp \
    findCorner.cpp \
    main.cpp \
    mainwindow.cpp \
    single_capture.cpp

HEADERS += \
    chessboard.h \
    dialog.h \
    double_capture.h \
    findCorner.h \
    mainwindow.h \
    single_capture.h

FORMS += \
    dialog.ui \
    double_capture.ui \
    mainwindow.ui \
    single_capture.ui

INCLUDEPATH += C:/Users/uncle/Desktop/OpenCV/install/include\
                            C:/Users/uncle/Desktop/OpenCV/install/include/opencv\
                            C:/Users/uncle/Desktop/OpenCV/install/include/opencv2

LIBS += C:/Users/uncle/Desktop/OpenCV/install/x86/mingw/lib/libopencv_core310.dll.a\
             C:/Users/uncle/Desktop/OpenCV/install/x86/mingw/lib/libopencv_calib3d310.dll.a\
             C:/Users/uncle/Desktop/OpenCV/install/x86/mingw/lib/libopencv_highgui310.dll.a\
             C:/Users/uncle/Desktop/OpenCV/install/x86/mingw/lib/libopencv_imgcodecs310.dll.a\
             C:/Users/uncle/Desktop/OpenCV/install/x86/mingw/lib/libopencv_imgproc310.dll.a\
             C:/Users/uncle/Desktop/OpenCV/install/x86/mingw/lib/libopencv_photo310.dll.a\
             C:/Users/uncle/Desktop/OpenCV/install/x86/mingw/lib/libopencv_shape310.dll.a\
             C:/Users/uncle/Desktop/OpenCV/install/x86/mingw/lib/libopencv_features2d310.dll.a

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    image.qrc
