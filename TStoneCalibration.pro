QT       += core gui
QT += multimedia multimediawidgets
QT += network
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

RC_ICONS = icon.ico

SOURCES += \
    AboutUs.cpp \
    camera_calibration/CameraCalibration.cpp \
    camera_calibration/chessboard.cpp \
    camera_calibration/choose_two_dir.cpp \
    camera_calibration/double_capture.cpp \
    camera_calibration/double_capture_linux.cpp \
    camera_calibration/findCorner.cpp \
    camera_calibration/single_capture_linux.cpp \
    hand_eye_calibration/HandEyeCalibration.cpp \
    hand_eye_calibration/TSAIleastSquareCalibration.cpp \
    main.cpp \
    camera_calibration/single_capture.cpp \
    camera_calibration/choose_yaml.cpp \
    mainwindow.cpp

HEADERS += \
    AboutUs.h \
    camera_calibration/CameraCalibration.h \
    camera_calibration/chessboard.h \
    camera_calibration/choose_two_dir.h \
    camera_calibration/double_capture.h \
    camera_calibration/double_capture_linux.h \
    camera_calibration/findCorner.h \
    camera_calibration/single_capture.h \
    camera_calibration/single_capture_linux.h \
    camera_calibration/v4l2.hpp \
    camera_calibration/choose_yaml.h \
    hand_eye_calibration/HandEyeCalibration.h \
    hand_eye_calibration/TSAIleastSquareCalibration.h \
    mainwindow.h

FORMS += \
    AboutUs.ui \
    camera_calibration/CameraCalibration.ui \
    camera_calibration/choose_two_dir.ui \
    camera_calibration/double_capture.ui \
    camera_calibration/single_capture.ui \
    camera_calibration/choose_yaml.ui \
    hand_eye_calibration/HandEyeCalibration.ui \
    mainwindow.ui

INCLUDEPATH += \
    C:\opencv\opencv64-build\install\include \
    C:\opencv\opencv64-build\install\include\opencv \
    C:\opencv\opencv64-build\install\include\opencv2

LIBS += \
    C:\opencv\opencv64-build\install\x64\mingw\bin\libopencv_*.dll

#    C:/opencv/build/x86/mingw/lib/libopencv_core310.dll.a\
#    C:/opencv/build/x86/mingw/lib/libopencv_calib3d310.dll.a\
#    C:/opencv/build/x86/mingw/lib/libopencv_highgui310.dll.a\
#    C:/opencv/build/x86/mingw/lib/libopencv_imgcodecs310.dll.a\
#    C:/opencv/build/x86/mingw/lib/libopencv_imgproc310.dll.a\
#    C:/opencv/build/x86/mingw/lib/libopencv_features2d310.dll.a

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    camera_calibration/res/image.qrc \
    hand_eye_calibration/res/image.qrc
