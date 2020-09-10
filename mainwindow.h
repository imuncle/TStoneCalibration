#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "camera_calibration/CameraCalibration.h"
#include "hand_eye_calibration/HandEyeCalibration.h"
#include "AboutUs.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void startCameraCalib();
    void startHandEyeCalib();
    void showIntro();

private:
    Ui::MainWindow *ui;
    CameraCalibration* camera_calibration;
    HandEyeCalibration* hand_eye_calibration;
    QAction *about;
    AboutUs *a;
};

#endif // MAINWINDOW_H
