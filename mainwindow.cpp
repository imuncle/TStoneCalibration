#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setFixedSize(this->width(), this->height());
    ui->CameraCalib->setFlat(true);
    connect(ui->CameraCalib, SIGNAL(clicked()), this, SLOT(startCameraCalib()));
    ui->HandEyeCalib->setFlat(true);
    connect(ui->HandEyeCalib, SIGNAL(clicked()), this, SLOT(startHandEyeCalib()));
    about = ui->menu->addAction("关于");
    QFont font = about->font();
    font.setPixelSize(12);
    about->setFont(font);
    connect(about, SIGNAL(triggered()), this, SLOT(showIntro()));

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showIntro()
{
    a = new AboutUs();
    QFont font = a->font();
    font.setPixelSize(12);
    a->setFont(font);
    a->setWindowTitle("关于TStoneCalibration");
    a->show();
}

void MainWindow::startCameraCalib()
{
    camera_calibration = new CameraCalibration();
    QFont font = camera_calibration->font();
    font.setPixelSize(12);
    camera_calibration->setFont(font);
    camera_calibration->setWindowTitle("相机标定");
    camera_calibration->show();
}

void MainWindow::startHandEyeCalib()
{
    hand_eye_calibration = new HandEyeCalibration();
    QFont font = hand_eye_calibration->font();
    font.setPixelSize(12);
    hand_eye_calibration->setFont(font);
    hand_eye_calibration->setWindowTitle("手眼标定");
    hand_eye_calibration->show();
}
