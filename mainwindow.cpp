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
