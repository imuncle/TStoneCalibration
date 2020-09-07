#include "imageundistort.h"
#include "ui_imageundistort.h"

ImageUndistort::ImageUndistort(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ImageUndistort)
{
    ui->setupUi(this);
}

ImageUndistort::~ImageUndistort()
{
    delete ui;
}
