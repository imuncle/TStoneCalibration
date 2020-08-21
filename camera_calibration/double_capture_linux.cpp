#include "double_capture_linux.h"
#include "ui_double_capture.h"
#ifdef Q_OS_LINUX

double_capture_linux::double_capture_linux(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::double_capture)
{
    ui->setupUi(this);
    ui->capture->setFlat(true);
    connect(ui->capture, SIGNAL(clicked()), this, SLOT(capture()));
}

double_capture_linux::~double_capture_linux()
{
    delete ui;
}

void double_capture_linux::capture()
{
}

#endif
