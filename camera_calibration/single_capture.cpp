#include "single_capture.h"
#include "ui_single_capture.h"
#include <iostream>
#include <QMessageBox>

single_capture::single_capture(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::single_capture)
{
    ui->setupUi(this);
    ui->capture->setFlat(true);
    connect(ui->capture, SIGNAL(clicked()), this, SLOT(capture()));
    last_info.deviceName() = QString("");
    camera_list = QCameraInfo::availableCameras();
    if(camera_list.length() > 0)
        Camera = new QCamera(camera_list[0]);
    else
    {
        QString str("");
        Camera = new QCamera(QCameraInfo(str.toUtf8()));
        QMessageBox::warning(this, "警告", "没有检测到摄像头，请插上摄像头后再打开该窗口", QMessageBox::Yes);
        return;
    }
    for(int i = 0; i < camera_list.length(); i++) {
        QAction *camera = ui->menu->addAction(camera_list[i].description());
        camera->setCheckable(true);
        connect(camera, &QAction::triggered,[=](){
            if(last_info.deviceName() != camera_list[i].deviceName())
            {
                open(camera_list[i]);
            }
            else
            {
                close();
            }
        });
    }
    CameraViewFinder = new QCameraViewfinder(ui->image);
    CameraViewFinder->resize(ui->image->width(), ui->image->height());
}

single_capture::~single_capture()
{
    Camera->stop();
    delete Camera;
    delete ui;
}

void single_capture::close()
{
    Camera->stop();
    last_info = QCameraInfo(QString("").toUtf8());
    open_flag = false;
}

void single_capture::open(QCameraInfo info)
{
    QList<QAction*> actions = ui->menu->actions();
    for(int i = 0; i < actions.length(); i++)
    {
        if(camera_list[i].deviceName() == info.deviceName())
        {
            actions[i]->setChecked(true);
        }
        else
            actions[i]->setChecked(false);
    }
    Camera->stop();
    Camera = new QCamera(info);
    Camera->setViewfinder(CameraViewFinder);
    //显示摄像头取景器
    CameraViewFinder->show();
    //开启摄像头
    try {
        Camera->start();
    } catch (QEvent *e) {
        QMessageBox::critical(this, "错误", "摄像头打开失败", QMessageBox::Yes);
    }
    if(Camera->status() != QCamera::ActiveStatus)
    {
        QMessageBox::warning(this, "警告", "摄像头打开失败！", QMessageBox::Yes);
        close();
        return;
    }
    //创建获取一帧数据对象
    CameraImageCapture = new QCameraImageCapture(Camera);
    //关联图像获取信号
    connect(CameraImageCapture, &QCameraImageCapture::imageCaptured, this, &single_capture::take_photo);
    open_flag = true;
    last_info = info;
}

void single_capture::capture()
{
    if(open_flag == false)
    {
        QMessageBox::critical(this, "错误", "请先选择相机！", QMessageBox::Yes);
        return;
    }
    if(save_path == "")
    {
        save_path = QFileDialog::getExistingDirectory(nullptr, "选择保存路径", "./");
        return;
    }
    CameraImageCapture->capture();
}

void single_capture::take_photo(int, const QImage &image)
{
    //使用系统时间来命名图片的名称，时间是唯一的，图片名也是唯一的
    QDateTime dateTime(QDateTime::currentDateTime());
    QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
    //创建图片保存路径名
    QString filename = save_path + QString("./%1.jpg").arg(time);
    //保存一帧数据
    image.save(filename);
}

void single_capture::closeEvent ( QCloseEvent *)
{
    Camera->stop();
}
