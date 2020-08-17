#include "double_capture.h"
#include "ui_double_capture.h"
#include <QMessageBox>

double_capture::double_capture(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::double_capture)
{
    ui->setupUi(this);
    ui->capture->setFlat(true);
    connect(ui->capture, SIGNAL(clicked()), this, SLOT(capture()));
    QList<QCameraInfo> camera_list = QCameraInfo::availableCameras();
    if(camera_list.length() > 1)
    {
        Camera_left = new QCamera(camera_list[0]);
        Camera_right = new QCamera(camera_list[1]);
    }
    else
    {
        QMessageBox::warning(this, "警告", "摄像头少于两个，请插上摄像头后再打开该窗口", QMessageBox::Yes);
    }
    for(int i = 0; i < camera_list.length(); i++) {
        QAction *camera = ui->menu_1->addAction(camera_list[i].description());
        camera->setCheckable(true);
        connect(camera, &QAction::triggered,[=](){
            if(last_left != camera_list[i])
            {
                open_left(camera_list[i]);
                last_left = camera_list[i];
            }
            else
            {
                close_left();
                QString str("");
                last_left = QCameraInfo(str.toUtf8());
            }
        });
        QAction *camera_1 = ui->menu_2->addAction(camera_list[i].description());
        camera_1->setCheckable(true);
        connect(camera_1, &QAction::triggered,[=](){
            if(last_right != camera_list[i])
            {
                open_right(camera_list[i]);
                last_right = camera_list[i];
            }
            else
            {
                close_right();
                QString str("");
                last_right = QCameraInfo(str.toUtf8());
            }
        });
    }
    CameraViewFinder_left = new QCameraViewfinder(ui->left_img);
    CameraViewFinder_left->resize(ui->left_img->width(), ui->left_img->height());
    CameraViewFinder_right = new QCameraViewfinder(ui->right_img);
    CameraViewFinder_right->resize(ui->right_img->width(), ui->right_img->height());
}

double_capture::~double_capture()
{
    delete ui;
    Camera_left->stop();
    Camera_right->stop();
}

void double_capture::close_left()
{
    Camera_left->stop();
}

void double_capture::open_left(QCameraInfo info)
{
    QList<QAction*> actions = ui->menu_1->actions();
    for(int i = 0; i < actions.length(); i++)
    {
        if(actions[i]->text() == info.description())
        {
            actions[i]->setChecked(true);
        }
        else
            actions[i]->setChecked(false);
    }
    Camera_left->stop();
    Camera_left = new QCamera(info);
    Camera_left->setViewfinder(CameraViewFinder_left);
    //显示摄像头取景器
    CameraViewFinder_left->show();
    //开启摄像头
    Camera_left->start();

    //创建获取一帧数据对象
    CameraImageCapture_left = new QCameraImageCapture(Camera_left);
    //关联图像获取信号
    connect(CameraImageCapture_left, &QCameraImageCapture::imageCaptured, this, &double_capture::left_take_photo);
}

void double_capture::close_right()
{
    Camera_right->stop();
}

void double_capture::open_right(QCameraInfo info)
{
    QList<QAction*> actions = ui->menu_2->actions();
    for(int i = 0; i < actions.length(); i++)
    {
        if(actions[i]->text() == info.description())
        {
            actions[i]->setChecked(true);
        }
        else
            actions[i]->setChecked(false);
    }
    Camera_right->stop();
    Camera_right = new QCamera(info);
    Camera_right->setViewfinder(CameraViewFinder_right);
    //显示摄像头取景器
    CameraViewFinder_right->show();
    //开启摄像头
    Camera_right->start();

    //创建获取一帧数据对象
    CameraImageCapture_right = new QCameraImageCapture(Camera_right);
    //关联图像获取信号
    connect(CameraImageCapture_right, &QCameraImageCapture::imageCaptured, this, &double_capture::right_take_photo);
}

void double_capture::capture()
{
    if(save_path == "")
    {
        save_path = QFileDialog::getExistingDirectory(nullptr, "选择保存路径", "./");
        QDir dir;
        dir.mkdir(save_path+"/left/");
        dir.mkdir(save_path+"/right/");
    }
    CameraImageCapture_left->capture();
    CameraImageCapture_right->capture();
}

void double_capture::left_take_photo(int, const QImage &image)
{
    //使用系统时间来命名图片的名称，时间是唯一的，图片名也是唯一的
    QDateTime dateTime(QDateTime::currentDateTime());
    QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
    //创建图片保存路径名
    QString filename = save_path + "/left/" + QString("./%1.jpg").arg(time);
    //保存一帧数据
    image.save(filename);
}

void double_capture::right_take_photo(int, const QImage &image)
{
    //使用系统时间来命名图片的名称，时间是唯一的，图片名也是唯一的
    QDateTime dateTime(QDateTime::currentDateTime());
    QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
    //创建图片保存路径名
    QString filename = save_path + "/right/" + QString("./%1.jpg").arg(time);
    //保存一帧数据
    image.save(filename);
}

void double_capture::closeEvent ( QCloseEvent * e )
{
    Camera_left->stop();
    Camera_right->stop();
}
