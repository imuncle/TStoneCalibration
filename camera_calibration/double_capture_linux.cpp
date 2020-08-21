#include "double_capture_linux.h"
#include "ui_double_capture.h"
#ifdef Q_OS_LINUX

V4L2 v4l2_left;
V4L2 v4l2_right;

double_capture_linux::double_capture_linux(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::double_capture)
{
    ui->setupUi(this);
    ui->capture->setFlat(true);
    connect(ui->capture, SIGNAL(clicked()), this, SLOT(capture()));
    capture_thread_l = new LeftCaptureThread();
    connect(capture_thread_l, SIGNAL(CaptureDone(QImage, int)), this, SLOT(DealLeftCaptureDone(QImage, int)));
    capture_thread_r = new RightCaptureThread();
    connect(capture_thread_r, SIGNAL(CaptureDone(QImage, int)), this, SLOT(DealRightCaptureDone(QImage, int)));
    err11_l = err19_l = err11_r = err19_r = 0;
    int camcount = v4l2_left.GetDeviceCount();
    for(int i = 0; i < camcount; i++)
    {
        QAction *camera = ui->menu_1->addAction(v4l2_left.GetCameraName(i));
        camera->setCheckable(true);
        connect(camera, &QAction::triggered,[=](){
            if(last_left_id != i)
            {
                open_left(i);
            }
            else
            {
                close_left();
            }
        });
        QAction *camera_1 = ui->menu_1->addAction(v4l2_left.GetCameraName(i));
        camera_1->setCheckable(true);
        connect(camera_1, &QAction::triggered,[=](){
            if(last_right_id != i)
            {
                open_right(i);
            }
            else
            {
                close_right();
            }
        });
    }
}

double_capture_linux::~double_capture_linux()
{
    delete ui;
}

void double_capture_linux::open_left(int id)
{
    QList<QAction*> actions = ui->menu_1->actions();
    if(id == last_right_id)
    {
        QMessageBox::warning(NULL, "警告", "摄像头"+actions[id]->text()+"已经打开！", QMessageBox::Yes);
        close_left();
        for(int i = 0; i < actions.length(); i++)
        {
            actions[i]->setChecked(false);
        }
        return;
    }
    close_left();
    last_left_id = id;

    for(int i = 0; i < actions.length(); i++)
    {
        if(id == i)
        {
            actions[i]->setChecked(true);
        }
        else
            actions[i]->setChecked(false);
    }
    v4l2_left.StartRun(id);
    capture_thread_l->init(id);
    capture_thread_l->start();
    open_left_flag = true;
}

void double_capture_linux::close_left()
{
    last_left_id = -1;
    capture_thread_l->stop();
    capture_thread_l->wait();
    v4l2_left.StopRun();
    open_left_flag = false;
}

void double_capture_linux::open_right(int id)
{
    QList<QAction*> actions = ui->menu_2->actions();
    if(id == last_left_id)
    {
        QMessageBox::warning(NULL, "警告", "摄像头"+actions[id]->text()+"已经打开！", QMessageBox::Yes);
        close_right();
        for(int i = 0; i < actions.length(); i++)
        {
            actions[i]->setChecked(false);
        }
        return;
    }
    close_right();
    last_right_id = id;

    for(int i = 0; i < actions.length(); i++)
    {
        if(id == i)
        {
            actions[i]->setChecked(true);
        }
        else
            actions[i]->setChecked(false);
    }
    v4l2_right.StartRun(id);
    capture_thread_r->init(id);
    capture_thread_r->start();
    open_right_flag = true;
}

void double_capture_linux::close_right()
{
    last_right_id = -1;
    capture_thread_r->stop();
    capture_thread_r->wait();
    v4l2_right.StopRun();
    open_right_flag = false;
}

void double_capture_linux::DealLeftCaptureDone(QImage image, int result)
{
    //超时后关闭视频
    //超时代表着VIDIOC_DQBUF会阻塞，直接关闭视频即可
    if(result == -1)
    {
        capture_thread_l->stop();
        capture_thread_l->wait();
        v4l2_left.StopRun();

        ui->left_img->clear();
        ui->left_img->setText("获取设备图像超时！");
    }

    if(!image.isNull())
    {
        q_left_image = image;
        ui->left_img->clear();
        switch(result)
        {
        case 0:     //Success
            err11_l = err19_l = 0;
            if(image.isNull())
                ui->left_img->setText("画面丢失！");
            else
                ui->left_img->setPixmap(QPixmap::fromImage(image.scaled(ui->left_img->size())));

            break;
        case 11:    //Resource temporarily unavailable
            err11_l++;
            if(err11_l == 10)
            {
                ui->left_img->clear();
                ui->left_img->setText("设备已打开，但获取视频失败！\n请尝试切换USB端口后断开重试！");
            }
            break;
        case 19:    //No such device
            err19_l++;
            if(err19_l == 10)
            {
                ui->left_img->clear();
                ui->left_img->setText("设备丢失！");
            }
            break;
        }
    }
}

void double_capture_linux::DealRightCaptureDone(QImage image, int result)
{
    //超时后关闭视频
    //超时代表着VIDIOC_DQBUF会阻塞，直接关闭视频即可
    if(result == -1)
    {
        capture_thread_r->stop();
        capture_thread_r->wait();
        v4l2_right.StopRun();

        ui->right_img->clear();
        ui->right_img->setText("获取设备图像超时！");
    }

    if(!image.isNull())
    {
        q_right_image = image;
        ui->right_img->clear();
        switch(result)
        {
        case 0:     //Success
            err11_r = err19_r = 0;
            if(image.isNull())
                ui->right_img->setText("画面丢失！");
            else
                ui->right_img->setPixmap(QPixmap::fromImage(image.scaled(ui->right_img->size())));

            break;
        case 11:    //Resource temporarily unavailable
            err11_r++;
            if(err11_r == 10)
            {
                ui->right_img->clear();
                ui->right_img->setText("设备已打开，但获取视频失败！\n请尝试切换USB端口后断开重试！");
            }
            break;
        case 19:    //No such device
            err19_r++;
            if(err19_r == 10)
            {
                ui->right_img->clear();
                ui->right_img->setText("设备丢失！");
            }
            break;
        }
    }
}

void double_capture_linux::capture()
{
    if(open_left_flag == false || open_right_flag == false)
    {
        QMessageBox::critical(this, "错误", "两个相机都打开后才能拍照！", QMessageBox::Yes);
        return;
    }
    if(save_path == "")
    {
        save_path = QFileDialog::getExistingDirectory(nullptr, "选择保存路径", "./");
        QDir dir;
        dir.mkdir(save_path+"/left/");
        dir.mkdir(save_path+"/right/");
        return;
    }
    //使用系统时间来命名图片的名称，时间是唯一的，图片名也是唯一的
    QDateTime dateTime(QDateTime::currentDateTime());
    QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
    //创建图片保存路径名
    QString filename = save_path + "/left/" + QString("%1.jpg").arg(time);
    //保存一帧数据
    q_left_image.save(filename);
    filename = save_path + "/right/" + QString("%1.jpg").arg(time);
    q_right_image.save(filename);
}

void double_capture_linux::closeEvent ( QCloseEvent *)
{
    capture_thread_l->stop();
    capture_thread_l->wait();
    v4l2_left.StopRun();

    capture_thread_r->stop();
    capture_thread_r->wait();
    v4l2_right.StopRun();
}

LeftCaptureThread::LeftCaptureThread()
{
    stopped = false;
    majorindex = -1;
}

void LeftCaptureThread::stop()
{
    stopped = true;
}

void LeftCaptureThread::init(int index)
{
    stopped = false;
    majorindex = index;
}

void LeftCaptureThread::run()
{
    if(majorindex != -1)
    {
        while(!stopped)
        {
            msleep(1000/30);

            QImage img;
            int ret = v4l2_left.GetFrame();
            if(ret == 0)
            {
                int WV = v4l2_left.GetCurResWidth();
                int HV = v4l2_left.GetCurResHeight();
                img = QImage(v4l2_left.rgb24, WV, HV, QImage::Format_RGB888);
            }

            emit CaptureDone(img, ret);
        }
    }
}

RightCaptureThread::RightCaptureThread()
{
    stopped = false;
    majorindex = -1;
}

void RightCaptureThread::stop()
{
    stopped = true;
}

void RightCaptureThread::init(int index)
{
    stopped = false;
    majorindex = index;
}

void RightCaptureThread::run()
{
    if(majorindex != -1)
    {
        while(!stopped)
        {
            msleep(1000/30);

            QImage img;
            int ret = v4l2_right.GetFrame();
            if(ret == 0)
            {
                int WV = v4l2_right.GetCurResWidth();
                int HV = v4l2_right.GetCurResHeight();
                img = QImage(v4l2_right.rgb24, WV, HV, QImage::Format_RGB888);
            }

            emit CaptureDone(img, ret);
        }
    }
}

#endif
