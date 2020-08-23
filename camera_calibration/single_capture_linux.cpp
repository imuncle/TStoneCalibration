#include "single_capture_linux.h"
#include "ui_single_capture.h"
#ifdef Q_OS_LINUX

V4L2 v4l2;

single_capture_linux::single_capture_linux(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::single_capture)
{
    ui->setupUi(this);
    ui->capture->setFlat(true);
    connect(ui->capture, SIGNAL(clicked()), this, SLOT(capture()));
    capture_thread = new SingleCaptureThread();
    connect(capture_thread, SIGNAL(SingleCaptureDone(QImage, int)), this, SLOT(DealCaptureDone(QImage, int)));
    err11 = err19 = 0;
    int camcount = v4l2.GetDeviceCount();
    for(int i = 0; i < camcount; i++)
    {
        QAction *camera = ui->menu->addAction(v4l2.GetCameraName(i));
        camera->setCheckable(true);
        connect(camera, &QAction::triggered,[=](){
            if(last_id != i)
            {
                open_camera(i);
            }
            else
            {
                close_camera();
            }
        });
    }
}

single_capture_linux::~single_capture_linux()
{
    delete ui;
}

void single_capture_linux::open_camera(int id)
{
    close_camera();
    last_id = id;
    QList<QAction*> actions = ui->menu->actions();
    for(int i = 0; i < actions.length(); i++)
    {
        if(id == i)
        {
            actions[i]->setChecked(true);
        }
        else
            actions[i]->setChecked(false);
    }
    v4l2.StartRun(id);
    capture_thread->init(id);
    capture_thread->start();
    open_flag = true;
}

void single_capture_linux::close_camera()
{
    last_id = -1;
    capture_thread->stop();
    capture_thread->quit();
    capture_thread->wait();
    v4l2.StopRun();
    open_flag = false;
}

void single_capture_linux::DealCaptureDone(QImage image, int result)
{
    //超时后关闭视频
    //超时代表着VIDIOC_DQBUF会阻塞，直接关闭视频即可
    if(result == -1)
    {
        close_camera();

        ui->image->clear();
        ui->image->setText("获取设备图像超时！");
    }

    if(!image.isNull())
    {
        q_image = image;
        ui->image->clear();
        switch(result)
        {
        case 0:     //Success
            err11 = err19 = 0;
            if(image.isNull())
                ui->image->setText("画面丢失！");
            else
                ui->image->setPixmap(QPixmap::fromImage(image.scaled(ui->image->size())));

            break;
        case 11:    //Resource temporarily unavailable
            err11++;
            if(err11 == 10)
            {
                ui->image->clear();
                ui->image->setText("设备已打开，但获取视频失败！\n请尝试切换USB端口后断开重试！");
            }
            break;
        case 19:    //No such device
            err19++;
            if(err19 == 10)
            {
                ui->image->clear();
                ui->image->setText("设备丢失！");
            }
            break;
        }
    }
}

void single_capture_linux::capture()
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
    //使用系统时间来命名图片的名称，时间是唯一的，图片名也是唯一的
    QDateTime dateTime(QDateTime::currentDateTime());
    QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
    //创建图片保存路径名
    QString filename = save_path +"/" + QString("%1.jpg").arg(time);
    //保存一帧数据
    q_image.save(filename);
}

void single_capture_linux::closeEvent ( QCloseEvent *)
{
    close_camera();
}

SingleCaptureThread::SingleCaptureThread()
{
    stopped = false;
    majorindex = -1;
}

void SingleCaptureThread::stop()
{
    stopped = true;
}

void SingleCaptureThread::init(int index)
{
    stopped = false;
    majorindex = index;
}

void SingleCaptureThread::run()
{
    if(majorindex != -1)
    {
        while(!stopped)
        {
            msleep(1000/30);

            QImage img;
            int ret = v4l2.GetFrame();
            if(ret == 0)
            {
                int WV = v4l2.GetCurResWidth();
                int HV = v4l2.GetCurResHeight();
                img = QImage(v4l2.rgb24, WV, HV, QImage::Format_RGB888);
            }

            emit SingleCaptureDone(img, ret);
        }
    }
}

#endif
