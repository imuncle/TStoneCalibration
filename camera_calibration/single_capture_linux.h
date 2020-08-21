#ifndef SINGLE_CAPTURE_LINUX_H
#define SINGLE_CAPTURE_LINUX_H

#include <QMainWindow>

#ifdef Q_OS_LINUX
#include "v4l2.hpp"
#include <QThread>
#include <QMessageBox>
#include <QFileDialog>
#include <QDate>

class SingleCaptureThread : public QThread
{
    Q_OBJECT
public:
    SingleCaptureThread();

    QImage majorImage;
    void stop();
    void init(int index);

protected:
    void run();

private:
    volatile int majorindex;
    volatile bool stopped;

signals:
    void SingleCaptureDone(QImage image, int result);
};

extern V4L2 v4l2;

namespace Ui {
class single_capture;
}

class single_capture_linux : public QMainWindow
{
    Q_OBJECT

public:
    explicit single_capture_linux(QWidget *parent = nullptr);
    ~single_capture_linux();

private slots:
    void capture();
    void DealCaptureDone(QImage image, int result);

private:
    Ui::single_capture *ui;
    SingleCaptureThread *capture_thread;
    int err11, err19;
    void open_camera(int id);
    void close_camera();
    QString save_path;
    bool open_flag = false;
    int last_id;
    QImage q_image;
};

#endif

#endif // SINGLE_CAPTURE_LINUX_H
