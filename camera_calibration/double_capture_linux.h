#ifndef DOUBLE_CAPTURE_LINUX_H
#define DOUBLE_CAPTURE_LINUX_H

#include <QMainWindow>

#ifdef Q_OS_LINUX
#include "v4l2.hpp"
#include <QThread>
#include <QMessageBox>
#include <QFileDialog>
#include <QDate>

class LeftCaptureThread : public QThread
{
    Q_OBJECT
public:
    LeftCaptureThread();

    QImage majorImage;
    void stop();
    void init(int index);

protected:
    void run();

private:
    volatile int majorindex;
    volatile bool stopped;

signals:
    void CaptureDone(QImage image, int result);
};

class RightCaptureThread : public QThread
{
    Q_OBJECT
public:
    RightCaptureThread();

    QImage majorImage;
    void stop();
    void init(int index);

protected:
    void run();

private:
    volatile int majorindex;
    volatile bool stopped;

signals:
    void CaptureDone(QImage image, int result);
};

extern V4L2 v4l2_left;
extern V4L2 v4l2_right;

namespace Ui {
class double_capture;
}

class double_capture_linux : public QMainWindow
{
    Q_OBJECT

public:
    explicit double_capture_linux(QWidget *parent = nullptr);
    ~double_capture_linux();

private slots:
    void capture();
    void DealLeftCaptureDone(QImage image, int result);
    void DealRightCaptureDone(QImage image, int result);

private:
    Ui::double_capture *ui;
    LeftCaptureThread *capture_thread_l;
    RightCaptureThread *capture_thread_r;
    int err11_l, err19_l;
    int err11_r, err19_r;
    void open_left(int id);
    void close_left();
    void open_right(int id);
    void close_right();
    void closeEvent(QCloseEvent *event);
    QString save_path;
    bool open_left_flag = false;
    int last_left_id;
    QImage q_left_image;
    bool open_right_flag = false;
    int last_right_id;
    QImage q_right_image;
};

#endif

#endif // DOUBLE_CAPTURE_LINUX_H
