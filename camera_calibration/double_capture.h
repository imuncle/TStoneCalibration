#ifndef DOUBLE_CAPTURE_H
#define DOUBLE_CAPTURE_H

#include <QMainWindow>
#include <QCameraInfo>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QDateTime>
#include <QFileDialog>

namespace Ui {
class double_capture;
}

class double_capture : public QMainWindow
{
    Q_OBJECT

public:
    explicit double_capture(QWidget *parent = nullptr);
    ~double_capture();

private slots:
    void capture();
    void left_take_photo(int id, const QImage &image);
    void right_take_photo(int id, const QImage &image);

private:
    Ui::double_capture *ui;
    //摄像头对象指针
    QCamera* Camera_left;
    QCamera* Camera_right;
    QCameraInfo last_left;
    QCameraInfo last_right;
    //摄像头的取景器
    QCameraViewfinder* CameraViewFinder_left;
    QCameraViewfinder* CameraViewFinder_right;
    QList<QCameraInfo> camera_list;
    void open_left(QCameraInfo info);
    void close_left();
    void open_right(QCameraInfo info);
    void close_right();
    void closeEvent(QCloseEvent *event);
    //记录摄像头内容
    QCameraImageCapture* CameraImageCapture_left;
    QCameraImageCapture* CameraImageCapture_right;
    QString save_path;
    bool left_open = false, right_open = false;
};

#endif // DOUBLE_CAPTURE_H
