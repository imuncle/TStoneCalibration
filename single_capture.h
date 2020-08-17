#ifndef SINGLE_CAPTURE_H
#define SINGLE_CAPTURE_H

#include <QMainWindow>
#include <QCameraInfo>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QDateTime>
#include <QFileDialog>

namespace Ui {
class single_capture;
}

class single_capture : public QMainWindow
{
    Q_OBJECT

public:
    explicit single_capture(QWidget *parent = nullptr);
    ~single_capture();

private slots:
    void capture();
    void take_photo(int id, const QImage &image);

private:
    Ui::single_capture *ui;
    //摄像头对象指针
    QCamera* Camera;
    QCameraInfo last_info;
    //摄像头的取景器
    QCameraViewfinder* CameraViewFinder;
    void open(QCameraInfo info);
    void close();
    void closeEvent(QCloseEvent *event);
    //记录摄像头内容
    QCameraImageCapture* CameraImageCapture;
    QString save_path;
};

#endif // SINGLE_CAPTURE_H
