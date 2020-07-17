#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <QTimer>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>

namespace Ui {
class Camera;
class MainWindow;
}

class Camera : public QMainWindow
{
    Q_OBJECT

public:
    Camera(QPushButton* choose_b,
           QPushButton* open_b,
           QPushButton* calibration_b,
           QPushButton* close_b,
           QLabel* dir_l,
           QLabel* inner_l,
           QLabel* ImageView_l,
           QCheckBox* correct_b,
           QComboBox* choose_camera,
           QStatusBar* bar);
    ~Camera();
    void setCameraNum(int num);
    void disable(bool state);
    void saveImage();
    QString getCurrentPath();
    void getCameraMatrix(cv::Mat* cameraMatrix, cv::Mat* distCoefficients);
    bool getInnerMatrix();
    void setWidth(int width);
    void setHeight(int height);
    void setSideLen(int side);

private slots:
    void chooseDir();
    void openCamera();
    void closeCamera();
    void calibration();
    void nextFrame();
    void correctImage(int state);
    void chooseCamera(int id);

private:
    QImage Mat2QImage(cv::Mat cvImg);
    bool readXml();
    double rate;
    cv::Mat img;
    cv::Mat mapx, mapy;
    QImage image;
    cv::VideoCapture capture;
    QTimer* timer;
    QPushButton* choose_dir_button;
    QPushButton* open_button;
    QPushButton* calibration_button;
    QPushButton* close_button;
    QLabel* dir_label;
    QLabel* inner_label;
    QLabel* ImageView;
    QCheckBox* correct_box;
    QComboBox* camera_id_choose;
    QStatusBar* status_bar;
    int current_camera_id;
    QString srcDirPath;
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    bool get_inner;
    bool correct_img;
    int save_img_id;
    int corner_width, corner_height, corner_side;
    bool camera_opened;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void GetCameraNum();
    void ModeChange(QString mode);
    void saveImage();
    void calibration();
    void widthChange(int width);
    void heightChange(int height);
    void sideChange(int side);

private:
    Ui::MainWindow *ui;
    Camera* camera_1;
    Camera* camera_2;
    QComboBox* choose_mode;
    cv::VideoCapture capture;
    QString mode;
    int corner_width, corner_height, corner_side;
};

#endif // MAINWINDOW_H
