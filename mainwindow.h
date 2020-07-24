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
    void setSideLen(double side);
    void showImg(cv::Mat img);

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
    int corner_width, corner_height;
    double corner_side;
    bool camera_opened;
};

class StereoRectify : public QMainWindow
{
    Q_OBJECT
public:
    StereoRectify(QPushButton* start,
                  QPushButton* stop,
                  QPushButton* dir,
                  QLineEdit* text,
                  QComboBox* left,
                  QComboBox* right,
                  QLabel* left_,
                  QLabel* right_,
                  QLabel* disparity,
                  QLabel* depth);
    ~StereoRectify();
    void setCameraNum(int num);

private slots:
    void choose_dir();
    void choose_camera_left(int id);
    void choose_camera_right(int id);
    void start();
    void stop();
    void nextFrame();

private:
    void ReadXml(QString path);
    void showImg(QLabel* img_view, cv::Mat img);
    QImage Mat2QImage(cv::Mat cvImg);
    QTimer* timer;
    QPushButton* start_button;
    QPushButton* stop_button;
    QPushButton* dir_choose;
    QLineEdit* dir_text;
    QComboBox* left_choose;
    QComboBox* right_choose;
    QLabel* left_img;
    QLabel* right_img;
    QLabel* disparity_img;
    QLabel* depth_img;
    cv::Mat left_mapx, left_mapy, right_mapx, right_mapy, Q;
    int current_left_camera, current_right_camera;
    cv::VideoCapture left_capture, right_capture;
    cv::Mat left_image, right_image, disparity_image, depth_image;
    cv::Ptr<cv::StereoSGBM> sgbm;
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
    void sideChange(double side);

private:
    Ui::MainWindow *ui;
    Camera* camera_1;
    Camera* camera_2;
    StereoRectify* stereo;
    QComboBox* choose_mode;
    cv::VideoCapture capture;
    QString mode;
    int corner_width, corner_height;
    double corner_side;
};

#endif // MAINWINDOW_H
