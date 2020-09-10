#ifndef HandEyeCalibration_H
#define HandEyeCalibration_H

#include <QMainWindow>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QSize>
#include <QSignalMapper>
#include <QProgressDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QListWidgetItem>
#include <QTextCodec>
#include <QCamera>
#include <opencv2/opencv.hpp>
#include "../camera_calibration/CameraCalibration.h"
#include "../camera_calibration/findCorner.h"
#include "../camera_calibration/chessboard.h"
#include "../camera_calibration/choose_two_dir.h"
#ifdef Q_OS_LINUX
#include "../camera_calibration/single_capture_linux.h"
#elif defined(Q_OS_WIN32)
#include "../camera_calibration/single_capture.h"
#endif

#ifdef Q_OS_LINUX
#include "../camera_calibration/double_capture_linux.h"
#elif defined(Q_OS_WIN32)
#include "../camera_calibration/double_capture.h"
#endif
#include "../camera_calibration/choose_yaml.h"

QT_BEGIN_NAMESPACE
namespace Ui { class HandEyeCalibration; }
QT_END_NAMESPACE

extern cv::Mat find_corner_thread_img;
extern struct Chessboarder_t find_corner_thread_chessboard;

class HandEyeCalibration : public QMainWindow
{
    Q_OBJECT

public:
    HandEyeCalibration(QWidget *parent = nullptr);
    ~HandEyeCalibration();

private slots:
    void addRobot();
    void addImage();
    void deleteImage();
    void calibrate();
    void fisheyeModeSwitch(int state);
    void exportParam();
    void distortModeSwitch();
    void receiveFromDialog(QString str);
    void receiveYamlPath(QString str);
    void chooseImage(QListWidgetItem* item, QListWidgetItem*);
    void reset();
    void undistortReset();
    void saveUndistort();
    void OpenCamera();
    void DealThreadDone();

private:
    Ui::HandEyeCalibration *ui;
    choose_two_dir *d;
    //choose_robot_dir *d_robot;
    choose_yaml *chooseYaml;
#ifdef Q_OS_LINUX
    single_capture_linux *single_c;
    double_capture_linux *double_c;
#elif defined(Q_OS_WIN32)
    single_capture *single_c;
    double_capture *double_c;
#endif
    QImage Mat2QImage(cv::Mat cvImg);
    void ShowIntro();
    void HiddenIntro();
    QAction *saveImage;
    std::vector<cv::Mat> robot_poses;
    std::vector<struct Img_t> imgs;
    std::vector<struct Stereo_Img_t> stereo_imgs;
    double chessboard_size;
    cv::Mat extrinsicMatrix;
    cv::Mat cameraMatrix;
    cv::Mat distCoefficients;
    cv::Matx33d K;
    cv::Vec4d D;
    cv::Mat cameraMatrix2;
    cv::Mat distCoefficients2;
    double error1, error2;
    cv::Matx33d K2;
    cv::Vec4d D2;
    cv::Size img_size;
    bool fisheye_flag = false;
    bool calibrate_flag = false;
    bool distort_flag = false;
    cv::Mat R, T, R1, R2, P1, P2, Q;
    FindcornerThread *findcorner_thread;
    bool thread_done = false;
};
#endif // HandEyeCalibration_H
