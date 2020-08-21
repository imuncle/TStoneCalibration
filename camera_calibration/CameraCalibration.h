#ifndef CameraCalibration_H
#define CameraCalibration_H

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
#include "findCorner.h"
#include "chessboard.h"
#include "choose_two_dir.h"
#ifdef Q_OS_LINUX
#include "single_capture_linux.h"
#elif defined(Q_OS_WIN32)
#include "single_capture.h"
#endif

#ifdef Q_OS_LINUX
#include "double_capture_linux.h"
#elif defined(Q_OS_WIN32)
#include "double_capture.h"
#endif

QT_BEGIN_NAMESPACE
namespace Ui { class CameraCalibration; }
QT_END_NAMESPACE

struct Img_t
{
    cv::Mat img;
    QString file_name;
    std::vector<cv::Point2f> img_points;
    std::vector<cv::Point3f> world_points;
    cv::Mat tvec;
    cv::Mat rvec;
    cv::Vec3d fish_tvec;
    cv::Vec3d fish_rvec;
};

struct Stereo_Img_t
{
    cv::Mat left_img;
    QString left_file_name;
    std::vector<cv::Point2f> left_img_points;
    cv::Mat left_tvec;
    cv::Mat left_rvec;
    cv::Vec3d left_fish_tvec;
    cv::Vec3d left_fish_rvec;

    cv::Mat right_img;
    QString right_file_name;
    std::vector<cv::Point2f> right_img_points;
    cv::Mat right_tvec;
    cv::Mat right_rvec;
    cv::Vec3d right_fish_tvec;
    cv::Vec3d right_fish_rvec;

    std::vector<cv::Point3f> world_points;
};

extern cv::Mat find_corner_thread_img;
extern struct Chessboarder_t find_corner_thread_chessboard;

class CameraCalibration : public QMainWindow
{
    Q_OBJECT

public:
    CameraCalibration(QWidget *parent = nullptr);
    ~CameraCalibration();

private slots:
    void addImage();
    void deleteImage();
    void calibrate();
    void fisheyeModeSwitch(int state);
    void exportParam();
    void distortModeSwitch();
    void receiveFromDialog(QString str);
    void chooseImage(QListWidgetItem* item, QListWidgetItem*);
    void reset();
    void saveUndistort();
    void OpenCamera();
    void DealThreadDone();

private:
    Ui::CameraCalibration *ui;
    choose_two_dir *d;
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
    std::vector<struct Img_t> imgs;
    std::vector<struct Stereo_Img_t> stereo_imgs;
    double chessboard_size;
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
#endif // CameraCalibration_H
