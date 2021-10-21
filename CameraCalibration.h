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
#include <QResizeEvent>
#include <opencv2/opencv.hpp>
#include "findCorner.h"
#include "chessboard.h"
#include "choose_two_dir.h"
#include "choose_yaml.h"
#include "types.h"
#include "monocular_calib.h"
#include "binocular_calib.h"
#include "AboutUs.h"

QT_BEGIN_NAMESPACE
namespace Ui { class CameraCalibration; }
QT_END_NAMESPACE

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
    void receiveYamlPath(QString str);
    void chooseImage(QListWidgetItem* item, QListWidgetItem*);
    void reset();
    void saveUndistort();
    void DealThreadDone();
    void showIntro();

private:
    Ui::CameraCalibration *ui;
    choose_two_dir *d;
    choose_yaml *chooseYaml;
    AboutUs* a;
    QImage Mat2QImage(cv::Mat cvImg);
    void ShowIntro();
    void HiddenIntro();
    QAction *saveImage;
    QAction *about;
    std::vector<Img_t> imgs;
    std::vector<Stereo_Img_t> stereo_imgs;
    MonoCalib mono_calib;
    BinoCalib bino_calib;
    cv::Size img_size;
    bool fisheye_flag = false;
    bool distort_flag = false;
    FindcornerThread *findcorner_thread;
    bool thread_done = false;
    bool list_select_ = false;
protected:
    void resizeEvent(QResizeEvent* event) override;
};
#endif // CameraCalibration_H
