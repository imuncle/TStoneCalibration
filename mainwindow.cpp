#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <iostream>
#include <QtXml>
#include <fstream>
#include "elas/elas.h"
#include "elas/image.h"

Camera::Camera(QPushButton *choose_b,
               QPushButton *open_b,
               QPushButton *calibration_b,
               QPushButton *close_b,
               QLabel *dir_l,
               QLabel *inner_l,
               QLabel *ImageView_l,
               QCheckBox* correct_b,
               QComboBox* choose_camera,
               QStatusBar* bar)
{
    choose_dir_button = choose_b;
    open_button = open_b;
    calibration_button = calibration_b;
    close_button = close_b;
    dir_label = dir_l;
    inner_label = inner_l;
    ImageView = ImageView_l;
    correct_box = correct_b;
    camera_id_choose = choose_camera;
    status_bar = bar;

    get_inner = false;
    correct_img = false;
    save_img_id = 0;
    camera_opened = false;

    connect(choose_dir_button, SIGNAL(clicked()), this, SLOT(chooseDir()));
    connect(open_button, SIGNAL(clicked()), this, SLOT(openCamera()));
    connect(calibration_button, SIGNAL(clicked()), this, SLOT(calibration()));
    connect(close_button, SIGNAL(clicked()), this, SLOT(closeCamera()));
    connect(correct_box, SIGNAL(stateChanged(int)), this, SLOT(correctImage(int)));
    connect(camera_id_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(chooseCamera(int)));

    open_button->setDisabled(true);
    calibration_button->setDisabled(true);
    close_button->setDisabled(true);
    correct_box->setDisabled(true);

    timer = new QTimer(this);
}

Camera::~Camera()
{
    delete choose_dir_button;
    delete open_button;
    delete calibration_button;
    delete close_button;
    delete dir_label;
    delete inner_label;
    delete ImageView;
    delete correct_box;
}

QImage Camera::Mat2QImage(cv::Mat cvImg)
{
    QImage qImg;
    if(cvImg.channels()==3)                             //3 channels color image
    {
        cv::cvtColor(cvImg,cvImg,CV_BGR2RGB);
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols, cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_RGB888);
    }
    else if(cvImg.channels()==1)                    //grayscale image
    {
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols,cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_Indexed8);
    }
    else
    {
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols,cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_RGB888);
    }
    return qImg;
}

void Camera::nextFrame()
{
    capture >> img;
    if(!img.empty())
    {
        if(correct_img == true)
        {
            cv::Mat cameraMatrix, distCoeffMatrix;
            getCameraMatrix(&cameraMatrix, &distCoeffMatrix);
            cv::Size image_size = img.size();
            cv::initUndistortRectifyMap(cameraMatrix, distCoeffMatrix, cv::Mat::eye(3, 3, CV_32FC1), cameraMatrix, image_size, CV_32FC1, mapx, mapy);
            cv::remap(img, img, mapx, mapy, cv::INTER_LINEAR);
        }
        showImg(img);
    }
}

void Camera::setCameraNum(int num)
{
    disconnect(camera_id_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(chooseCamera(int)));
    camera_id_choose->clear();
    for(int i = 0; i < num; i++)
    {
        camera_id_choose->addItem(QString::asprintf("%d",i));
    }
    connect(camera_id_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(chooseCamera(int)));
    open_button->setEnabled(true);
    close_button->setEnabled(true);
    camera_opened = false;
    current_camera_id = 0;
}

void Camera::chooseCamera(int id)
{
    current_camera_id = id;
    open_button->setEnabled(true);
    calibration_button->setEnabled(true);
    close_button->setEnabled(true);
    camera_opened = false;
}

void Camera::openCamera()
{
    if(capture.isOpened())
        capture.release();
    capture.open(current_camera_id);
    if(capture.isOpened())
    {
        rate = 30;
        timer->setInterval(1000/rate);
        connect(timer, SIGNAL(timeout()), this, SLOT(nextFrame()));
        timer->start();
        status_bar->showMessage(tr("Open Camera"), 1000);
        close_button->setEnabled(true);
        camera_opened = true;
    }
    else
    {
        camera_opened = false;
        status_bar->showMessage(tr("Camera Error"), 1000);
    }
}

void Camera::closeCamera()
{
    camera_opened = false;
    if(capture.isOpened())
        capture.release();
    timer->stop();
    status_bar->showMessage(tr("Camera Stop"), 1000);
}

void Camera::saveImage()
{
    if(camera_opened)
    {
        if(img.channels() == 3)
        {
            cv::cvtColor(img, img, CV_RGB2BGR);
        }
        cv::imwrite(srcDirPath.toStdString()+"/"+std::to_string(save_img_id)+".jpg", img);
        save_img_id++;
    }
}

bool Camera::readXml()
{
    QFile file(srcDirPath+"/calibration.xml"); //相对路径、绝对路径、资源路径都行
    if(!file.open(QFile::ReadOnly))
        return false;
    QDomDocument doc;
    if(!doc.setContent(&file))
    {
        file.close();
        return false;
    }
    file.close();
    QDomElement inner_camera = doc.documentElement();
    QDomNode node = inner_camera.firstChild();
    node = node.nextSibling();
    QDomElement e = node.toElement();
    fx = e.attribute("fx").toDouble();
    fy = e.attribute("fy").toDouble();
    cx = e.attribute("cx").toDouble();
    cy = e.attribute("cy").toDouble();
    node = node.nextSibling();
    e = node.toElement();
    k1 = e.attribute("k1").toDouble();
    k2 = e.attribute("k2").toDouble();
    k3 = e.attribute("k3").toDouble();
    p1 = e.attribute("p1").toDouble();
    p2 = e.attribute("p2").toDouble();
    return true;
}

void Camera::chooseDir()
{
    srcDirPath = QFileDialog::getExistingDirectory(nullptr, "Choose Directory", "./");
    if(srcDirPath.isEmpty())
    {
        return;
    }
    else
    {
        dir_label->setText(srcDirPath);
        calibration_button->setEnabled(true);
        QFileInfo fileInfo(srcDirPath+"/calibration.xml");
        if(fileInfo.isFile())
        {
            if(readXml() == true)
            {
                inner_label->setEnabled(true);
                correct_box->setEnabled(true);
                get_inner = true;
            }
            else
            {
                inner_label->setDisabled(true);
                get_inner = false;
            }
        }
        else
        {
            inner_label->setDisabled(true);
            get_inner = false;
        }
        save_img_id = 0;
    }
}

void Camera::calibration()
{
    //目录地址
    QDir dir(srcDirPath);
    dir.setFilter(QDir::Files | QDir::NoSymLinks);
    QStringList filters;
    filters << "*.png" << "*.jpg" << "*.jpeg";
    dir.setNameFilters(filters);
    QStringList imagesList = dir.entryList();
    if(imagesList.length() < 2)
    {
        inner_label->setDisabled(true);
        get_inner = false;
        status_bar->showMessage(tr("No enough image"), 1000);
        return;
    }

    status_bar->showMessage(tr("calibration processing..."), 1000);
    cv::Size image_size;//保存图片大小
    cv::Size pattern_size = cv::Size(corner_width, corner_height);
    std::vector<cv::Point2f> corner_points_buf;//建一个数组缓存检测到的角点
    std::vector<std::vector<cv::Point2f>> corner_points_of_all_imgs;
    int image_num = 0;
    QString filename;
    while(image_num < imagesList.length())
    {
        filename = srcDirPath + "/" + imagesList[image_num];
        image_num++;
        cv::Mat image_input = cv::imread(filename.toStdString());
        image_size.width = image_input.cols;
        image_size.height = image_input.rows;

        if(cv::findChessboardCorners(image_input, pattern_size, corner_points_buf) == 0)
        {
            continue;
        }
        else
        {
            cv::Mat gray;
            cv::cvtColor(image_input, gray, CV_RGB2GRAY);
            cv::find4QuadCornerSubpix(gray, corner_points_buf, pattern_size);
            corner_points_of_all_imgs.push_back(corner_points_buf);
        }
    }
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    cv::Mat distCoefficients = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
    std::vector<cv::Mat> tvecsMat;
    std::vector<cv::Mat> rvecsMat;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    int i,j,k;
    for(k=0;k<image_num;k++)
    {
        std::vector<cv::Point3f> tempCornerPoints;
        for(i=0; i<pattern_size.height;i++)
        {
            for(j=0;j<pattern_size.width;j++)
            {
                cv::Point3f singleRealPoint;
                singleRealPoint.x = j * corner_side;
                singleRealPoint.y = i * corner_side;
                singleRealPoint.z = 0;
                tempCornerPoints.push_back(singleRealPoint);
            }
        }
        objectPoints.push_back(tempCornerPoints);
    }

    double rms = cv::calibrateCamera(objectPoints, corner_points_of_all_imgs, image_size, cameraMatrix, distCoefficients, rvecsMat, tvecsMat, 0);
    fx = cameraMatrix.at<double>(0,0);
    fy = cameraMatrix.at<double>(1,1);
    cx = cameraMatrix.at<double>(0,2);
    cy = cameraMatrix.at<double>(1,2);
    k1 = distCoefficients.at<double>(0,0);
    k2 = distCoefficients.at<double>(0,1);
    p1 = distCoefficients.at<double>(0,2);
    p2 = distCoefficients.at<double>(0,3);
    k3 = distCoefficients.at<double>(0,4);

    QDomDocument doc;
    QDomElement inner_matrix=doc.createElement("inner_matrix");
    doc.appendChild(inner_matrix);
    QDomElement error = doc.createElement("error");
    error.setAttribute("e", rms);
    inner_matrix.appendChild(error);
    QDomElement camera_Matrix = doc.createElement("cameraMatrix");
    camera_Matrix.setAttribute("fx",fx);
    camera_Matrix.setAttribute("fy",fy);
    camera_Matrix.setAttribute("cx",cx);
    camera_Matrix.setAttribute("cy",cy);
    inner_matrix.appendChild(camera_Matrix);
    QDomElement distCoefficients_Matrix = doc.createElement("distCoefficients");
    distCoefficients_Matrix.setAttribute("k1",k1);
    distCoefficients_Matrix.setAttribute("k2",k2);
    distCoefficients_Matrix.setAttribute("p1",p1);
    distCoefficients_Matrix.setAttribute("p2",p2);
    distCoefficients_Matrix.setAttribute("k3",k3);
    inner_matrix.appendChild(distCoefficients_Matrix);

    QFile file(srcDirPath+"/calibration.xml");
    file.open(QFile::WriteOnly|QFile::Truncate);
    QTextStream out_stream(&file);
    doc.save(out_stream,4); //缩进4格
    file.close();

    cv::initUndistortRectifyMap(cameraMatrix, distCoefficients, cv::noArray(), cameraMatrix, image_size, CV_16SC2, mapx, mapy);
    filename = srcDirPath + "/" + imagesList[0];
    img = cv::imread(filename.toStdString());
    cv::remap(img, img, mapx, mapy, cv::INTER_LINEAR);
    showImg(img);
    status_bar->showMessage(tr("Done"), 1000);
    inner_label->setEnabled(true);
    get_inner = true;
    correct_box->setEnabled(true);
}

void Camera::correctImage(int state)
{
    correct_img = state;
}

void Camera::disable(bool state)
{
    if(state == true)
    {
        choose_dir_button->setDisabled(true);
        open_button->setDisabled(true);
        calibration_button->setDisabled(true);
        close_button->setDisabled(true);
        inner_label->setDisabled(true);
        correct_box->setDisabled(true);
        camera_id_choose->setDisabled(true);
        open_button->setDisabled(true);
        get_inner = false;
    }
    else
    {
        choose_dir_button->setEnabled(true);
        camera_id_choose->setEnabled(true);
    }
}

QString Camera::getCurrentPath()
{
    return srcDirPath;
}

void Camera::getCameraMatrix(cv::Mat *cameraMatrix, cv::Mat *distCoefficients)
{
    *cameraMatrix = (cv::Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    *distCoefficients = (cv::Mat_<double>(1,5)<<k1,k2,p1,p2,k3);
}

bool Camera::getInnerMatrix()
{
    return get_inner;
}

void Camera::setWidth(int width)
{
    corner_width = width;
}

void Camera::setHeight(int height)
{
    corner_height = height;
}

void Camera::setSideLen(double side)
{
    corner_side = side;
}

void Camera::showImg(cv::Mat img)
{
    image = Mat2QImage(img);
    ImageView->setPixmap(QPixmap::fromImage(image));
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->ImageView_1->setScaledContents(true);
    ui->ImageView_2->setScaledContents(true);
    ui->left->setScaledContents(true);
    ui->right->setScaledContents(true);
    ui->disparity->setScaledContents(true);
    ui->depth->setScaledContents(true);
    camera_1 = new Camera(ui->choose_dir_1,
                          ui->open_1,
                          ui->calibration_1,
                          ui->close_1,
                          ui->dir_1,
                          ui->inner_1,
                          ui->ImageView_1,
                          ui->correct_1,
                          ui->choose_camera_1,
                          ui->statusBar);
    camera_2 = new Camera(ui->choose_dir_2,
                          ui->open_2,
                          ui->calibration_2,
                          ui->close_2,
                          ui->dir_2,
                          ui->inner_2,
                          ui->ImageView_2,
                          ui->correct_2,
                          ui->choose_camera_2,
                          ui->statusBar);
    camera_2->disable(true);
    stereo = new StereoRectify(ui->two_begin,
                               ui->two_stop,
                               ui->two_dir,
                               ui->two_dir_text,
                               ui->left_camera,
                               ui->right_camera,
                               ui->left,
                               ui->right,
                               ui->disparity,
                               ui->depth);
    ui->choose_mode->addItem(tr("single"));
    ui->choose_mode->addItem(tr("double"));
    ui->two_calibration->setDisabled(true);
    mode = "single";
    connect(ui->save, SIGNAL(clicked()), this, SLOT(saveImage()));
    connect(ui->two_calibration, SIGNAL(clicked()), this, SLOT(calibration()));
    connect(ui->choose_mode, SIGNAL(currentIndexChanged(QString)), this, SLOT(ModeChange(QString)));
    connect(ui->refresh_camera_button, SIGNAL(clicked()), this, SLOT(GetCameraNum()));
    connect(ui->corner_width, SIGNAL(valueChanged(int)), this, SLOT(widthChange(int)));
    connect(ui->corner_height, SIGNAL(valueChanged(int)), this, SLOT(heightChange(int)));
    connect(ui->side_len, SIGNAL(valueChanged(double)), this, SLOT(sideChange(double)));
    ui->corner_height->setValue(8);
    ui->corner_width->setValue(11);
    ui->side_len->setValue(30);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete camera_1;
    delete camera_2;
}

void MainWindow::ModeChange(QString mode_)
{
    mode = mode_;
    if(mode_ == tr("single"))
    {
        ui->two_calibration->setDisabled(true);
        camera_2->disable(true);
    }
    else if(mode_ == tr("double"))
    {
        ui->two_calibration->setEnabled(true);
        camera_2->disable(false);
    }
}

void MainWindow::GetCameraNum()
{
    int i = 0;
    for(i = 0; i < 10; i++)
    {
        capture.open(i);
        if(!capture.isOpened())
        {
            break;
        }
        capture.release();
    }
    camera_1->setCameraNum(i);
    camera_2->setCameraNum(i);
    stereo->setCameraNum(i);
    char str[10];
    sprintf(str, "camera num: %d", i);
    ui->statusBar->showMessage(tr(str), 1000);
}

void MainWindow::saveImage()
{
    camera_1->saveImage();
    if(mode == "double")
    {
        camera_2->saveImage();
    }
}

void MainWindow::calibration()
{
    if(!camera_1->getInnerMatrix() || !camera_2->getInnerMatrix())
    {
        ui->statusBar->showMessage(tr("camera not calibrate!"), 1000);
        return;
    }
    cv::Mat left_camera_matrix, left_distcoeff_matrix, right_camera_matrix, right_distcoeff_matrix;
    camera_1->getCameraMatrix(&left_camera_matrix, &left_distcoeff_matrix);
    camera_2->getCameraMatrix(&right_camera_matrix, &right_distcoeff_matrix);
    ui->statusBar->showMessage(tr("two_camera calibration process..."), 1000);
    QString left_path, right_path;
    left_path = camera_1->getCurrentPath();
    right_path = camera_2->getCurrentPath();
    QDir dir(left_path);
    dir.setFilter(QDir::Files | QDir::NoSymLinks);
    QStringList filters;
    filters << "*.png" << "*.jpg" << "*.jpeg";
    dir.setNameFilters(filters);
    QStringList imagesList = dir.entryList();

    cv::Size image_size;//保存图片大小
    cv::Size pattern_size = cv::Size(corner_width, corner_height);

    std::vector<cv::Point2f> corner_points_buf;//建一个数组缓存检测到的角点
    std::vector<std::vector<cv::Point2f>> left_corner_points_of_all_imgs;
    std::vector<std::vector<cv::Point2f>> right_corner_points_of_all_imgs;
    int image_num = 0;
    QString filename;
    while(image_num < imagesList.length())
    {
        // 左侧相机
        filename = left_path + "/" + imagesList[image_num];
        cv::Mat image_input = cv::imread(filename.toStdString());
        image_size = image_input.size();

        if(cv::findChessboardCorners(image_input, pattern_size, corner_points_buf) == 0)
        {
            continue;
        }
        else
        {
            cv::Mat gray;
            cv::cvtColor(image_input, gray, CV_RGB2GRAY);
            cv::find4QuadCornerSubpix(gray, corner_points_buf, pattern_size);
            left_corner_points_of_all_imgs.push_back(corner_points_buf);
        }

        //  右侧相机
        filename = right_path + "/" + imagesList[image_num];
        image_num++;
        image_input = cv::imread(filename.toStdString());
        if(cv::findChessboardCorners(image_input, pattern_size, corner_points_buf) == 0)
        {
            continue;
        }
        else
        {
            cv::Mat gray;
            cv::cvtColor(image_input, gray, CV_RGB2GRAY);
            cv::find4QuadCornerSubpix(gray, corner_points_buf, pattern_size);
            right_corner_points_of_all_imgs.push_back(corner_points_buf);
        }
    }

    std::vector<std::vector<cv::Point3f>> objectPoints;
    int i,j,k;
    for(k=0;k<image_num;k++)
    {
        std::vector<cv::Point3f> tempCornerPoints;
        for(i=0; i<pattern_size.height;i++)
        {
            for(j=0;j<pattern_size.width;j++)
            {
                cv::Point3f singleRealPoint;
                singleRealPoint.x = j * corner_side;
                singleRealPoint.y = i * corner_side;
                singleRealPoint.z = 0;
                tempCornerPoints.push_back(singleRealPoint);
            }
        }
        objectPoints.push_back(tempCornerPoints);
    }

    cv::Mat R, T, E, F;
    double rms = cv::stereoCalibrate(objectPoints, left_corner_points_of_all_imgs, right_corner_points_of_all_imgs,
                                    left_camera_matrix, left_distcoeff_matrix,
                                    right_camera_matrix, right_distcoeff_matrix,
                                    image_size, R, T, E, F, CV_CALIB_FIX_INTRINSIC);
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(left_camera_matrix, left_distcoeff_matrix,
                      right_camera_matrix, right_distcoeff_matrix,
                      image_size, R, T,
                      R1, R2, P1, P2, Q,0,0, image_size);
    QDomDocument doc;
    QDomElement inner_matrix=doc.createElement("stereoCalibration");
    doc.appendChild(inner_matrix);
    QDomElement left_matrix = doc.createElement("left_matrix");
    left_matrix.setAttribute("fx",left_camera_matrix.at<double>(0,0));
    left_matrix.setAttribute("fy",left_camera_matrix.at<double>(1,1));
    left_matrix.setAttribute("cx",left_camera_matrix.at<double>(0,2));
    left_matrix.setAttribute("cy",left_camera_matrix.at<double>(1,2));
    inner_matrix.appendChild(left_matrix);
    QDomElement left_distcoeff = doc.createElement("left_distcoeff");
    left_distcoeff.setAttribute("k1",left_distcoeff_matrix.at<double>(0,0));
    left_distcoeff.setAttribute("k2",left_distcoeff_matrix.at<double>(0,1));
    left_distcoeff.setAttribute("p1",left_distcoeff_matrix.at<double>(0,2));
    left_distcoeff.setAttribute("p2",left_distcoeff_matrix.at<double>(0,3));
    left_distcoeff.setAttribute("k3",left_distcoeff_matrix.at<double>(0,4));
    inner_matrix.appendChild(left_distcoeff);
    QDomElement right_matrix = doc.createElement("right_matrix");
    right_matrix.setAttribute("fx",right_camera_matrix.at<double>(0,0));
    right_matrix.setAttribute("fy",right_camera_matrix.at<double>(1,1));
    right_matrix.setAttribute("cx",right_camera_matrix.at<double>(0,2));
    right_matrix.setAttribute("cy",right_camera_matrix.at<double>(1,2));
    inner_matrix.appendChild(right_matrix);
    QDomElement right_distcoeff = doc.createElement("right_distcoeff");
    right_distcoeff.setAttribute("k1",right_distcoeff_matrix.at<double>(0,0));
    right_distcoeff.setAttribute("k2",right_distcoeff_matrix.at<double>(0,1));
    right_distcoeff.setAttribute("p1",right_distcoeff_matrix.at<double>(0,2));
    right_distcoeff.setAttribute("p2",right_distcoeff_matrix.at<double>(0,3));
    right_distcoeff.setAttribute("k3",right_distcoeff_matrix.at<double>(0,4));
    inner_matrix.appendChild(right_distcoeff);
    QDomElement R1_ = doc.createElement("R1");
    R1_.setAttribute("a11",R1.at<double>(0,0));
    R1_.setAttribute("a12",R1.at<double>(0,1));
    R1_.setAttribute("a13",R1.at<double>(0,2));
    R1_.setAttribute("a21",R1.at<double>(1,0));
    R1_.setAttribute("a22",R1.at<double>(1,1));
    R1_.setAttribute("a23",R1.at<double>(1,2));
    R1_.setAttribute("a31",R1.at<double>(2,0));
    R1_.setAttribute("a32",R1.at<double>(2,1));
    R1_.setAttribute("a33",R1.at<double>(2,2));
    inner_matrix.appendChild(R1_);
    QDomElement P1_ = doc.createElement("P1");
    P1_.setAttribute("a11",P1.at<double>(0,0));
    P1_.setAttribute("a12",P1.at<double>(0,1));
    P1_.setAttribute("a13",P1.at<double>(0,2));
    P1_.setAttribute("a21",P1.at<double>(1,0));
    P1_.setAttribute("a22",P1.at<double>(1,1));
    P1_.setAttribute("a23",P1.at<double>(1,2));
    P1_.setAttribute("a31",P1.at<double>(2,0));
    P1_.setAttribute("a32",P1.at<double>(2,1));
    P1_.setAttribute("a33",P1.at<double>(2,2));
    inner_matrix.appendChild(P1_);
    QDomElement R2_ = doc.createElement("R2");
    R2_.setAttribute("a11",R2.at<double>(0,0));
    R2_.setAttribute("a12",R2.at<double>(0,1));
    R2_.setAttribute("a13",R2.at<double>(0,2));
    R2_.setAttribute("a21",R2.at<double>(1,0));
    R2_.setAttribute("a22",R2.at<double>(1,1));
    R2_.setAttribute("a23",R2.at<double>(1,2));
    R2_.setAttribute("a31",R2.at<double>(2,0));
    R2_.setAttribute("a32",R2.at<double>(2,1));
    R2_.setAttribute("a33",R2.at<double>(2,2));
    inner_matrix.appendChild(R2_);
    QDomElement P2_ = doc.createElement("P2");
    P2_.setAttribute("a11",P2.at<double>(0,0));
    P2_.setAttribute("a12",P2.at<double>(0,1));
    P2_.setAttribute("a13",P2.at<double>(0,2));
    P2_.setAttribute("a21",P2.at<double>(1,0));
    P2_.setAttribute("a22",P2.at<double>(1,1));
    P2_.setAttribute("a23",P2.at<double>(1,2));
    P2_.setAttribute("a31",P2.at<double>(2,0));
    P2_.setAttribute("a32",P2.at<double>(2,1));
    P2_.setAttribute("a33",P2.at<double>(2,2));
    inner_matrix.appendChild(P2_);
    QDomElement Q_ = doc.createElement("Q");
    Q_.setAttribute("a11",Q.at<double>(0,0));
    Q_.setAttribute("a12",Q.at<double>(0,1));
    Q_.setAttribute("a13",Q.at<double>(0,2));
    Q_.setAttribute("a14",Q.at<double>(0,3));
    Q_.setAttribute("a21",Q.at<double>(1,0));
    Q_.setAttribute("a22",Q.at<double>(1,1));
    Q_.setAttribute("a23",Q.at<double>(1,2));
    Q_.setAttribute("a24",Q.at<double>(1,3));
    Q_.setAttribute("a31",Q.at<double>(2,0));
    Q_.setAttribute("a32",Q.at<double>(2,1));
    Q_.setAttribute("a33",Q.at<double>(2,2));
    Q_.setAttribute("a34",Q.at<double>(2,3));
    Q_.setAttribute("a41",Q.at<double>(3,0));
    Q_.setAttribute("a42",Q.at<double>(3,1));
    Q_.setAttribute("a43",Q.at<double>(3,2));
    Q_.setAttribute("a44",Q.at<double>(3,3));
    inner_matrix.appendChild(Q_);
    QDomElement size = doc.createElement("img_size");
    size.setAttribute("width", image_size.width);
    size.setAttribute("height", image_size.height);
    inner_matrix.appendChild(size);
    QDomElement error = doc.createElement("error");
    error.setAttribute("e", rms);
    inner_matrix.appendChild(error);

    QFile file(left_path+"/../stereoCalibration.xml");
    file.open(QFile::WriteOnly|QFile::Truncate);
    QTextStream out_stream(&file);
    doc.save(out_stream,4); //缩进4格
    file.close();

    cv::Mat mapx, mapy;
    cv::initUndistortRectifyMap(left_camera_matrix, left_distcoeff_matrix, R1, P1, image_size, CV_16SC2, mapx, mapy);
    filename = left_path + "/" + imagesList[0];
    cv::Mat img = cv::imread(filename.toStdString());
    cv::remap(img, img, mapx, mapy, cv::INTER_LINEAR);
    camera_1->showImg(img);

    cv::initUndistortRectifyMap(right_camera_matrix, right_distcoeff_matrix, R2, P2, image_size, CV_16SC2, mapx, mapy);
    filename = right_path + "/" + imagesList[0];
    img = cv::imread(filename.toStdString());
    cv::remap(img, img, mapx, mapy, cv::INTER_LINEAR);
    camera_2->showImg(img);

    ui->statusBar->showMessage(tr("Done!"), 1000);
}

void MainWindow::widthChange(int width)
{
    corner_width = width;
    camera_1->setWidth(width);
    camera_2->setWidth(width);
}

void MainWindow::heightChange(int height)
{
    corner_height = height;
    camera_1->setHeight(height);
    camera_2->setHeight(height);
}

void MainWindow::sideChange(double side)
{
    corner_side = side;
    camera_1->setSideLen(side);
    camera_2->setSideLen(side);
}

StereoRectify::StereoRectify(QPushButton* start,
                             QPushButton* stop,
                             QPushButton* dir,
                             QLineEdit* text,
                             QComboBox* left,
                             QComboBox* right,
                             QLabel* left_,
                             QLabel* right_,
                             QLabel* disparity,
                             QLabel* depth)
{
    start_button = start;
    stop_button = stop;
    dir_choose = dir;
    dir_text = text;
    left_choose = left;
    right_choose = right;
    left_img = left_;
    right_img = right_;
    disparity_img = disparity;
    depth_img = depth;

    connect(start_button, SIGNAL(clicked()), this, SLOT(start()));
    connect(stop_button, SIGNAL(clicked()), this, SLOT(stop()));
    connect(left_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(choose_camera_left(int)));
    connect(right_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(choose_camera_right(int)));
    connect(dir_choose, SIGNAL(clicked()), this, SLOT(choose_dir()));

    timer = new QTimer(this);

    sgbm = cv::StereoSGBM::create(0, 16, 3);
    sgbm->setPreFilterCap(63);
    int sgbmWinSize = 1;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = 3;
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    int numberOfDisparities = ((480 / 8) + 15) & -16;
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
}

StereoRectify::~StereoRectify()
{
    delete timer;
}

void StereoRectify::choose_dir()
{
    QString filename = QFileDialog::getOpenFileName(this, "选择标定文件", "./", "*.xml");
    dir_text->setText(filename);
    ReadXml(filename);
}

void StereoRectify::ReadXml(QString path)
{
    QFile file(path); //相对路径、绝对路径、资源路径都行
    if(!file.open(QFile::ReadOnly))
        return;
    QDomDocument doc;
    QString err;
    if(!doc.setContent(&file, true, &err))
    {
        file.close();
        std::cout << err.toStdString() << std::endl;
        return;
    }
    file.close();
    QDomNodeList list = doc.elementsByTagName("left_matrix");
    QDomNode node = list.at(0);
    QDomElement e = node.toElement();
    cv::Mat left_matrix = (cv::Mat_<double>(3,3)<<e.attribute("fx").toDouble(),0,e.attribute("cx").toDouble(),
                           0,e.attribute("fy").toDouble(),e.attribute("cy").toDouble(),0,0,1);
    list = doc.elementsByTagName("left_distcoeff");
    node = list.at(0);
    e = node.toElement();
    cv::Mat left_distcoeff = (cv::Mat_<double>(1,5)<<e.attribute("k1").toDouble(),e.attribute("k2").toDouble(),e.attribute("p1").toDouble(),
                              e.attribute("p2").toDouble(),e.attribute("k3").toDouble());
    list = doc.elementsByTagName("right_matrix");
    node = list.at(0);
    e = node.toElement();
    cv::Mat right_matrix = (cv::Mat_<double>(3,3)<<e.attribute("fx").toDouble(),0,e.attribute("cx").toDouble(),
                           0,e.attribute("fy").toDouble(),e.attribute("cy").toDouble(),0,0,1);
    list = doc.elementsByTagName("right_distcoeff");
    node = list.at(0);
    e = node.toElement();
    cv::Mat right_distcoeff = (cv::Mat_<double>(1,5)<<e.attribute("k1").toDouble(),e.attribute("k2").toDouble(),e.attribute("p1").toDouble(),
                              e.attribute("p2").toDouble(),e.attribute("k3").toDouble());
    list = doc.elementsByTagName("R1");
    node = list.at(0);
    e = node.toElement();
    cv::Mat R1 = (cv::Mat_<double>(3,3)<<e.attribute("a11").toDouble(),e.attribute("a12").toDouble(),e.attribute("a13").toDouble(),
                  e.attribute("a21").toDouble(),e.attribute("a22").toDouble(),e.attribute("a23").toDouble(),
                  e.attribute("a31").toDouble(),e.attribute("a32").toDouble(),e.attribute("a33").toDouble());
    list = doc.elementsByTagName("R2");
    node = list.at(0);
    e = node.toElement();
    cv::Mat R2 = (cv::Mat_<double>(3,3)<<e.attribute("a11").toDouble(),e.attribute("a12").toDouble(),e.attribute("a13").toDouble(),
                  e.attribute("a21").toDouble(),e.attribute("a22").toDouble(),e.attribute("a23").toDouble(),
                  e.attribute("a31").toDouble(),e.attribute("a32").toDouble(),e.attribute("a33").toDouble());
    list = doc.elementsByTagName("P1");
    node = list.at(0);
    e = node.toElement();
    cv::Mat P1 = (cv::Mat_<double>(3,3)<<e.attribute("a11").toDouble(),e.attribute("a12").toDouble(),e.attribute("a13").toDouble(),
                  e.attribute("a21").toDouble(),e.attribute("a22").toDouble(),e.attribute("a23").toDouble(),
                  e.attribute("a31").toDouble(),e.attribute("a32").toDouble(),e.attribute("a33").toDouble());
    list = doc.elementsByTagName("P2");
    node = list.at(0);
    e = node.toElement();
    cv::Mat P2 = (cv::Mat_<double>(3,3)<<e.attribute("a11").toDouble(),e.attribute("a12").toDouble(),e.attribute("a13").toDouble(),
                  e.attribute("a21").toDouble(),e.attribute("a22").toDouble(),e.attribute("a23").toDouble(),
                  e.attribute("a31").toDouble(),e.attribute("a32").toDouble(),e.attribute("a33").toDouble());
    list = doc.elementsByTagName("Q");
    node = list.at(0);
    e = node.toElement();
    Q = (cv::Mat_<double>(4,4)<<e.attribute("a11").toDouble(),e.attribute("a12").toDouble(),e.attribute("a13").toDouble(),e.attribute("a14").toDouble(),
         e.attribute("a21").toDouble(),e.attribute("a22").toDouble(),e.attribute("a23").toDouble(),e.attribute("a24").toDouble(),
         e.attribute("a31").toDouble(),e.attribute("a32").toDouble(),e.attribute("a33").toDouble(),e.attribute("a34").toDouble(),
         e.attribute("a41").toDouble(),e.attribute("a42").toDouble(),e.attribute("a43").toDouble(),e.attribute("a44").toDouble());
    list = doc.elementsByTagName("img_size");
    node = list.at(0);
    e = node.toElement();
    cv::Size image_size;
    image_size.width = e.attribute("width").toInt();
    image_size.height = e.attribute("height").toInt();
    cv::initUndistortRectifyMap(left_matrix, left_distcoeff, R1, P1, image_size, CV_16SC2, left_mapx, left_mapy);
    cv::initUndistortRectifyMap(right_matrix, right_distcoeff, R2, P2, image_size, CV_16SC2, right_mapx, right_mapy);
//    cv::Mat img_1 = cv::imread("C:/Users/uncle/Desktop/left.jpg");
//    cv::Mat img_2 = cv::imread("C:/Users/uncle/Desktop/right.jpg");
//    showImg(left_img, img_1);
//    showImg(right_img, img_2);
//    cv::cvtColor(img_1, img_1, CV_BGR2GRAY);
//    cv::cvtColor(img_2, img_2, CV_BGR2GRAY);

//    disparity_image = cv::Mat::zeros(img_1.size(), CV_8UC1);
//    depth_image = cv::Mat::zeros(img_1.size(), CV_8UC1);
//    cv::Mat disp = cv::Mat::zeros(img_1.size(), CV_8UC1);

//    int dim[3] = {img_1.size().width, img_1.size().height, img_1.size().width};
//    float* D1_data = (float*)malloc(img_1.size().width*img_1.size().height*sizeof(float));
//    float* D2_data = (float*)malloc(img_1.size().width*img_1.size().height*sizeof(float));
//    // 参数设置
//    Elas::parameters param;
//    param.postprocess_only_left = false;
//    Elas elas(param);
//    elas.process(img_1.data, img_2.data, D1_data, D2_data, dim);
//    // find maximum disparity for scaling output disparity images to [0..255]
//    float disp_max = 0;
//    float disp_min = img_1.cols;
//    for (int32_t i=0; i<img_1.size().width*img_1.size().height; i++) {
//        if (D1_data[i]>disp_max) disp_max = D1_data[i];
//        if (D1_data[i]>0 && D1_data[i]<disp_min) disp_min = D1_data[i];
//    }
//    float depth_max = Q.at<double>(2,3)/(Q.at<double>(3,2)*disp_min+Q.at<double>(3,3));
//    for (int i = 0; i < img_1.size().width; i++) {
//        for (int j = 0; j < img_1.size().height; j++) {
//            disp.at<uchar>(j,i) = (uint8_t)std::max(255.0*D1_data[i+j*img_1.size().width]/disp_max,0.0);
//            disparity_image.at<uchar>(j, i) = D1_data[i+j*img_1.size().width];
//            double distance = Q.at<double>(2,3)/(Q.at<double>(3,2)*D1_data[i+j*img_1.cols]+Q.at<double>(3,3));
//            depth_image.at<uchar>(j, i) = (uint8_t)std::max(255.0*distance/depth_max,0.0);
//        }
//    }
//    // free memory
//    free(D1_data);
//    free(D2_data);
//    showImg(disparity_img, disp);
//    showImg(depth_img, depth_image);
}

QImage StereoRectify::Mat2QImage(cv::Mat cvImg)
{
    QImage qImg;
    if(cvImg.channels()==3)                             //3 channels color image
    {
        cv::cvtColor(cvImg,cvImg,CV_BGR2RGB);
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols, cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_RGB888);
    }
    else if(cvImg.channels()==1)                    //grayscale image
    {
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols,cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_Indexed8);
    }
    else
    {
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols,cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_RGB888);
    }
    return qImg;
}

void StereoRectify::start()
{
    if(left_capture.isOpened())
        left_capture.release();
    left_capture.open(current_left_camera);
    if(right_capture.isOpened())
        right_capture.release();
    right_capture.open(current_right_camera);
    if(left_capture.isOpened() && right_capture.isOpened())
    {
        double rate = 5;
        timer->setInterval(1000/rate);
        connect(timer, SIGNAL(timeout()), this, SLOT(nextFrame()));
        timer->start();
    }
}

void StereoRectify::stop()
{
    if(left_capture.isOpened())
        left_capture.release();
    if(right_capture.isOpened())
        right_capture.release();
    timer->stop();
}

void StereoRectify::choose_camera_left(int id)
{
    current_left_camera = id;
}

void StereoRectify::choose_camera_right(int id)
{
    current_right_camera = id;
}

void StereoRectify::setCameraNum(int num)
{
    disconnect(left_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(choose_camera_left(int)));
    disconnect(right_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(choose_camera_right(int)));
    left_choose->clear();
    right_choose->clear();
    for(int i = 0; i < num; i++)
    {
        left_choose->addItem(QString::asprintf("%d",i));
        right_choose->addItem(QString::asprintf("%d",i));
    }
    connect(left_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(choose_camera_left(int)));
    connect(right_choose, SIGNAL(currentIndexChanged(int)), this, SLOT(choose_camera_right(int)));
    current_left_camera = 0;
    current_right_camera = 0;
}

void StereoRectify::showImg(QLabel* img_view, cv::Mat img)
{
    QImage image = Mat2QImage(img);
    img_view->setPixmap(QPixmap::fromImage(image));
}

void StereoRectify::nextFrame()
{
    left_capture >> left_image;
    if(!left_image.empty())
    {
        cv::remap(left_image, left_image, left_mapx, left_mapy, cv::INTER_LINEAR);
        showImg(left_img, left_image);
    }

    right_capture >> right_image;
    if(!right_image.empty())
    {
        cv::remap(right_image, right_image, right_mapx, right_mapy, cv::INTER_LINEAR);
        showImg(right_img, right_image);
    }

//    sgbm->compute(left_image, right_image, disparity_image);
    cv::cvtColor(left_image, left_image, CV_BGR2GRAY);
    cv::cvtColor(right_image, right_image, CV_BGR2GRAY);

    disparity_image = cv::Mat::zeros(left_image.size(), CV_32FC1);
    depth_image = cv::Mat::zeros(left_image.size(), CV_32FC1);
    cv::Mat disp = cv::Mat::zeros(left_image.size(), CV_8UC1);
    cv::Mat depth = cv::Mat::zeros(left_image.size(), CV_8UC1);

    int dim[3] = {left_image.cols, left_image.rows, left_image.cols};
    float* D1_data = (float*)malloc(left_image.cols*left_image.rows*sizeof(float));
    float* D2_data = (float*)malloc(left_image.cols*left_image.rows*sizeof(float));
    // 参数设置
    Elas::parameters param;
    param.postprocess_only_left = false;
    Elas elas(param);
    elas.process(left_image.data, right_image.data, D1_data, D2_data, dim);
    // find maximum disparity for scaling output disparity images to [0..255]
    float disp_max = 0;
    float disp_min = left_image.cols;
    for (int32_t i=0; i<left_image.cols*left_image.rows; i++) {
        if (D1_data[i]>disp_max) disp_max = D1_data[i];
        if (D1_data[i]>0 && D1_data[i]<disp_min) disp_min = D1_data[i];
    }
    float depth_max = Q.at<double>(2,3)/(Q.at<double>(3,2)*disp_min+Q.at<double>(3,3));
    for (int i = 0; i < left_image.cols; i++) {
        for (int j = 0; j < left_image.rows; j++) {
            disp.at<uchar>(j,i) = (uint8_t)std::max(255.0*D1_data[i+j*left_image.cols]/disp_max,0.0);
            disparity_image.at<double>(j, i) = D1_data[i+j*left_image.cols];
            depth_image.at<double>(j, i) = std::max(Q.at<double>(2,3)/(Q.at<double>(3,2)*D1_data[i+j*left_image.cols]+Q.at<double>(3,3)),0.0);
            depth.at<uchar>(j, i) = (uint8_t)std::max(255.0*depth_image.at<double>(j, i)/depth_max,0.0);
        }
    }
    // free memory
    free(D1_data);
    free(D2_data);
    showImg(disparity_img, disp);
    showImg(depth_img, depth);
}
