#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <iostream>
#include <QtXml>
#include <fstream>

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
            cv::Size image_size;
            image_size.width = img.cols;
            image_size.height = img.rows;
            cv::Mat new_cameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffMatrix, image_size, 0, image_size, 0);
            cv::initUndistortRectifyMap(cameraMatrix, distCoeffMatrix, cv::Mat::eye(3, 3, CV_32FC1), new_cameraMatrix, image_size, CV_32FC1, mapx, mapy);
            cv::remap(img, img, mapx, mapy, cv::INTER_LINEAR);
        }

        image = Mat2QImage(img);
        ImageView->setPixmap(QPixmap::fromImage(image));
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

    cv::Mat new_cameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoefficients, image_size, 0, image_size, 0);

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

    cv::initUndistortRectifyMap(cameraMatrix, distCoefficients, cv::Mat::eye(3, 3, CV_32FC1), new_cameraMatrix, image_size, CV_32FC1, mapx, mapy);
    filename = srcDirPath + "/" + imagesList[0];
    img = cv::imread(filename.toStdString());
    cv::remap(img, img, mapx, mapy, cv::INTER_LINEAR);
    image = Mat2QImage(img);
    ImageView->setPixmap(QPixmap::fromImage(image));
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

void Camera::setSideLen(int side)
{
    corner_side = side;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->ImageView_1->setScaledContents(true);
    ui->ImageView_2->setScaledContents(true);
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
    connect(ui->side_len, SIGNAL(valueChanged(int)), this, SLOT(sideChange(int)));
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
    // 左侧相机
    std::vector<cv::Point2f> corner_points_buf;//建一个数组缓存检测到的角点
    std::vector<std::vector<cv::Point2f>> left_corner_points_of_all_imgs;
    int image_num = 0;
    QString filename;
    while(image_num < imagesList.length())
    {
        filename = left_path + "/" + imagesList[image_num];
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
            left_corner_points_of_all_imgs.push_back(corner_points_buf);
        }
    }
    // 右侧相机
    std::vector<std::vector<cv::Point2f>> right_corner_points_of_all_imgs;
    int right_image_num = 0;
    dir.cd(right_path);
    dir.setFilter(QDir::Files | QDir::NoSymLinks);
    dir.setNameFilters(filters);
    imagesList = dir.entryList();
    while(right_image_num < image_num)
    {
        filename = right_path + "/" + imagesList[right_image_num];
        right_image_num++;
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
                                    image_size, R, T, E, F);
    cv::Mat R1, R2, P1, P2, Q;
    // R1-输出矩阵，第一个摄像机的校正变换矩阵（旋转变换）
    // R2-输出矩阵，第二个摄像机的校正变换矩阵（旋转矩阵）
    // P1-输出矩阵，第一个摄像机在新坐标系下的投影矩阵
    // P2-输出矩阵，第二个摄像机在想坐标系下的投影矩阵
    // Q-4*4的深度差异映射矩阵
    cv::stereoRectify(left_camera_matrix, left_distcoeff_matrix,
                      right_camera_matrix, right_distcoeff_matrix,
                      image_size, R, T,
                      R1, R2, P1, P2, Q);
    std::ofstream file(left_path.toStdString()+"/../result.txt", std::ios_base::out);
    file << "R:" << std::endl << R << std::endl << std::endl;
    file << "T:" << std::endl << T << std::endl << std::endl;
    file << "E:" << std::endl << E << std::endl << std::endl;
    file << "F:" << std::endl << F << std::endl << std::endl;
    file << "R1:" << std::endl << R1 << std::endl << std::endl;
    file << "R2:" << std::endl << R2 << std::endl << std::endl;
    file << "P1:" << std::endl << P1 << std::endl << std::endl;
    file << "P2:" << std::endl << P2 << std::endl << std::endl;
    file << "Q:" << std::endl << Q << std::endl << std::endl;
    file << "Error:" << rms << std::endl;
    file.close();

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

void MainWindow::sideChange(int side)
{
    corner_side = side;
    camera_1->setSideLen(side);
    camera_2->setSideLen(side);
}
