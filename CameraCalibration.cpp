#include "CameraCalibration.h"
#include "ui_CameraCalibration.h"

cv::Mat find_corner_thread_img;
struct Chessboarder_t find_corner_thread_chessboard;

CameraCalibration::CameraCalibration(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::CameraCalibration)
{
    ui->setupUi(this);
    setMinimumSize(this->width(), this->height());
    ui->addImage->setFlat(true);
    ui->calibrate->setFlat(true);
    ui->export_1->setFlat(true);
    ui->delete_1->setFlat(true);
    ui->undistort->setFlat(true);
    findcorner_thread = new FindcornerThread();
    connect(findcorner_thread, SIGNAL(isDone()), this, SLOT(DealThreadDone()));
    ShowIntro();
    QFont font = ui->menu->font();
    font.setPixelSize(12);
    ui->listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection); // 按ctrl多选
    connect(ui->addImage, SIGNAL(clicked()), this, SLOT(addImage()));
    connect(ui->delete_1, SIGNAL(clicked()), this, SLOT(deleteImage()));
    connect(ui->calibrate, SIGNAL(clicked()), this, SLOT(calibrate()));
    connect(ui->fisheye, SIGNAL(stateChanged(int)), this, SLOT(fisheyeModeSwitch(int)));
    connect(ui->export_1, SIGNAL(clicked()), this, SLOT(exportParam()));
    connect(ui->undistort, SIGNAL(clicked()), this, SLOT(distortModeSwitch()));
    connect(ui->listWidget, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), this, SLOT(chooseImage(QListWidgetItem*, QListWidgetItem*)));
    connect(ui->double_camera, SIGNAL(toggled(bool)), this, SLOT(reset()));
    connect(ui->single_camera, SIGNAL(toggled(bool)), this, SLOT(reset()));
    connect(ui->double_undistort, SIGNAL(toggled(bool)), this, SLOT(reset()));
    connect(ui->single_undistort, SIGNAL(toggled(bool)), this, SLOT(reset()));
    saveImage = ui->menu->addAction("导出矫正图片");
    font = saveImage->font();
    font.setPixelSize(12);
    saveImage->setFont(font);
    about = ui->menu->addAction("关于");
    about->setFont(font);
    connect(saveImage, SIGNAL(triggered()), this, SLOT(saveUndistort()));
    connect(about, SIGNAL(triggered()), this, SLOT(showIntro()));
}

CameraCalibration::~CameraCalibration()
{
    delete ui;
    deleteLater();
}

void CameraCalibration::resizeEvent(QResizeEvent* event)
{
    QSize window_size = event->size();
    int left_list_width = window_size.width()*5/16;
    int left_list_height = window_size.height()-80;
    if(left_list_width > 500) left_list_width = 500;
    ui->scrollArea->setGeometry(0,80,left_list_width, left_list_height);
    ui->scrollAreaWidgetContents->setGeometry(0,0,left_list_width-2, left_list_height-2);
    ui->intro->setGeometry(0,0,left_list_width-2, 100);
    ui->listWidget->setGeometry(0,0,left_list_width-2, left_list_height-2);

    ui->Image->setGeometry(left_list_width,80,window_size.width()-left_list_width, left_list_height);
}

void CameraCalibration::showIntro()
{
    a = new AboutUs(this);
    QFont font = a->font();
    font.setPixelSize(12);
    a->setFont(font);
    a->setWindowTitle("关于TStoneCalibration");
    a->show();
}

// 显示简单的文字引导
void CameraCalibration::ShowIntro()
{
    ui->intro->setFixedHeight(100);
    if(ui->double_camera->isChecked() || ui->double_undistort->isChecked())
    {
        QString str = "点击左上角添加图片";
        str += "\n";
        str += "左右相机图片分别放于两个文件夹";
        str += "\n";
        str += "对应图片名称需一致。";
        str += "\n";
        str += "\n";
        str += "单/双目纠正模式可纠正无棋盘的图片。";
        ui->intro->setText(str);
    }
    else if(ui->single_camera->isChecked() || ui->single_undistort->isChecked())
    {
        QString str = "点击左上角添加图片";
        str += "\n";
        str += "15至20张较为适宜";
        str += "\n";
        str += "\n";
        str += "单/双目纠正模式可纠正无棋盘的图片。";
        ui->intro->setText(str);
    }
}

// 隐藏文字指导
void CameraCalibration::HiddenIntro()
{
    ui->intro->setText("");
    ui->intro->setFixedHeight(0);
}

// 导出矫正图片
void CameraCalibration::saveUndistort()
{
    if((!mono_calib.hasParam() && ui->single_undistort->isChecked()) || 
        (!bino_calib.hasParam() && ui->double_undistort->isChecked()))
    {
        QMessageBox::warning(this, "警告", "还未获取到相机参数，请点击UNDISTORT按钮加载yaml文件。", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    QString srcDirPath = QFileDialog::getExistingDirectory(this, "选择保存路径", "./");
    if(srcDirPath.length() <= 0)
        return;
    QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
    if(ui->double_camera->isChecked() || ui->double_undistort->isChecked())
    {
        QDir dir;
        dir.mkdir(srcDirPath+"/left/");
        dir.mkdir(srcDirPath+"/right/");
        for(unsigned int i = 0; i < stereo_imgs.size(); i++)
        {
            struct Stereo_Img_t img = stereo_imgs[i];
            cv::Mat left_dst = cv::imread(img.left_path);
            cv::Mat right_dst = cv::imread(img.right_path);
            bino_calib.undistort(left_dst, right_dst);
            QString save_name = srcDirPath + "/left/" + img.left_file_name;
            std::string str = code->fromUnicode(save_name).data();
            cv::imwrite(str, left_dst);
            save_name = srcDirPath + "/right/" + img.left_file_name;
            str = code->fromUnicode(save_name).data();
            cv::imwrite(str, right_dst);
            cv::Mat combian_img;
            cv::hconcat(left_dst, right_dst, combian_img);
            for(int i = 1; i < 10; i++)
            {
                cv::line(combian_img, cv::Point(0, combian_img.rows*i/10), cv::Point(combian_img.cols-1, combian_img.rows*i/10), cv::Scalar(0,0,255), 2);
            }
            save_name = srcDirPath + "/" + img.left_file_name;
            str = code->fromUnicode(save_name).data();
            cv::imwrite(str, combian_img);
        }
    }
    else
    {
        for(unsigned int i = 0; i < imgs.size(); i++)
        {
            cv::Mat dst = cv::imread(imgs[i].file_path);
            mono_calib.undistort(dst);
            QString save_name = srcDirPath + "/" + imgs[i].file_name;
            std::string str = code->fromUnicode(save_name).data();
            cv::imwrite(str, dst);
        }
    }
}

// 切换模式时删除所有图片缓存
void CameraCalibration::reset()
{
    ShowIntro();
    ui->k_2->setEnabled(true);
    ui->k_3->setEnabled(true);
    ui->tangential->setEnabled(true);
    ui->calibrate->setEnabled(true);
    ui->export_1->setEnabled(true);
    bino_calib.reset();
    mono_calib.reset();
    distort_flag = false;
    disconnect(ui->listWidget, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), this, SLOT(chooseImage(QListWidgetItem*, QListWidgetItem*)));
    ui->listWidget->clear();
    connect(ui->listWidget, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), this, SLOT(chooseImage(QListWidgetItem*, QListWidgetItem*)));
    if(imgs.size() > 0)
        imgs.clear();
    if(stereo_imgs.size() > 0)
        stereo_imgs.clear();
    list_select_ = false;
}

// 显示选择的图像
void CameraCalibration::chooseImage(QListWidgetItem* item, QListWidgetItem*)
{
    list_select_ = true;
    int id = ui->listWidget->row(item);
    if(ui->double_camera->isChecked())
    {
        // 如果是双目模式
        struct Stereo_Img_t img = stereo_imgs[id];
        cv::Mat left_dst = cv::imread(img.left_path);
        cv::Mat right_dst = cv::imread(img.right_path);
        if(distort_flag == true && bino_calib.hasParam())
        {
            // 对左右图像进行矫正
            bino_calib.undistort(left_dst, right_dst);
            // 绘制横线以观察左右图像极线是否对齐
            for(int i = 1; i < 10; i++)
            {
                cv::line(left_dst, cv::Point(0, left_dst.rows*i/10), cv::Point(left_dst.cols-1, left_dst.rows*i/10), cv::Scalar(0,0,255), 2);
                cv::line(right_dst, cv::Point(0, right_dst.rows*i/10), cv::Point(right_dst.cols-1, right_dst.rows*i/10), cv::Scalar(0,0,255), 2);
            }
        }
        else
        {
            bino_calib.showCorners(left_dst, right_dst, img);
        }
        cv::Mat combian_img;
        cv::hconcat(left_dst, right_dst, combian_img);
        QImage qimage = Mat2QImage(combian_img);
        ui->Image->SetImage(qimage);
        ui->Image->repaint();
    }
    else if(ui->single_camera->isChecked())
    {
        // 单目逻辑同双目
        cv::Mat img;
        std::vector<cv::Point2f> corners;
        std::vector<cv::Point3f> w_p;
        cv::Mat rvecs, tvecs;
        cv::Vec3d fish_rvecs, fish_tvecs;
        img = cv::imread(imgs[id].file_path);
        
        if(distort_flag == true && mono_calib.hasParam())
        {
            mono_calib.undistort(img);
        }
        else
        {
            mono_calib.showCorners(img, imgs[id]);
        }
        QImage qimage = Mat2QImage(img);
        ui->Image->SetImage(qimage);
        ui->Image->repaint();
    }
    else if(ui->single_undistort->isChecked())
    {
        // 单目纠正
        cv::Mat img = cv::imread(imgs[id].file_path);
        if(distort_flag == true && mono_calib.hasParam())
        {
            mono_calib.undistort(img);
        }
        QImage qimage = Mat2QImage(img);
        ui->Image->SetImage(qimage);
        ui->Image->repaint();
    }
    else
    {
        // 双目纠正
        struct Stereo_Img_t img = stereo_imgs[id];
        cv::Mat left_dst = cv::imread(img.left_path);
        cv::Mat right_dst = cv::imread(img.right_path);
        if(distort_flag == true && bino_calib.hasParam())
        {
            bino_calib.undistort(left_dst, right_dst);
            for(int i = 1; i < 10; i++)
            {
                cv::line(left_dst, cv::Point(0, left_dst.rows*i/10), cv::Point(left_dst.cols-1, left_dst.rows*i/10), cv::Scalar(0,0,255), 2);
                cv::line(right_dst, cv::Point(0, right_dst.rows*i/10), cv::Point(right_dst.cols-1, right_dst.rows*i/10), cv::Scalar(0,0,255), 2);
            }
        }
        cv::Mat combian_img;
        cv::hconcat(left_dst, right_dst, combian_img);
        QImage qimage = Mat2QImage(combian_img);
        ui->Image->SetImage(qimage);
        ui->Image->repaint();
    }
}

// 角点寻找线程结束时的回调函数
void CameraCalibration::DealThreadDone()
{
    findcorner_thread->quit();
    findcorner_thread->wait();
    thread_done = true;
}

void CameraCalibration::receiveYamlPath(QString str)
{
    cv::FileStorage fs_read(str.toStdString(), cv::FileStorage::READ);
    if(ui->single_undistort->isChecked())
    {
        if(fs_read["cameraMatrix"].empty() || fs_read["distCoeffs"].empty())
        {
            QMessageBox::critical(this, "错误", "YAML文件格式错误", QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
        cv::Mat cameraMatrix, distCoeffs;
        fs_read["cameraMatrix"] >> cameraMatrix;
        fs_read["distCoeffs"] >> distCoeffs;
        mono_calib.setCameraParam(cameraMatrix, distCoeffs, fisheye_flag);
    }
    else
    {
        if(fs_read["left_camera_Matrix"].empty() || fs_read["left_camera_distCoeffs"].empty() || fs_read["right_camera_Matrix"].empty() || fs_read["right_camera_distCoeffs"].empty())
        {
            QMessageBox::critical(this, "错误", "YAML文件格式错误", QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
        if(fs_read["Rotate_Matrix"].empty() || fs_read["Translate_Matrix"].empty() || fs_read["R1"].empty() || fs_read["R2"].empty() || fs_read["P1"].empty() ||
                fs_read["P2"].empty() || fs_read["Q"].empty())
        {
            QMessageBox::critical(this, "错误", "YAML文件格式错误", QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
        cv::Mat left_in, right_in, left_d, right_d, r, t, r1, r2, p1, p2, q;
        fs_read["left_camera_Matrix"] >> left_in;
        fs_read["right_camera_Matrix"] >> right_in;
        fs_read["left_camera_distCoeffs"] >> left_d;
        fs_read["right_camera_distCoeffs"] >> right_d;
        fs_read["Rotate_Matrix"] >> r;
        fs_read["Translate_Matrix"] >> t;
        fs_read["R1"] >> r1;
        fs_read["R2"] >> r2;
        fs_read["P1"] >> p1;
        fs_read["P2"] >> p2;
        fs_read["Q"] >> q;
        bino_calib.setCameraParam(left_in, left_d, right_in, right_d, fisheye_flag, r, t, r1, r2, p1, p2, q);
    }
}

// 加载双目图像
void CameraCalibration::receiveFromDialog(QString str)
{
    HiddenIntro();
    QStringList list = str.split(",");
    QString left_src = list[0];
    QString right_src = list[1];
    double chessboard_size = list[2].toDouble();
    bino_calib.setChessboardSize(chessboard_size);
    QDir dir(left_src);
    dir.setFilter(QDir::Files | QDir::NoSymLinks);
    QStringList filters;
    filters << "*.png" << "*.jpg" << "*.jpeg" << "*.jpe" << "*.pbm" << "*.pgm" << "*.ppm" << "*.tiff" << "*.tif" << "*.bmp" << "*.dib";
    dir.setNameFilters(filters);
    QStringList imagesList = dir.entryList();
    if(ui->double_camera->isChecked())
    {
        if(imagesList.length() <= 3)
        {
            QMessageBox::critical(this, "错误", "至少需要四组图片", QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
        QProgressDialog *dialog = new QProgressDialog(tr("检测角点..."),tr("取消"),0,imagesList.length(),this);
        dialog->setWindowModality(Qt::WindowModal);
        dialog->setMinimumDuration(0);
        dialog->setWindowTitle("请稍候");
        dialog->setValue(0);
        QFont font = dialog->font();
        font.setPixelSize(12);
        dialog->setFont(font);
        dialog->show();
        QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
        for(int i = 0; i < imagesList.length(); i++)
        {
            QString left_file_name = left_src + "/" + imagesList[i];
            QString right_file_name = right_src + "/" + imagesList[i];
            std::string str = code->fromUnicode(right_file_name).data();
            cv::Mat right_image = cv::imread(str);
            if(right_image.empty())
            {
                dialog->setValue(i+1);
                continue;
            }
            str = code->fromUnicode(left_file_name).data();
            cv::Mat left_image = cv::imread(str);
            if(left_image.cols != right_image.cols || left_image.rows != right_image.rows)
            {
                QMessageBox::critical(this, "错误", "左右相机图片尺寸不一致", QMessageBox::Yes, QMessageBox::Yes);
                delete dialog;
                return;
            }
            find_corner_thread_img = left_image;
            thread_done = false;
            findcorner_thread->start();
            while(thread_done == false)
            {
                if(dialog->wasCanceled())
                {
                    DealThreadDone();
                    delete dialog;
                    return;
                }
                // 强制Windows消息循环，防止程序出现未响应情况
                QCoreApplication::processEvents();
            }
            struct Chessboarder_t left_chess = find_corner_thread_chessboard;
            // 如果检测到多个棋盘，则剔除该图片
            if(left_chess.chessboard.size() != 1)
            {
                dialog->setValue(i+1);
                continue;
            }
            find_corner_thread_img = right_image;
            thread_done = false;
            findcorner_thread->start();
            while(thread_done == false)
            {
                if(dialog->wasCanceled())
                {
                    DealThreadDone();
                    delete dialog;
                    return;
                }
                QCoreApplication::processEvents();
            }
            struct Chessboarder_t right_chess = find_corner_thread_chessboard;
            // 如果检测到多个棋盘，则剔除该图片
            if(right_chess.chessboard.size() != 1)
            {
                dialog->setValue(i+1);
                continue;
            }
            // 如果左右图片检测到的棋盘尺寸不对，则剔除该组图片
            if(left_chess.chessboard[0].rows != right_chess.chessboard[0].rows ||
                    left_chess.chessboard[0].cols != right_chess.chessboard[0].cols)
            {
                dialog->setValue(i+1);
                continue;
            }
            // 缓存图片及角点
            struct Stereo_Img_t img_;
            img_.left_path = code->fromUnicode(left_file_name).data();
            img_.right_path = code->fromUnicode(right_file_name).data();
            img_.left_file_name = imagesList[i];
            img_.right_file_name = imagesList[i];
            for (unsigned int j = 0; j < left_chess.chessboard.size(); j++)
            {
                for (int u = 0; u < left_chess.chessboard[j].rows; u++)
                {
                    for (int v = 0; v < left_chess.chessboard[j].cols; v++)
                    {
                        img_.left_img_points.push_back(left_chess.corners.p[left_chess.chessboard[j].at<uint16_t>(u, v)]);
                        img_.world_points.push_back(cv::Point3f(u*chessboard_size, v*chessboard_size, 0));
                        img_.right_img_points.push_back(right_chess.corners.p[right_chess.chessboard[j].at<uint16_t>(u, v)]);
                    }
                }
            }
            stereo_imgs.push_back(img_);
            img_size = left_image.size();
            int img_height = left_image.rows*90/left_image.cols;
            cv::resize(left_image, left_image, cv::Size(90, img_height));
            cv::resize(right_image, right_image, cv::Size(90, img_height));
            cv::Mat combian_img = cv::Mat::zeros(cv::Size(180, img_height), CV_8UC3);
            cv::Mat new_roi = combian_img(cv::Rect(0, 0, 90, img_height));
            left_image.convertTo(new_roi, new_roi.type());
            new_roi = combian_img(cv::Rect(90, 0, 90, img_height));
            right_image.convertTo(new_roi, new_roi.type());
            QPixmap pixmap = QPixmap::fromImage(Mat2QImage(combian_img));
            QListWidgetItem* temp = new QListWidgetItem();
            temp->setSizeHint(QSize(220, img_height));
            temp->setIcon(QIcon(pixmap));
            temp->setText(imagesList[i]);
            ui->listWidget->addItem(temp);
            ui->listWidget->setIconSize(QSize(180, img_height));
            dialog->setValue(i+1);
        }
        delete dialog;
    }
    else if(ui->double_undistort->isChecked())
    {
        QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
        for(int i = 0; i < imagesList.length(); i++)
        {
            QString left_file_name = left_src + "/" + imagesList[i];
            QString right_file_name = right_src + "/" + imagesList[i];
            std::string str = code->fromUnicode(right_file_name).data();
            cv::Mat right_image = cv::imread(str);
            if(right_image.empty())
            {
                continue;
            }
            str = code->fromUnicode(left_file_name).data();
            cv::Mat left_image = cv::imread(str);
            if(left_image.cols != right_image.cols || left_image.rows != right_image.rows)
            {
                QMessageBox::critical(this, "错误", "左右相机图片尺寸不一致", QMessageBox::Yes, QMessageBox::Yes);
                return;
            }
            struct Stereo_Img_t img_;
            img_.left_path = code->fromUnicode(left_file_name).data();
            img_.right_path = code->fromUnicode(right_file_name).data();
            img_.left_file_name = imagesList[i];
            img_.right_file_name = imagesList[i];
            stereo_imgs.push_back(img_);
            img_size = left_image.size();
            int img_height = left_image.rows*90/left_image.cols;
            cv::resize(left_image, left_image, cv::Size(90, img_height));
            cv::resize(right_image, right_image, cv::Size(90, img_height));
            cv::Mat combian_img = cv::Mat::zeros(cv::Size(180, img_height), CV_8UC3);
            cv::Mat new_roi = combian_img(cv::Rect(0, 0, 90, img_height));
            left_image.convertTo(new_roi, new_roi.type());
            new_roi = combian_img(cv::Rect(90, 0, 90, img_height));
            right_image.convertTo(new_roi, new_roi.type());
            QPixmap pixmap = QPixmap::fromImage(Mat2QImage(combian_img));
            QListWidgetItem* temp = new QListWidgetItem();
            temp->setSizeHint(QSize(220, img_height));
            temp->setIcon(QIcon(pixmap));
            temp->setText(imagesList[i]);
            ui->listWidget->addItem(temp);
            ui->listWidget->setIconSize(QSize(180, img_height));
        }
    }
}

// 鱼眼相机切换
void CameraCalibration::fisheyeModeSwitch(int state)
{
    fisheye_flag = state==0 ? false : true;
    if(fisheye_flag == true)
    {
        ui->k_2->setDisabled(true);
        ui->k_3->setDisabled(true);
        ui->tangential->setDisabled(true);
    }
    else
    {
        ui->k_2->setEnabled(true);
        ui->k_3->setEnabled(true);
        ui->tangential->setEnabled(true);
    }
}

// 畸变矫正切换
void CameraCalibration::distortModeSwitch()
{
    if((!mono_calib.hasParam() && ui->single_undistort->isChecked()) || 
        (!bino_calib.hasParam() && ui->double_undistort->isChecked()))
    {
        chooseYaml = new choose_yaml(this);
        chooseYaml->setWindowTitle("选择相机参数文件");
        QFont font = chooseYaml->font();
        font.setPixelSize(12);
        chooseYaml->setFont(font);
        chooseYaml->show();
        connect(chooseYaml, SIGNAL(SendSignal(QString)), this, SLOT(receiveYamlPath(QString)));
        return;
    }
    static int cnt = 0;
    cnt++;
    if(cnt%2 == 1)
    {
        distort_flag = true;
    }
    else
    {
        distort_flag = false;
    }
    if(list_select_)
    {
        int id = ui->listWidget->selectionModel()->selectedIndexes()[0].row();
        chooseImage(ui->listWidget->item(id), ui->listWidget->item(id));
    }
}

// Mat格式转QImage格式
QImage CameraCalibration::Mat2QImage(cv::Mat cvImg)
{
    QImage qImg;
    if(cvImg.channels()==3)
    {
        cv::cvtColor(cvImg,cvImg,cv::COLOR_BGR2RGB);
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols, cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_RGB888);
    }
    else if(cvImg.channels()==1)
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

// 单目相机图片加载，逻辑同双目相机图片加载
void CameraCalibration::addImage()
{
    HiddenIntro();
    if(ui->double_camera->isChecked() || ui->double_undistort->isChecked())
    {
        d = new choose_two_dir(this);
        d->setWindowTitle("选择图片文件夹");
        QFont font = d->font();
        font.setPixelSize(12);
        d->setFont(font);
        d->show();
        connect(d, SIGNAL(SendSignal(QString)), this, SLOT(receiveFromDialog(QString)));
        return;
    }
    else if(ui->single_camera->isChecked())
    {
        QStringList path_list = QFileDialog::getOpenFileNames(this, tr("选择图片"), tr("./"), tr("图片文件(*.jpg *.png *.pgm);;所有文件(*.*);;"));
        QProgressDialog *dialog = new QProgressDialog(tr("检测角点..."),tr("取消"),0,path_list.size(), this);
        dialog->setWindowModality(Qt::WindowModal);
        dialog->setMinimumDuration(0);
        dialog->setWindowTitle("请稍候");
        dialog->setValue(0);
        QFont font = dialog->font();
        font.setPixelSize(12);
        dialog->setFont(font);
        dialog->show();
        QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
        for(int i = 0; i < path_list.size(); i++)
        {
            QFileInfo file = QFileInfo(path_list[i]);
            QString file_name = file.fileName();
            std::string str = code->fromUnicode(path_list[i]).data();
            cv::Mat img = cv::imread(str);
            find_corner_thread_img = img;
            thread_done = false;
            findcorner_thread->start();
            while(thread_done == false)
            {
                if(dialog->wasCanceled())
                {
                    DealThreadDone();
                    delete dialog;
                    return;
                }
                QCoreApplication::processEvents();
            }
            struct Chessboarder_t chess = find_corner_thread_chessboard;
            if(chess.chessboard.size() != 1)
            {
                dialog->setValue(i+1);
                continue;
            }
            struct Img_t img_;
            img_.file_path = str;
            img_.file_name = file_name;
            std::vector<cv::Point2f> img_p;
            std::vector<cv::Point3f> world_p;
            for (unsigned int j = 0; j < chess.chessboard.size(); j++)
            {
                for (int u = 0; u < chess.chessboard[j].rows; u++)
                {
                    for (int v = 0; v < chess.chessboard[j].cols; v++)
                    {
                        img_p.push_back(chess.corners.p[chess.chessboard[j].at<uint16_t>(u, v)]);
                        world_p.push_back(cv::Point3f(u*mono_calib.getChessboardSize(), v*mono_calib.getChessboardSize(), 0));
                    }
                }
            }
            img_.img_points = img_p;
            img_.world_points = world_p;
            imgs.push_back(img_);
            img_size = img.size();
            int img_height = img.rows*124/img.cols;
            cv::resize(img, img, cv::Size(124, img_height));
            QPixmap pixmap = QPixmap::fromImage(Mat2QImage(img));
            QListWidgetItem* temp = new QListWidgetItem();
            temp->setSizeHint(QSize(200, img_height));
            temp->setIcon(QIcon(pixmap));
            temp->setText(file_name);
            ui->listWidget->addItem(temp);
            ui->listWidget->setIconSize(QSize(124, img_height));
            dialog->setValue(i+1);
        }
        delete dialog;
    }
    else if(ui->single_undistort->isChecked())
    {
        QStringList path_list = QFileDialog::getOpenFileNames(this, tr("选择图片"), tr("./"), tr("图片文件(*.jpg *.png *.pgm);;所有文件(*.*);;"));
        QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
        for(int i = 0; i < path_list.size(); i++)
        {
            QFileInfo file = QFileInfo(path_list[i]);
            QString file_name = file.fileName();
            std::string str = code->fromUnicode(path_list[i]).data();
            cv::Mat img = cv::imread(str);
            img_size = img.size();
            Img_t single_img;
            single_img.file_path = str;
            single_img.file_name = file_name;
            imgs.push_back(single_img);
            int img_height = img.rows*124/img.cols;
            cv::resize(img, img, cv::Size(124, img_height));
            QPixmap pixmap = QPixmap::fromImage(Mat2QImage(img));
            QListWidgetItem* temp = new QListWidgetItem();
            temp->setSizeHint(QSize(200, img_height));
            temp->setIcon(QIcon(pixmap));
            temp->setText(file_name);
            ui->listWidget->addItem(temp);
            ui->listWidget->setIconSize(QSize(124, img_height));
        }
    }
}

// 删除选中的图片
void CameraCalibration::deleteImage()
{
    std::vector<int> delete_idx;
    foreach(QModelIndex index,ui->listWidget->selectionModel()->selectedIndexes()){
        delete_idx.push_back(index.row());
    }
    if(delete_idx.size() == 0)
        return;
    std::sort(delete_idx.begin(), delete_idx.end());
    disconnect(ui->listWidget, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), this, SLOT(chooseImage(QListWidgetItem*, QListWidgetItem*)));
    for(int i = delete_idx.size()-1; i >= 0; i--)
    {
        ui->listWidget->takeItem(delete_idx[i]);
        if(ui->double_camera->isChecked())
            stereo_imgs.erase(stereo_imgs.begin()+delete_idx[i]);
        else
            imgs.erase(imgs.begin()+delete_idx[i]);
    }
    connect(ui->listWidget, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), this, SLOT(chooseImage(QListWidgetItem*, QListWidgetItem*)));
    // 如果图片全部删除，则重新显示文字引导
    if(ui->listWidget->count() == 0)
    {
        reset();
    }
}

// 相机标定
void CameraCalibration::calibrate()
{
    if(!ui->double_camera->isChecked())
    {
        // 单目标定
        int state = mono_calib.calibrate(imgs, fisheye_flag, ui->tangential->isChecked(), ui->k_3->isChecked());
        if(state == 1)
        {
            QMessageBox::critical(this, "错误", "至少需要四张图片", QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
        else if(state == 2)
        {
            QMessageBox::critical(this, "错误", "请采集更多方位的图片或者检查角点识别！", QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
    }
    else
    {
        if(!bino_calib.isChessboardSizeValid())
        {
            bool ok;
            double chessboard_size = QInputDialog::getDouble(this,tr("角点间距"),tr("请输入角点间距(mm)"),20,0,1000,2,&ok);
            if(!ok)
            {
                chessboard_size = 0;
                return;
            }
            bino_calib.setChessboardSize(chessboard_size);
        }
        // 双目标定
        int state = bino_calib.calibrate(stereo_imgs, fisheye_flag, ui->tangential->isChecked(), ui->k_3->isChecked());
        if(state == 1)
        {
            QMessageBox::critical(this, "错误", "至少需要四组图片", QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
        else if(state == 2)
        {
            QMessageBox::critical(this, "错误", "请采集更多方位的图片！", QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
    }
}

// 导出标定参数
void CameraCalibration::exportParam()
{
    if((!mono_calib.hasParam() && ui->single_undistort->isChecked()) || 
        (!bino_calib.hasParam() && ui->double_undistort->isChecked()))
    {
        QMessageBox::critical(this, "错误", "请先标定相机", QMessageBox::Yes);
        return;
    }
    QString strSaveName = QFileDialog::getSaveFileName(this,tr("保存的文件"),tr("calibration_param.yaml"),tr("yaml files(*.yaml)"));
    if(strSaveName.isNull() || strSaveName.isEmpty() || strSaveName.length() == 0)
        return;
    cv::FileStorage fs_write(strSaveName.toStdString(), cv::FileStorage::WRITE);
    if(ui->double_camera->isChecked())
    {
        bino_calib.exportParam(strSaveName.toStdString());
    }
    else
    {
        mono_calib.exportParam(strSaveName.toStdString());
    }
}
