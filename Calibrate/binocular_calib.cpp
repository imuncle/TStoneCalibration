#include "binocular_calib.h"

BinoCalib::BinoCalib():
    has_param_(false),
    chessboard_size_(0),
    fisheye_flag_(false),
    map_init_(false)
{

}

BinoCalib::~BinoCalib()
{

}

int BinoCalib::calibrate(std::vector<Stereo_Img_t>& imgs, bool fisheye, bool tangential, bool k3)
{
    if(imgs.size() <= 3)
    {
        return 1;
    }
    fisheye_flag_ = fisheye;
    cv::Mat img = cv::imread(imgs[0].left_path);
    cv::Size img_size = img.size();
    std::vector<std::vector<cv::Point2f>> left_img_points;
    std::vector<std::vector<cv::Point2f>> right_img_points;
    std::vector<std::vector<cv::Point3f>> world_points;
    std::vector<cv::Mat> left_tvecsMat;
    std::vector<cv::Mat> left_rvecsMat;
    std::vector<cv::Mat> right_tvecsMat;
    std::vector<cv::Mat> right_rvecsMat;
    std::vector<std::vector<cv::Point2d> > left_imagePoints;
    std::vector<std::vector<cv::Point2d> > right_imagePoints;
    std::vector<std::vector<cv::Point3d> > objectPoints;
    std::vector<cv::Vec3d> left_rvecs;
    std::vector<cv::Vec3d> left_tvecs;
    std::vector<cv::Vec3d> right_rvecs;
    std::vector<cv::Vec3d> right_tvecs;
    for (unsigned int j = 0; j < imgs.size(); j++)
    {
        left_img_points.push_back(imgs[j].left_img_points);
        right_img_points.push_back(imgs[j].right_img_points);
        world_points.push_back(imgs[j].world_points);
        std::vector<cv::Point2d> img_p, img_p_;
        std::vector<cv::Point3d> obj_p;
        for(unsigned int i = 0; i < imgs[j].left_img_points.size(); i++)
        {
            img_p.push_back(imgs[j].left_img_points[i]);
            img_p_.push_back(imgs[j].right_img_points[i]);
            obj_p.push_back(imgs[j].world_points[i]);
        }
        left_imagePoints.push_back(img_p);
        right_imagePoints.push_back(img_p_);
        objectPoints.push_back(obj_p);
    }
    cameraMatrix1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);
    cameraMatrix2 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);
    if(fisheye == false)
    {
        distCoefficients1 = cv::Mat::zeros(cv::Size(5, 1), CV_32F);
        distCoefficients2 = cv::Mat::zeros(cv::Size(5, 1), CV_32F);
    }

    // 单目标定
    if(fisheye == false)
    {
        int flag = 0;
        if(!tangential)
            flag |= cv::CALIB_ZERO_TANGENT_DIST;
        if(!k3)
            flag |= cv::CALIB_FIX_K3;
        cv::calibrateCamera(world_points, left_img_points, img_size, cameraMatrix1, distCoefficients1, left_rvecsMat, left_tvecsMat, flag);
        cv::calibrateCamera(world_points, right_img_points, img_size, cameraMatrix2, distCoefficients2, right_rvecsMat, right_tvecsMat, flag);
    }
    else
    {
        int flag = 0;
        flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        flag |= cv::fisheye::CALIB_FIX_SKEW;
        cv::fisheye::calibrate(objectPoints, left_imagePoints, img_size, K1, D1, left_rvecs, left_tvecs, flag);
        cv::fisheye::calibrate(objectPoints, right_imagePoints, img_size, K2, D2, right_rvecs, right_tvecs, flag);
    }
    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        if(fisheye == false)
        {
            imgs[i].left_tvec = left_tvecsMat[i];
            imgs[i].left_rvec = left_rvecsMat[i];
            imgs[i].right_tvec = right_tvecsMat[i];
            imgs[i].right_rvec = right_rvecsMat[i];
        }
        else
        {
            imgs[i].left_fish_tvec = left_tvecs[i];
            imgs[i].left_fish_rvec = left_rvecs[i];
            imgs[i].right_fish_tvec = right_tvecs[i];
            imgs[i].right_fish_rvec = right_rvecs[i];
        }
    }

    // 评估标定结果
    std::vector<float> left_error, right_error;
    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        std::vector<cv::Point3f> world_p = imgs[i].world_points;
        std::vector<cv::Point2f> left_img_p = imgs[i].left_img_points, right_img_p = imgs[i].right_img_points;
        std::vector<cv::Point2f> left_reproject_img_p, right_reproject_img_p;
        std::vector<cv::Point2d> left_fisheye_reproject_p, right_fisheye_reproject_p;
        if(fisheye == false)
        {
            projectPoints(world_p, left_rvecsMat[i], left_tvecsMat[i], cameraMatrix1, distCoefficients1, left_reproject_img_p);
            projectPoints(world_p, right_rvecsMat[i], right_tvecsMat[i], cameraMatrix2, distCoefficients2, right_reproject_img_p);
        }
        else
        {
            cv::fisheye::projectPoints(objectPoints[i], left_fisheye_reproject_p, left_rvecs[i], left_tvecs[i], K1, D1);
            cv::fisheye::projectPoints(objectPoints[i], right_fisheye_reproject_p, right_rvecs[i], right_tvecs[i], K2, D2);
        }
        float left_err = 0, right_err = 0;
        for (unsigned int j = 0; j < world_p.size(); j++)
        {
            if(fisheye == false)
            {
                left_err += sqrt((left_img_p[j].x-left_reproject_img_p[j].x)*(left_img_p[j].x-left_reproject_img_p[j].x)+
                            (left_img_p[j].y-left_reproject_img_p[j].y)*(left_img_p[j].y-left_reproject_img_p[j].y));
                right_err += sqrt((right_img_p[j].x-right_reproject_img_p[j].x)*(right_img_p[j].x-right_reproject_img_p[j].x)+
                            (right_img_p[j].y-right_reproject_img_p[j].y)*(right_img_p[j].y-right_reproject_img_p[j].y));
            }
            else
            {
                left_err += sqrt((left_img_p[j].x-left_fisheye_reproject_p[j].x)*(left_img_p[j].x-left_fisheye_reproject_p[j].x)+
                            (left_img_p[j].y-left_fisheye_reproject_p[j].y)*(left_img_p[j].y-left_fisheye_reproject_p[j].y));
                right_err += sqrt((right_img_p[j].x-right_fisheye_reproject_p[j].x)*(right_img_p[j].x-right_fisheye_reproject_p[j].x)+
                            (right_img_p[j].y-right_fisheye_reproject_p[j].y)*(right_img_p[j].y-right_fisheye_reproject_p[j].y));
            }
        }
        left_error.push_back(left_err/world_p.size());
        right_error.push_back(right_err/world_p.size());
    }
    int max_idx = max_element(left_error.begin(), left_error.end()) - left_error.begin();
    float max_error = left_error[max_idx];
    max_idx = max_element(right_error.begin(), right_error.end()) - right_error.begin();
    max_error = std::max(max_error, right_error[max_idx]);
    int width = 480 / imgs.size();
    cv::Mat error_plot = cv::Mat(260, 560, CV_32FC3, cv::Scalar(255,255,255));
    cv::rectangle(error_plot, cv::Rect(40, 20, 480, 200), cv::Scalar(0, 0, 0),1, cv::LINE_8,0);
    cv::putText(error_plot, "0", cv::Point(20, 220), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, 0);
    char *chCode;
    chCode = new(std::nothrow)char[20];
    sprintf(chCode, "%.2lf", max_error*200/195);
    std::string strCode(chCode);
    delete []chCode;
    cv::putText(error_plot, strCode, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, 0);
    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        int height = 195*left_error[i]/max_error;
        cv::rectangle(error_plot, cv::Rect(i*width+43, 220-height, width/2-4, height), cv::Scalar(255,0,0), -1, cv::LINE_8,0);
        height = 195*right_error[i]/max_error;
        cv::rectangle(error_plot, cv::Rect(i*width+41+width/2, 220-height, width/2-4, height), cv::Scalar(0,255,0), -1, cv::LINE_8,0);
        cv::putText(error_plot, std::to_string(i), cv::Point(i*width+40+width/2-2, 240), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, 0);
    }

    // 双目标定
    if(fisheye == false)
    {
        int flag = 0;
        if(!tangential)
            flag |= cv::CALIB_ZERO_TANGENT_DIST;
        if(!k3)
            flag |= cv::CALIB_FIX_K3;
        flag |= cv::CALIB_USE_INTRINSIC_GUESS;
        cv::stereoCalibrate(world_points, left_img_points, right_img_points,
                            cameraMatrix1, distCoefficients1,
                            cameraMatrix2, distCoefficients2,
                            img_size, R, T, cv::noArray(), cv::noArray(), flag);
        cv::stereoRectify(cameraMatrix1, distCoefficients1,
                                cameraMatrix2, distCoefficients2,
                                img_size, R, T,
                                R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, img_size);
    }
    else
    {
        int flag = 0;
        flag |= cv::fisheye::CALIB_FIX_SKEW;
        flag |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
        try {
            cv::fisheye::stereoCalibrate(objectPoints, left_imagePoints, right_imagePoints,
                                                K1, D1,
                                                K2, D2,
                                                img_size, R, T, flag);
        } catch (cv::Exception& e) {
            return 2;
        }

        cv::fisheye::stereoRectify(K1, D1,
                                K2, D2,
                                img_size, R, T,
                                R1, R2, P1, P2, Q, 0, img_size);
    }
    has_param_ = true;
    cv::imshow("error", error_plot);
}

void BinoCalib::exportParam(std::string file_name)
{
    cv::FileStorage fs_write(file_name, cv::FileStorage::WRITE);
    if(fisheye_flag_ == false)
    {
        fs_write << "left_camera_Matrix" << cameraMatrix1 << "left_camera_distCoeffs" << distCoefficients1;
        fs_write << "right_camera_Matrix" << cameraMatrix2 << "right_camera_distCoeffs" << distCoefficients2;
        fs_write << "Rotate_Matrix" << R << "Translate_Matrix" << T;
        fs_write << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
    }
    else
    {
        cv::Mat camera_matrix = cv::Mat::zeros(cv::Size(3,3), CV_64F);
        camera_matrix.at<double>(0,0) = K1(0,0);
        camera_matrix.at<double>(0,2) = K1(0,2);
        camera_matrix.at<double>(1,1) = K1(1,1);
        camera_matrix.at<double>(1,2) = K1(1,2);
        camera_matrix.at<double>(2,2) = 1;
        cv::Mat Dist = (cv::Mat_<double>(1,4) << D1[0], D1[1], D1[2], D1[3]);
        fs_write << "left_camera_Matrix" << camera_matrix << "left_camera_distCoeffs" << Dist;
        camera_matrix.at<double>(0,0) = K2(0,0);
        camera_matrix.at<double>(0,2) = K2(0,2);
        camera_matrix.at<double>(1,1) = K2(1,1);
        camera_matrix.at<double>(1,2) = K2(1,2);
        Dist = (cv::Mat_<double>(1,4) << D2[0], D2[1], D2[2], D2[3]);
        fs_write << "right_camera_Matrix" << camera_matrix << "right_camera_distCoeffs" << Dist;
        fs_write << "Rotate_Matrix" << R << "Translate_Matrix" << T;
        fs_write << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
    }
    fs_write.release();
}

void BinoCalib::setCameraParam(cv::Mat intrinsic1, cv::Mat distCoefficients1_,
                        cv::Mat intrinsic2, cv::Mat distCoefficients2_, bool fisheye,
                        cv::Mat R_, cv::Mat T_, cv::Mat R1_, cv::Mat R2_, cv::Mat P1_, cv::Mat P2_, cv::Mat Q_)
{
    R = R_;
    T = T_;
    P1 = P1_;
    P2 = P2_;
    R1 = R1_;
    R2 = R2_;
    Q = Q_;
    if(fisheye == false)
    {
        cameraMatrix1 = intrinsic1;
        distCoefficients1 = distCoefficients1_;
        cameraMatrix2 = intrinsic2;
        distCoefficients2 = distCoefficients2_;
    }
    else
    {
        K1(0,0) = intrinsic1.at<double>(0,0);
        K1(0,1) = 0;
        K1(0,2) = intrinsic1.at<double>(0,2);
        K1(1,0) = 0;
        K1(1,1) = intrinsic1.at<double>(1,1);
        K1(1,2) = intrinsic1.at<double>(1,2);
        K1(1,0) = 0;
        K1(2,1) = 0;
        K1(2,2) = 1;
        D1[0] = distCoefficients1_.at<double>(0,0);
        D1[1] = distCoefficients1_.at<double>(0,1);
        D1[2] = distCoefficients1_.at<double>(0,2);
        D1[3] = distCoefficients1_.at<double>(0,3);

        K2(0,0) = intrinsic2.at<double>(0,0);
        K2(0,1) = 0;
        K2(0,2) = intrinsic2.at<double>(0,2);
        K2(1,0) = 0;
        K2(1,1) = intrinsic2.at<double>(1,1);
        K2(1,2) = intrinsic2.at<double>(1,2);
        K2(1,0) = 0;
        K2(2,1) = 0;
        K2(2,2) = 1;
        D2[0] = distCoefficients2_.at<double>(0,0);
        D2[1] = distCoefficients2_.at<double>(0,1);
        D2[2] = distCoefficients2_.at<double>(0,2);
        D2[3] = distCoefficients2_.at<double>(0,3);
    }
    has_param_ = true;
}

void BinoCalib::undistort(cv::Mat& left, cv::Mat& right)
{
    if(!has_param_) return;
    if(map_init_ == false)
    {
        if(fisheye_flag_ == false)
        {
            cv::initUndistortRectifyMap(cameraMatrix1, distCoefficients1, R1, P1, left.size(), CV_16SC2, left_mapx, left_mapy);
            cv::initUndistortRectifyMap(cameraMatrix2, distCoefficients2, R2, P2, left.size(), CV_16SC2, right_mapx, right_mapy);
        }
        else
        {
            cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, left.size(), CV_16SC2, left_mapx, left_mapy);
            cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, left.size(), CV_16SC2, right_mapx, right_mapy);
        }
        map_init_ = true;
    }
    cv::remap(left, left, left_mapx, left_mapy, cv::INTER_LINEAR);
    cv::remap(right, right, right_mapx, right_mapy, cv::INTER_LINEAR);
}

void BinoCalib::showCorners(cv::Mat& left, cv::Mat& right, Stereo_Img_t& img)
{
    // 非矫正模式下显示角点位置
    for(unsigned int i = 0; i < img.left_img_points.size(); i++)
    {
        cv::circle(left, img.left_img_points[i], 5, cv::Scalar(0,0,255), 2);
    }
    for(unsigned int i = 0; i < img.right_img_points.size(); i++)
    {
        cv::circle(right, img.right_img_points[i], 5, cv::Scalar(0,0,255), 2);
    }
    if(has_param_)
    {
        // 显示标定后角点的重投影
        if(fisheye_flag_ == false)
        {
            std::vector<cv::Point2f> reproject_img_p;
            projectPoints(img.world_points, img.left_rvec, img.left_tvec, cameraMatrix1, distCoefficients1, reproject_img_p);
            for(unsigned int i = 0; i < reproject_img_p.size(); i++)
            {
                cv::circle(left, reproject_img_p[i], 3, cv::Scalar(255,0,0), 2);
            }
            projectPoints(img.world_points, img.right_rvec, img.right_tvec, cameraMatrix2, distCoefficients2, reproject_img_p);
            for(unsigned int i = 0; i < reproject_img_p.size(); i++)
            {
                cv::circle(right, reproject_img_p[i], 3, cv::Scalar(255,0,0), 2);
            }
        }
        else
        {
            std::vector<cv::Point2d> reproject_img_p;
            std::vector<cv::Point3d> w_p_;
            for(unsigned int i = 0; i < img.world_points.size(); i++)
            {
                w_p_.push_back(img.world_points[i]);
            }
            cv::fisheye::projectPoints(w_p_, reproject_img_p, img.left_fish_rvec, img.left_fish_tvec, K1, D1);
            for(unsigned int i = 0; i < reproject_img_p.size(); i++)
            {
                cv::circle(left, reproject_img_p[i], 3, cv::Scalar(255,0,0), 2);
            }
            cv::fisheye::projectPoints(w_p_, reproject_img_p, img.right_fish_rvec, img.right_fish_tvec, K2, D2);
            for(unsigned int i = 0; i < reproject_img_p.size(); i++)
            {
                cv::circle(right, reproject_img_p[i], 3, cv::Scalar(255,0,0), 2);
            }
        }
    }
}