#include "monocular_calib.h"

MonoCalib::MonoCalib():
    has_param_(false),
    chessboard_size_(20.0),
    fisheye_flag_(false),
    map_init_(false)
{

}

MonoCalib::~MonoCalib()
{

}

int MonoCalib::calibrate(std::vector<Img_t>& imgs, bool fisheye, bool tangential, bool k3)
{
    if(imgs.size() <= 3)
    {
        return 1;
    }
    cv::Mat img = cv::imread(imgs[0].file_path);
    cv::Size img_size = img.size();
    fisheye_flag_ = fisheye;
    std::vector<std::vector<cv::Point2f>> img_points;
    std::vector<std::vector<cv::Point3f>> world_points;
    std::vector<cv::Mat> tvecsMat;
    std::vector<cv::Mat> rvecsMat;
    std::vector<std::vector<cv::Point2d> > imagePoints;
    std::vector<std::vector<cv::Point3d> > objectPoints;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    for (unsigned int j = 0; j < imgs.size(); j++)
    {
        img_points.push_back(imgs[j].img_points);
        world_points.push_back(imgs[j].world_points);
        std::vector<cv::Point2d> img_p;
        std::vector<cv::Point3d> obj_p;
        for(unsigned int i = 0; i < imgs[j].img_points.size(); i++)
        {
            img_p.push_back(imgs[j].img_points[i]);
            obj_p.push_back(imgs[j].world_points[i]);
        }
        imagePoints.push_back(img_p);
        objectPoints.push_back(obj_p);
    }
    intrinsic_ = cv::Mat::zeros(cv::Size(3, 3), CV_32F);
    if(fisheye_flag_ == false)
        distCoefficients_ = cv::Mat::zeros(cv::Size(5, 1), CV_32F);

    if(fisheye_flag_ == false)
    {
        int flag = 0;
        if(!tangential)
            flag |= cv::CALIB_ZERO_TANGENT_DIST;
        if(!k3)
            flag |= cv::CALIB_FIX_K3;
        cv::calibrateCamera(world_points, img_points, img_size, intrinsic_, distCoefficients_, rvecsMat, tvecsMat, flag);
    }
    else
    {
        int flag = 0;
        flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        flag |= cv::fisheye::CALIB_FIX_SKEW;
        try {
            cv::fisheye::calibrate(objectPoints, imagePoints, img_size, K, D, rvecs, tvecs, flag);
        } catch (cv::Exception& e) {
            return 2;
        }
    }
    has_param_ = true;
    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        if(fisheye_flag_ == false)
        {
            imgs[i].tvec = tvecsMat[i];
            imgs[i].rvec = rvecsMat[i];
        }
        else
        {
            imgs[i].fish_tvec = tvecs[i];
            imgs[i].fish_rvec = rvecs[i];
        }
    }
    // 评估标定结果
    std::vector<float> error;
    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        std::vector<cv::Point3f> world_p = imgs[i].world_points;
        std::vector<cv::Point2f> img_p = imgs[i].img_points, reproject_img_p;
        std::vector<cv::Point2d> fisheye_reproject_p;
        if(fisheye_flag_ == false)
            projectPoints(world_p, rvecsMat[i], tvecsMat[i], intrinsic_, distCoefficients_, reproject_img_p);
        else
            cv::fisheye::projectPoints(objectPoints[i], fisheye_reproject_p, rvecs[i], tvecs[i], K, D);
        float err = 0;
        for (unsigned int j = 0; j < img_p.size(); j++)
        {
            if(fisheye_flag_ == false)
                err += sqrt((img_p[j].x-reproject_img_p[j].x)*(img_p[j].x-reproject_img_p[j].x)+
                            (img_p[j].y-reproject_img_p[j].y)*(img_p[j].y-reproject_img_p[j].y));
            else
                err += sqrt((img_p[j].x-fisheye_reproject_p[j].x)*(img_p[j].x-fisheye_reproject_p[j].x)+
                            (img_p[j].y-fisheye_reproject_p[j].y)*(img_p[j].y-fisheye_reproject_p[j].y));
        }
        error.push_back(err/img_p.size());
    }
    int max_idx = max_element(error.begin(), error.end()) - error.begin();
    float max_error = error[max_idx];
    int width = 240 / imgs.size();
    cv::Mat error_plot = cv::Mat(260, 320, CV_32FC3, cv::Scalar(255,255,255));
    cv::rectangle(error_plot, cv::Rect(40, 20, 240, 200), cv::Scalar(0, 0, 0),1, cv::LINE_8,0);
    cv::putText(error_plot, "0", cv::Point(20, 220), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, 0);
    char *chCode;
    chCode = new(std::nothrow)char[20];
    sprintf(chCode, "%.2lf", max_error*200/195);
    std::string strCode(chCode);
    delete []chCode;
    cv::putText(error_plot, strCode, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, 0);
    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        int height = 195*error[i]/max_error;
        cv::rectangle(error_plot, cv::Rect(i*width+41, 220-height, width-2, height), cv::Scalar(255,0,0), -1, cv::LINE_8,0);
        cv::putText(error_plot, std::to_string(i), cv::Point(i*width+40, 240), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, 0);
    }
    cv::imshow("error", error_plot);
    return 0;
}

void MonoCalib::exportParam(std::string file_name)
{
    cv::FileStorage fs_write(file_name, cv::FileStorage::WRITE);
    if(fisheye_flag_ == false)
        fs_write << "cameraMatrix" << intrinsic_ << "distCoeffs" << distCoefficients_;
    else
    {
        cv::Mat camera_matrix = cv::Mat::zeros(cv::Size(3,3), CV_64F);
        camera_matrix.at<double>(0,0) = K(0,0);
        camera_matrix.at<double>(0,2) = K(0,2);
        camera_matrix.at<double>(1,1) = K(1,1);
        camera_matrix.at<double>(1,2) = K(1,2);
        camera_matrix.at<double>(2,2) = 1;
        cv::Mat Dist = (cv::Mat_<double>(1,4) << D[0], D[1], D[2], D[3]);
        fs_write << "cameraMatrix" << camera_matrix << "distCoeffs" << Dist;
    }
    fs_write.release();
}

void MonoCalib::setCameraParam(cv::Mat intrinsic, cv::Mat distCoefficients, bool fisheye)
{
    if(fisheye == false)
    {
        intrinsic_ = intrinsic;
        distCoefficients_ = distCoefficients;
    }
    else
    {
        K(0,0) = intrinsic.at<double>(0,0);
        K(0,1) = 0;
        K(0,2) = intrinsic.at<double>(0,2);
        K(1,0) = 0;
        K(1,1) = intrinsic.at<double>(1,1);
        K(1,2) = intrinsic.at<double>(1,2);
        K(1,0) = 0;
        K(2,1) = 0;
        K(2,2) = 1;
        D[0] = distCoefficients.at<double>(0,0);
        D[1] = distCoefficients.at<double>(0,1);
        D[2] = distCoefficients.at<double>(0,2);
        D[3] = distCoefficients.at<double>(0,3);
    }
    has_param_ = true;
}

void MonoCalib::undistort(cv::Mat& img)
{
    if(!has_param_) return;
    if(map_init_ == false)
    {
        if(fisheye_flag_ == false)
        {
            cv::initUndistortRectifyMap(intrinsic_, distCoefficients_, cv::noArray(), intrinsic_, img.size(), CV_16SC2, mapx, mapy);
        }
        else
        {
            cv::fisheye::initUndistortRectifyMap(K, D, cv::noArray(), intrinsic_, img.size(), CV_16SC2, mapx, mapy);
        }
        map_init_ = true;
    }
    cv::remap(img, img, mapx, mapy, cv::INTER_LINEAR);
}

void MonoCalib::showCorners(cv::Mat& left, Img_t& img)
{
    for(unsigned int i = 0; i < img.img_points.size(); i++)
    {
        cv::circle(left, img.img_points[i], 5, cv::Scalar(0,0,255), 2);
    }
    if(has_param_ == true)
    {
        if(fisheye_flag_ == false)
        {
            std::vector<cv::Point2f> reproject_img_p;
            projectPoints(img.world_points, img.rvec, img.tvec, intrinsic_, distCoefficients_, reproject_img_p);
            for(unsigned int i = 0; i < reproject_img_p.size(); i++)
            {
                cv::circle(left, reproject_img_p[i], 3, cv::Scalar(255,0,0), 2);
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
            cv::fisheye::projectPoints(w_p_, reproject_img_p, img.fish_rvec, img.fish_tvec, K, D);
            for(unsigned int i = 0; i < reproject_img_p.size(); i++)
            {
                cv::circle(left, reproject_img_p[i], 3, cv::Scalar(255,0,0), 2);
            }
        }
    }
}