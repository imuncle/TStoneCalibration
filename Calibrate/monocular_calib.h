#ifndef MONOCULAR_CALIB_H
#define MONOCULAR_CALIB_H

#include "types.h"
#include <opencv2/opencv.hpp>

class MonoCalib
{
    public:
    MonoCalib();
    ~MonoCalib();
    int calibrate(std::vector<Img_t>& imgs, bool fisheye, bool tangential, bool k3);
    void undistort(cv::Mat& img);
    void setCameraParam(cv::Mat intrinsic, cv::Mat distCoefficients, bool fisheye);
    void reset()
    {
        has_param_ = false;
        map_init_ = false;
    }
    void exportParam(std::string file_name);
    void showCorners(cv::Mat& left, Img_t& img);
    double getChessboardSize()
    {
        return chessboard_size_;
    }
    bool hasParam()
    {
        return has_param_;
    }
    private:
    cv::Mat intrinsic_;
    cv::Mat distCoefficients_;
    bool has_param_;
    bool fisheye_flag_;
    double chessboard_size_;
    cv::Matx33d K;
    cv::Vec4d D;
    cv::Mat mapx, mapy;
    bool map_init_;
};

#endif