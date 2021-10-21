#ifndef BINOCULAR_CALIB_H
#define BINOCULAR_CALIB_H

#include <opencv2/opencv.hpp>
#include "types.h"

class BinoCalib
{
    public:
    BinoCalib();
    ~BinoCalib();
    int calibrate(std::vector<Stereo_Img_t>& imgs, bool fisheye, bool tangential, bool k3);
    void setChessboardSize(double chessboard_size)
    {
        chessboard_size_ = chessboard_size;
    }
    bool isChessboardSizeValid()
    {
        if(chessboard_size_ > 0) return true;
        else return false;
    }
    void undistort(cv::Mat& left, cv::Mat &right);
    void setCameraParam(cv::Mat intrinsic1, cv::Mat distCoefficients1_,
                        cv::Mat intrinsic2, cv::Mat distCoefficients2_, bool fisheye,
                        cv::Mat R, cv::Mat T, cv::Mat R1, cv::Mat R2, cv::Mat P1, cv::Mat P2, cv::Mat Q);
    void reset()
    {
        chessboard_size_ = 0;
        has_param_ = false;
    }
    void exportParam(std::string file_name);
    void showCorners(cv::Mat& left, cv::Mat& right, Stereo_Img_t& img);
    double getChessboardSize()
    {
        return chessboard_size_;
    }
    bool hasParam()
    {
        return has_param_;
    }
    private:
    cv::Mat cameraMatrix1, cameraMatrix2;
    cv::Mat distCoefficients1, distCoefficients2;
    cv::Matx33d K1, K2;
    cv::Vec4d D1, D2;
    bool has_param_;
    bool fisheye_flag_;
    float chessboard_size_;
    cv::Mat R, T, R1, R2, P1, P2, Q;
    bool map_init_;
    cv::Mat left_mapx, left_mapy, right_mapx, right_mapy;
};

#endif