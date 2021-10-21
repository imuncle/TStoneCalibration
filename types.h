#ifndef TYPES_H
#define TYPES_H

#include <QString>
#include <opencv2/opencv.hpp>

struct Img_t
{
    std::string file_path;
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
    std::string left_path;
    QString left_file_name;
    std::vector<cv::Point2f> left_img_points;
    cv::Mat left_tvec;
    cv::Mat left_rvec;
    cv::Vec3d left_fish_tvec;
    cv::Vec3d left_fish_rvec;

    std::string right_path;
    QString right_file_name;
    std::vector<cv::Point2f> right_img_points;
    cv::Mat right_tvec;
    cv::Mat right_rvec;
    cv::Vec3d right_fish_tvec;
    cv::Vec3d right_fish_rvec;

    std::vector<cv::Point3f> world_points;
};

#endif