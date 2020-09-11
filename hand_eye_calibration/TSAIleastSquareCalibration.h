#ifndef TSAILEASTSQUARECALIBRATION_H
#define TSAILEASTSQUARECALIBRATION_H

#include <opencv2/opencv.hpp>
#include "../camera_calibration/CameraCalibration.h"

QT_BEGIN_NAMESPACE
namespace Ui { class TSAIleastSquareCalibration; }
QT_END_NAMESPACE

class TSAIleastSquareCalibration
{
public:
    TSAIleastSquareCalibration();
    static void runCalibration(std::vector<cv::Mat> Hg, std::vector<cv::Mat> Hc, int mode, cv::Mat &extrinsicMatrix);
    static void crossprod(cv::Mat a, cv::Mat &ax);
};

#endif // TSAILEASTSQUARECALIBRATION_H
