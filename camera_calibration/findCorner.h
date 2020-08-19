#ifndef FIND_CORNER_H
#define FIND_CORNER_H

#include <opencv2/opencv.hpp>
#include "chessboard.h"
#include <QThread>

class FindcornerThread : public QThread
{
    Q_OBJECT
public:
    explicit FindcornerThread(QObject *parent = 0);
    ~FindcornerThread();

protected:
    void run();

signals:
    void isDone();  //处理完成信号
};

struct Template_t
{
    cv::Mat a1;
    cv::Mat a2;
    cv::Mat b1;
    cv::Mat b2;
};

struct Chessboarder_t
{
    struct Corner_t corners;
    std::vector<cv::Mat> chessboard;
};

struct Chessboarder_t findCorner(cv::Mat img, int sigma);
void secondDerivCornerMetric(cv::Mat img, int sigma, cv::Mat *cxy, cv::Mat *Ixy);
struct Corner_t getOrientations(cv::Mat img_angle, cv::Mat img_weight, struct Corner_t corners, int r);
void edgeOrientations(cv::Mat img_angle, cv::Mat img_weight, cv::Point2d* v1, cv::Point2d* v2);
std::vector<cv::Point3d> findModesMeanShift(cv::Mat hist, int sigma);
struct Corner_t subPixelLocation(cv::Mat metric, struct Corner_t corners);

#endif
