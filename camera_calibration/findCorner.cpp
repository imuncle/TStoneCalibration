#include "findCorner.h"
#include "chessboard.h"
#include "CameraCalibration.h"
#include <algorithm>
#include <math.h>
#include <numeric>

FindcornerThread::FindcornerThread(QObject *parent){}

FindcornerThread::~FindcornerThread(){}

void FindcornerThread::run()
{
    find_corner_thread_chessboard = findCorner(find_corner_thread_img, 2);
    emit isDone();  //发送完成信号
}

struct Chessboarder_t findCorner(cv::Mat img, int sigma)
{
    if (img.channels() == 3)
        cv::cvtColor(img, img, CV_BGR2GRAY);
    cv::Mat du = (cv::Mat_<double>(3, 3) << -1, 0, 1, -1, 0, 1, -1, 0, 1);
    cv::Mat dv = du.t();
    cv::Mat img_dv, img_du;
    cv::Mat img_angle = cv::Mat::zeros(img.size(), CV_64F);
    cv::Mat img_weight = cv::Mat::zeros(img.size(), CV_64F);
    cv::Mat img_double;
    img.convertTo(img_double, CV_64F);
    cv::filter2D(img_double, img_du, img_double.depth(), du, cvPoint(-1, -1));
    cv::filter2D(img_double, img_dv, img_double.depth(), dv, cvPoint(-1, -1));
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            img_angle.at<double>(i, j) = double(atan2(img_dv.at<double>(i, j), img_du.at<double>(i, j)));
            if (img_angle.at<double>(i, j) < 0) img_angle.at<double>(i, j) += M_PI;
            if (img_angle.at<double>(i, j) > M_PI) img_angle.at<double>(i, j) -= M_PI;
            img_weight.at<double>(i, j) = sqrt(img_dv.at<double>(i, j) * img_dv.at<double>(i, j) + img_du.at<double>(i, j) * img_du.at<double>(i, j));
        }
    }
    img.convertTo(img_double, CV_64F);
    double minVal, maxVal;
    int minIdx[2] = {}, maxIdx[2] = {};
    cv::minMaxIdx(img_double, &minVal, &maxVal, minIdx, maxIdx);
    img_double = (img_double - minVal) / (maxVal - minVal);
    cv::Mat cxy = img_double.clone();
    cv::Mat Ixy = img_double.clone();
    secondDerivCornerMetric(img_double, sigma, &cxy, &Ixy);
    struct Corner_t corners;

    cv::SimpleBlobDetector::Params params;
    params.filterByArea = false;
    params.filterByColor = false;
    params.minThreshold = 4;
    params.filterByConvexity = false;
    params.minConvexity = 0.7f;
    params.filterByCircularity = false;
    cv::Ptr<cv::SimpleBlobDetector> sbd = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> detectKeyPoint;
    cv::Mat a = cxy * 1000;
    a.convertTo(a, CV_8U);
    sbd->detect(a, detectKeyPoint);
    for (unsigned int i = 0; i < detectKeyPoint.size(); i++)
    {
        corners.p.push_back(detectKeyPoint[i].pt);
    }

    corners = getOrientations(img_angle, img_weight, corners, 5);
    std::vector<cv::Mat> chessboards = chessboardsFromCorners(corners);
    struct Corner_t new_corner;
    for (unsigned int j = 0; j < chessboards.size(); j++)
    {
        for (int u = 0; u < chessboards[j].rows; u++)
        {
            for (int v = 0; v < chessboards[j].cols; v++)
            {
                new_corner.p.push_back(corners.p[chessboards[j].at<uint16_t>(u, v)]);
                new_corner.v1.push_back(corners.v1[chessboards[j].at<uint16_t>(u, v)]);
                new_corner.v2.push_back(corners.v2[chessboards[j].at<uint16_t>(u, v)]);
            }
        }
    }
    new_corner = subPixelLocation(Ixy, new_corner);
    int cnt = 0;
    for (unsigned int j = 0; j < chessboards.size(); j++)
    {
        for (int u = 0; u < chessboards[j].rows; u++)
        {
            for (int v = 0; v < chessboards[j].cols; v++)
            {
                corners.p[chessboards[j].at<uint16_t>(u, v)] = new_corner.p[cnt];
                cnt++;
            }
        }
    }
    struct Chessboarder_t chess;
    chess.corners = corners;
    chess.chessboard = chessboards;
    return chess;
}

double normpdf(double x, int mu, int sigma)
{
    return double(exp(-(x - mu)*(x - mu) / 2 / sigma / sigma) / sqrt(2 * M_PI) / sigma);
}

struct Template_t createCorrelationPatch(double angle_1, double angle_2, int radius)
{
    struct Template_t result;
    int width = radius * 2 + 1;
    int height = radius * 2 + 1;

    result.a1 = cv::Mat::zeros(cv::Size(width, height), CV_64F);
    result.a2 = cv::Mat::zeros(cv::Size(width, height), CV_64F);
    result.b1 = cv::Mat::zeros(cv::Size(width, height), CV_64F);
    result.b2 = cv::Mat::zeros(cv::Size(width, height), CV_64F);

    int mu = radius;
    int mv = radius;

    for (int u = 0; u < width; u++)
    {
        for (int v = 0; v < height; v++)
        {
            int du = u - mu;
            int dv = v - mv;
            double dist = sqrt(du*du + dv*dv);
            double s1 = -du*sin(angle_1) + dv*cos(angle_1);
            double s2 = -du*sin(angle_2) + dv*cos(angle_2);
            if (s1 <= -0.1 && s2 <= -0.1)
                result.a1.at<double>(v, u) = normpdf(dist, 0, radius / 2);
            else if (s1 >= 0.1 && s2 >= 0.1)
                result.a2.at<double>(v, u) = normpdf(dist, 0, radius / 2);
            else if (s1 <= -0.1 && s2 >= 0.1)
                result.b1.at<double>(v, u) = normpdf(dist, 0, radius / 2);
            else if (s1 >= 0.1 && s2 <= -0.1)
                result.b2.at<double>(v, u) = normpdf(dist, 0, radius / 2);
        }
    }
    cv::normalize(result.a1, result.a1, 1, 0, cv::NORM_L1);
    cv::normalize(result.a2, result.a2, 1, 0, cv::NORM_L1);
    cv::normalize(result.b1, result.b1, 1, 0, cv::NORM_L1);
    cv::normalize(result.b2, result.b2, 1, 0, cv::NORM_L1);
    return result;
}

void secondDerivCornerMetric(cv::Mat img, int sigma, cv::Mat *cxy, cv::Mat *Ixy)
{
    cv::GaussianBlur(img, img, cv::Size(7 * sigma + 1, 7 * sigma + 1), sigma, sigma);
    cv::Mat Ix = img.clone();
    cv::Mat du = (cv::Mat_<double>(1, 3) << 1, 0, -1);
    cv::Mat dv = du.t();
    cv::filter2D(img, Ix, img.depth(), du, cvPoint(-1, -1));
    cv::filter2D(Ix, *Ixy, Ix.depth(), dv, cvPoint(-1, -1));

    int radius[3] = { 4, 8, 12 };
    cv::Mat template_props = (cv::Mat_<double>(6, 3) << 0, M_PI / 2, radius[0], -M_PI / 4, M_PI / 4, radius[0], 0, M_PI / 2, radius[1], -M_PI / 4, M_PI / 4, radius[1], 0, M_PI / 2, radius[2], -M_PI / 4, M_PI / 4, radius[2]);
    cv::Mat img_corner = cv::Mat::zeros(img.size(), CV_64FC1);
    for (int i = 0; i < template_props.rows; i++)
    {
        struct Template_t corner_template = createCorrelationPatch(template_props.at<double>(i, 0), template_props.at<double>(i, 1), int(template_props.at<double>(i, 2)));
        cv::Mat img_corners_a1, img_corners_a2, img_corners_b1, img_corners_b2, img_corners_mu;
        cv::filter2D(img, img_corners_a1, img.depth(), corner_template.a1, cvPoint(-1, -1));
        cv::filter2D(img, img_corners_a2, img.depth(), corner_template.a2, cvPoint(-1, -1));
        cv::filter2D(img, img_corners_b1, img.depth(), corner_template.b1, cvPoint(-1, -1));
        cv::filter2D(img, img_corners_b2, img.depth(), corner_template.b2, cvPoint(-1, -1));
        for (int j = 0; j < img.rows; j++)
        {
            for (int k = 0; k < img.cols; k++)
            {
                double a1 = img_corners_a1.at<double>(j, k);
                double a2 = img_corners_a2.at<double>(j, k);
                double b1 = img_corners_b1.at<double>(j, k);
                double b2 = img_corners_b2.at<double>(j, k);
                double mu = (a1 + a2 + b1 + b2) / 4;
                double img_corners_a = std::min(a1 - mu, a2 - mu);
                double img_corners_b = std::min(mu - b1, mu - b2);
                double img_corners_1 = std::min(img_corners_a, img_corners_b);

                img_corners_a = std::min(mu - a1, mu - a2);
                img_corners_b = std::min(b1 - mu, b2 - mu);
                double img_corners_2 = std::min(img_corners_a, img_corners_b);

                img_corner.at<double>(j, k) = std::max(img_corner.at<double>(j, k), img_corners_1);
                img_corner.at<double>(j, k) = std::max(img_corner.at<double>(j, k), img_corners_2);
            }
        }
    }
    *cxy = img_corner;
}

struct Corner_t getOrientations(cv::Mat img_angle, cv::Mat img_weight, struct Corner_t corners, int r)
{
    int height = img_angle.rows;
    int width = img_angle.cols;
    corners.score.resize(corners.p.size());
    for (unsigned int i = 0; i < corners.p.size(); i++)
    {
        int cu = corners.p[i].x;
        int cv = corners.p[i].y;
        int x1 = std::min(cv + r, height - 1);
        int x2 = std::max(cv - r, 0);
        int x3 = std::min(cu + r, width - 1);
        int x4 = std::max(cu - r, 0);
        cv::Mat img_angle_sub = cv::Mat::zeros(cv::Size(x3 - x4 + 1, x1 - x2 + 1), CV_64F);
        cv::Mat img_weight_sub = cv::Mat::zeros(cv::Size(x3 - x4 + 1, x1 - x2 + 1), CV_64F);
        for (int j = 0; j <= x1 - x2; j++)
        {
            for (int k = 0; k <= x3 - x4; k++)
            {
                img_angle_sub.at<double>(j, k) = img_angle.at<double>(j + x2, k + x4);
                img_weight_sub.at<double>(j, k) = img_weight.at<double>(j + x2, k + x4);
            }
        }
        cv::Point2d v1, v2;
        edgeOrientations(img_angle_sub, img_weight_sub, &v1, &v2);
        corners.v1.push_back(v1);
        corners.v2.push_back(v2);
    }
    return corners;
}

void edgeOrientations(cv::Mat img_angle, cv::Mat img_weight, cv::Point2d* v1, cv::Point2d* v2)
{
    int bin_num = 32;
    cv::Mat angle_hist = cv::Mat::zeros(cv::Size(bin_num, 1), CV_64F);
    for (int i = 0; i < img_angle.rows; i++)
    {
        for (int j = 0; j < img_angle.cols; j++)
        {
            img_angle.at<double>(i, j) += M_PI / 2;
            if (img_angle.at<double>(i, j) > M_PI) img_angle.at<double>(i, j) -= M_PI;
            int bin = std::max(std::min(int(floor(img_angle.at<double>(i, j) / (M_PI / bin_num))), bin_num - 1), 0);
            angle_hist.at<double>(0, bin) += img_weight.at<double>(i, j);
        }
    }
    std::vector<cv::Point3d> modes = findModesMeanShift(angle_hist, 1);
    if (modes.size() <= 1) return;
    for (unsigned int i = 0; i < modes.size(); i++)
    {
        modes[i].z = modes[i].x*M_PI / bin_num;
    }
    double delta_angle = 0;
    if (modes[0].z > modes[1].z)
    {
        *v1 = cv::Point2d(cos(modes[1].z), sin(modes[1].z));
        *v2 = cv::Point2d(cos(modes[0].z), sin(modes[0].z));
        delta_angle = std::min(modes[0].z - modes[1].z, modes[1].z + M_PI - modes[0].z);
    }
    else
    {
        *v1 = cv::Point2d(cos(modes[0].z), sin(modes[0].z));
        *v2 = cv::Point2d(cos(modes[1].z), sin(modes[1].z));
        delta_angle = std::min(modes[1].z - modes[0].z, modes[0].z + M_PI - modes[1].z);
    }
    if (delta_angle <= 0.3)
    {
        *v1 = cv::Point2d(0, 0);
        *v2 = cv::Point2d(0, 0);
    }
}

bool comp(const cv::Point3d &a, const cv::Point3d &b)
{
    return a.y > b.y;
}

std::vector<cv::Point3d> findModesMeanShift(cv::Mat hist, int sigma)
{
    std::vector<cv::Point3d> modes;
    cv::Mat hist_smoothed = hist.clone();
    for (int i = 0; i < hist.cols; i++)
    {
        double sum = 0;
        for (int j = -round(2 * sigma); j <= round(2 * sigma); j++)
        {
            int idx = fmod(i + j, hist.cols);
            sum += hist.at<double>(0, idx) * normpdf(j, 0, sigma);
        }
        hist_smoothed.at<double>(0, i) = sum;

    }
    bool failed = true;
    for (int i = 1; i < hist_smoothed.cols; i++)
    {
        if (fabs(hist_smoothed.at<double>(0, i) - hist_smoothed.at<double>(0, 0)) > 1e-5)
        {
            failed = false;
            break;
        }
    }
    if (failed == true)
        return modes;

    for (int i = 0; i < hist_smoothed.cols; i++)
    {
        int j = i;
        while (1)
        {
            double h0 = hist_smoothed.at<double>(0, j);
            int j1 = fmod(j + 1, hist_smoothed.cols);
            int j2 = fmod(j - 1, hist_smoothed.cols);
            double h1 = hist_smoothed.at<double>(0, j1);
            double h2 = hist_smoothed.at<double>(0, j2);
            if (h1 >= h0 && h1 >= h2)
                j = j1;
            else if (h2 > h0 && h2 > h1)
                j = j2;
            else
                break;
        }
        cv::Point3d mode = cv::Point3d(j, hist_smoothed.at<double>(0, j), 0);
        if (modes.size() == 0)
            modes.push_back(mode);
        else
        {
            bool find_same = false;
            for (unsigned int k = 0; k < modes.size(); k++)
            {
                if (modes[k].x == j)
                {
                    find_same = true;
                    break;
                }
            }
            if (find_same == false)
                modes.push_back(mode);
        }
    }
    sort(modes.begin(), modes.end(), comp);
    return modes;
}

struct Corner_t subPixelLocation(cv::Mat metric, struct Corner_t corners)
{
    cv::Mat X = (cv::Mat_<double>(6, 25) << 0.028571, 0.028571, 0.028571, 0.028571, 0.028571, -0.014286, -0.014286, -0.014286, -0.014286, -0.014286, -0.028571, -0.028571, -0.028571, -0.028571, -0.028571, -0.014286, -0.014286, -0.014286, -0.014286, -0.014286, 0.028571, 0.028571, 0.028571, 0.028571, 0.028571, 0.028571, -0.014286, -0.028571, -0.014286, 0.028571, 0.028571, -0.014286, -0.028571, -0.014286, 0.028571, 0.028571, -0.014286, -0.028571, -0.014286, 0.028571, 0.028571, -0.014286, -0.028571, -0.014286, 0.028571, 0.028571, -0.014286, -0.028571, -0.014286, 0.028571, -0.04, -0.04, -0.04, -0.04, -0.04, -0.02, -0.02, -0.02, -0.02, -0.02, 0, 0, 0, 0, 0, 0.02, 0.02, 0.02, 0.02, 0.02, 0.04, 0.04, 0.04, 0.04, 0.04, -0.04, -0.02, 0, 0.02, 0.04, -0.04, -0.02, 0, 0.02, 0.04, -0.04, -0.02, 0, 0.02, 0.04, -0.04, -0.02, 0, 0.02, 0.04, -0.04, -0.02, 0, 0.02, 0.04, 0.04, 0.02, 0, -0.02, -0.04, 0.02, 0.01, 0, -0.01, -0.02, 0, 0, 0, 0, 0, -0.02, -0.01, 0, 0.01, 0.02, -0.04, -0.02, 0, 0.02, 0.04, -0.074286, 0.011429, 0.04, 0.011429, -0.074286, 0.011429, 0.097143, 0.12571, 0.097143, 0.011429, 0.04, 0.12571, 0.15429, 0.12571, 0.04, 0.011429, 0.097143, 0.12571, 0.097143, 0.011429, -0.074286, 0.011429, 0.04, 0.011429, -0.074286);
    for (unsigned int i = 0; i < corners.p.size(); i++)
    {
        cv::Point2d p = corners.p[i];
        int halfPatchSize = 2;
        cv::Mat patch = cv::Mat::zeros(cv::Size(1, 25), CV_64F);
        int cnt = 0;
        for (int j = p.x - halfPatchSize; j <= p.x + halfPatchSize; j++)
        {
            for (int k = p.y - halfPatchSize; k <= p.y + halfPatchSize; k++)
            {
                patch.at<double>(cnt, 0) = metric.at<double>(k, j);
                cnt++;
            }
        }
        cv::Mat beta = X * patch;
        double A = beta.at<double>(0,0);
        double B = beta.at<double>(1, 0);
        double C = beta.at<double>(2, 0);
        double D = beta.at<double>(3, 0);
        double E = beta.at<double>(4, 0);
        double x = -(2 * B*C - D*E) / (4 * A*B - E*E);
        double y = -(2 * A*D - C*E) / (4 * A*B - E*E);
        if (fabs(x) > halfPatchSize || fabs(y) > halfPatchSize)
        {
            x = 0;
            y = 0;
        }
        corners.p[i].x += x;
        corners.p[i].y += y;
    }
    return corners;
}
