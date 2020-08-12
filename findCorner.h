#ifndef FIND_CORNER_H
#define FIND_CORNER_H

#include <opencv2/opencv.hpp>

#define M_PI 3.141592653f

struct Template_t
{
	cv::Mat a1;
	cv::Mat a2;
	cv::Mat b1;
	cv::Mat b2;
};

struct Corner_t
{
	std::vector<cv::Point2d> p;
	std::vector<cv::Point2d> v1;
	std::vector<cv::Point2d> v2;
	std::vector<double> score;
};

int sign(double x);
struct Corner_t findCorner(cv::Mat img, double tau, bool refine);
struct Template_t createCorrelationPatch(double angle_1, double angle_2, int radius);
double normpdf(double x, int mu, int sigma);
std::vector<cv::Point2d> nonMaximumSuppression(cv::Mat img, int n, double tau, int margin);
struct Corner_t getOrientations(cv::Mat img_angle, cv::Mat img_weight, struct Corner_t corners, int r);
struct Corner_t refineCorners(cv::Mat img_du, cv::Mat img_dv, cv::Mat img_angle, cv::Mat img_weight, struct Corner_t corners, int r);
void edgeOrientations(cv::Mat img_angle, cv::Mat img_weight, cv::Point2d* v1, cv::Point2d* v2);
std::vector<cv::Point3d> findModesMeanShift(cv::Mat hist, int sigma);
struct Corner_t scoreCorners(cv::Mat img, cv::Mat img_weight, struct Corner_t corners, int* radius, double tau);
double cornerCorrelationScore(cv::Mat img, cv::Mat img_weight, cv::Point2d v1, cv::Point2d v2);

#endif