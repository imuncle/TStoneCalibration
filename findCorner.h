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
	std::vector<cv::Point2f> p;
	std::vector<cv::Point2f> v1;
	std::vector<cv::Point2f> v2;
	std::vector<float> score;
};

int sign(double x);
struct Corner_t findCorner(cv::Mat img, double tau, bool refine);
struct Template_t createCorrelationPatch(float angle_1, float angle_2, int radius);
float normpdf(double x, int mu, int sigma);
std::vector<cv::Point2f> nonMaximumSuppression(cv::Mat img, int n, float tau, int margin);
struct Corner_t refineCorners(cv::Mat img_du, cv::Mat img_dv, cv::Mat img_angle, cv::Mat img_weight, struct Corner_t corners, int r);
void edgeOrientations(cv::Mat img_angle, cv::Mat img_weight, cv::Point2f* v1, cv::Point2f* v2);
std::vector<cv::Point3f> findModesMeanShift(cv::Mat hist, int sigma);
struct Corner_t scoreCorners(cv::Mat img, cv::Mat img_weight, struct Corner_t corners, int* radius);
float cornerCorrelationScore(cv::Mat img, cv::Mat img_weight, cv::Point2f v1, cv::Point2f v2);

#endif