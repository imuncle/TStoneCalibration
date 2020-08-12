#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "findCorner.h"
#include "chessboard.h"

int main()
{
	cv::Mat img = cv::imread("02.png");
	struct Corner_t corners = findCorner(img, 0.01, true);
	std::vector<cv::Mat> chessboards = chessboardsFromCorners(corners);
	plotChessboards(img, chessboards, corners);
	cv::imshow("final", img);
	cv::waitKey(0);
	return 0;
	std::vector<std::vector<cv::Point2d>> img_points, world_points;
	std::vector<cv::Mat> H;
	double chessboad_size = 28.33;
	for (int i = 0; i < 14; i++)
	{
		cv::Mat img = cv::imread(std::to_string(i)+".jpg");
		struct Corner_t corners = findCorner(img, 0.01, true);
		std::vector<cv::Mat> chessboards = chessboardsFromCorners(corners);
		plotChessboards(img, chessboards, corners);
		for (int j = 0; j < chessboards.size(); j++)
		{
			std::vector<cv::Point2d> img_p, world_p;
			for (int u = 0; u < chessboards[j].rows; u++)
			{
				for (int v = 0; v < chessboards[j].cols; v++)
				{
					img_p.push_back(corners.p[chessboards[j].at<uint16_t>(u, v)]);
					world_p.push_back(cv::Point2d(u*chessboad_size, v*chessboad_size));
				}
			}
			cv::Mat h = cv::findHomography(img_p, world_p, cv::RANSAC);
			H.push_back(h);
			img_points.push_back(img_p);
			world_points.push_back(world_p);
		}
		cv::imshow("final", img);
		cv::waitKey(0);
	}
	cv::Mat V = cv::Mat::zeros(cv::Size(6, 2*H.size()), CV_64F);
	for (int i = 0; i < H.size(); i++)
	{
		V.at<double>(2 * i, 0) = H[i].at<double>(0, 0) * H[i].at<double>(0, 1);
		V.at<double>(2 * i, 1) = H[i].at<double>(0, 0) * H[i].at<double>(1, 1) + H[i].at<double>(1, 0) * H[i].at<double>(0, 1);
		V.at<double>(2 * i, 2) = H[i].at<double>(1, 0) * H[i].at<double>(1, 1);
		V.at<double>(2 * i, 3) = H[i].at<double>(2, 0) * H[i].at<double>(0, 1) + H[i].at<double>(0, 0) * H[i].at<double>(2, 1);
		V.at<double>(2 * i, 4) = H[i].at<double>(2, 0) * H[i].at<double>(1, 1) + H[i].at<double>(1, 0) * H[i].at<double>(2, 1);
		V.at<double>(2 * i, 5) = H[i].at<double>(2, 0) * H[i].at<double>(2, 1);

		V.at<double>(2 * i + 1, 0) = H[i].at<double>(0, 0) * H[i].at<double>(0, 0) - H[i].at<double>(0, 1) * H[i].at<double>(0, 1);
		V.at<double>(2 * i + 1, 0) = H[i].at<double>(0, 0) * H[i].at<double>(1, 0) + H[i].at<double>(1, 0) * H[i].at<double>(0, 0) - H[i].at<double>(0, 1) * H[i].at<double>(1, 1) - H[i].at<double>(1, 1) * H[i].at<double>(0, 1);
		V.at<double>(2 * i + 1, 0) = H[i].at<double>(1, 0) * H[i].at<double>(1, 0) - H[i].at<double>(1, 1) * H[i].at<double>(1, 1);
		V.at<double>(2 * i + 1, 0) = H[i].at<double>(2, 0) * H[i].at<double>(0, 0) + H[i].at<double>(0, 0) * H[i].at<double>(2, 0) - H[i].at<double>(2, 1) * H[i].at<double>(0, 1) - H[i].at<double>(0, 1) * H[i].at<double>(2, 1);
		V.at<double>(2 * i + 1, 0) = H[i].at<double>(2, 0) * H[i].at<double>(1, 0) + H[i].at<double>(1, 0) * H[i].at<double>(2, 0) - H[i].at<double>(2, 1) * H[i].at<double>(1, 1) - H[i].at<double>(1, 1) * H[i].at<double>(2, 1);
		V.at<double>(2 * i + 1, 0) = H[i].at<double>(2, 0) * H[i].at<double>(2, 0) - H[i].at<double>(2, 1) * H[i].at<double>(2, 1);
	}
	cv::Mat v;
	cv::transpose(V, v);
	v = v*V;
	cv::Mat w, u, vt;
	cv::SVD::compute(v, w, u, vt);
	cv::transpose(vt, vt);
	double v0 = (vt.at<double>(1, 5)*vt.at<double>(3, 5) - vt.at<double>(0, 5)*vt.at<double>(4, 5)) / (vt.at<double>(0, 5) * vt.at<double>(2, 5) - vt.at<double>(1, 5)*vt.at<double>(1, 5));
	double s = vt.at<double>(5, 5) - (vt.at<double>(3, 5)*vt.at<double>(3, 5) + v0*(vt.at<double>(1, 5)*vt.at<double>(3, 5) - vt.at<double>(0, 5)*vt.at<double>(4, 5))) / vt.at<double>(0, 5);
	double alpha_u = sqrt(s / vt.at<double>(0, 5));
	double alpha_v = sqrt(s*vt.at<double>(0, 5) / (vt.at<double>(0, 5)*vt.at<double>(2, 5) - vt.at<double>(1, 5)*vt.at<double>(1, 5)));
	double skew = -vt.at<double>(1, 5)*alpha_u*alpha_u*alpha_v / s;
	double u0 = skew*v0 / alpha_u - vt.at<double>(3, 5)*alpha_u*alpha_u / s;
	cv::Mat A = cv::Mat::zeros(cv::Size(3,3), CV_64F);
	A.at<double>(0, 0) = alpha_u;
	A.at<double>(0, 1) = skew;
	A.at<double>(0, 2) = u0;
	A.at<double>(1, 1) = alpha_v;
	A.at<double>(1, 2) = v0;
	A.at<double>(2, 2) = 1;
	std::cout << A << std::endl;
	cv::waitKey(0);
	return 0;
}