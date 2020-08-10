#include <opencv2/opencv.hpp>
#include "findCorner.h"
#include "chessboard.h"

int main()
{
	cv::Mat img = cv::imread("02.png");
	struct Corner_t corners = findCorner(img, 0.01, true);
	std::vector<cv::Mat> chessboards = chessboardsFromCorners(corners);
	for (int i = 0; i < corners.p.size(); i++)
	{
		cv::circle(img, corners.p[i], 10, cv::Scalar(100, 100, 100));
	}
	plotChessboards(img, chessboards, corners);
	cv::imshow("final", img);
	cv::waitKey(0);
	return 0;
}