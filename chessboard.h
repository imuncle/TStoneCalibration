#ifndef CHESSBOARD_H
#define CHESSBOARD_H

#include "findCorner.h"

std::vector<cv::Mat> chessboardsFromCorners(struct Corner_t corners);
cv::Mat initChessboard(struct Corner_t corners, int idx);
float average(float* a, int length);
float stdd(float* a, int length, float mean);
void directionalNeighbor(int idx, cv::Point2f v, cv::Mat chessboard, struct Corner_t corners, int* neighbor_idx, float* min_dist);
float chessboardEnergy(cv::Mat chessboard, struct Corner_t corners);
cv::Mat growChessboard(cv::Mat chessboard, struct Corner_t corners, int boarder_type);
cv::Point2f predictCorners(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);
std::vector<int> assignClosestCorners(std::vector<cv::Point2f> cand, std::vector<cv::Point2f> pred);
void plotChessboards(cv::Mat img, std::vector<cv::Mat> chessboards, struct Corner_t corners);

#endif