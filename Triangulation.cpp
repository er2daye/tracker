#include "stdafx.h"
#include "Triangulation.h"

cv::Point2f pixel2cam(const cv::Point2f &pt, cv::Mat &K) {
	float fx = K.at<float>(0, 0);
	float fy = K.at<float>(1, 1);
	float cx = K.at<float>(0, 2);
	float cy = K.at<float>(1, 2);

	float x = pt.x;
	float y = pt.y;

	return cv::Point2f(1/fx*(x - cx), 1/ fy*(y-cy));
}

cv::Mat triangulation(const cv::Point2f &pt1, const cv::Point2f &pt2, Camera *c1, Camera *c2) {
	cv::Mat K1 = c1->K;
	cv::Mat K2 = c2->K;
	cv::Mat T1 = c1->T;
	cv::Mat T2 = c2->T;
	cv::Point2f p2d1 = pixel2cam(pt1, K1);
	cv::Point2f p2d2 = pixel2cam(pt2, K1);
	cv::Mat p2ds1(2, 1, CV_64F);
	p2ds1.at<float>(0, 0) = p2d1.x;
	p2ds1.at<float>(1, 0) = p2d1.y;
	cv::Mat p2ds2(2, 1, CV_64F);
	p2ds2.at<float>(0, 0) = p2d2.x;
	p2ds2.at<float>(1, 0) = p2d2.y;
	cv::Mat p4d;
	cv::triangulatePoints(T1.rowRange(0, 3), T2.colRange(0, 3), p2ds1, p2ds2, p4d);
	return p4d;
}