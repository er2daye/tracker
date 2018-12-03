#include "stdafx.h"
#include "Triangulation.h"

cv::Point2f pixel2cam(const cv::Point2f &pt, cv::Mat &K) {
	float fx = K.at<double>(0, 0);
	float fy = K.at<double>(1, 1);
	float cx = K.at<double>(0, 2);
	float cy = K.at<double>(1, 2);

	float x = pt.x;
	float y = pt.y;

	return cv::Point2f(1/fx*(x - cx), 1/ fy*(y-cy));
}

cv::Mat triangulation(const cv::Point2f &pt1, const cv::Point2f &pt2, Camera *c1, Camera *c2) {
	cv::Mat K1 = c1->K;
	cv::Mat K2 = c2->K;
	cv::Mat T1 = c1->T;
	cv::Mat T2 = c2->T;
	TRACE("pose:\n");
	for (int i = 0; i < 4; i++) {
		TRACE("%.5f %.5f %.5f %.5f\n", T1.at<double>(i, 0), T1.at<double>(i, 1), T1.at<double>(i, 2), T1.at<double>(i, 3));
	}
	for (int i = 0; i < 4; i++) {
		TRACE("%.5f %.5f %.5f %.5f\n", T2.at<double>(i, 0), T2.at<double>(i, 1), T2.at<double>(i, 2), T2.at<double>(i, 3));
	}
	cv::Point2f p2d1 = pixel2cam(pt1, K1);
	cv::Point2f p2d2 = pixel2cam(pt2, K2);
	cv::Mat p2ds1(2, 1, CV_64F);
	p2ds1.at<double>(0, 0) = p2d1.x;
	p2ds1.at<double>(1, 0) = p2d1.y;
	cv::Mat p2ds2(2, 1, CV_64F);
	p2ds2.at<double>(0, 0) = p2d2.x;
	p2ds2.at<double>(1, 0) = p2d2.y;
	/*-------------------------test begin-------------------*/
	cv::Mat p1(3, 1, CV_64F);
	cv::Mat T21 = T2 * T1.inv();
	cv::Mat T12 = T1 * T2.inv();
	cv::Mat t1 = T21.rowRange(0, 3).col(3);
	p1.at<double>(0) = p2d1.x;
	p1.at<double>(1) = p2d1.y;
	p1.at<double>(2) = 1;
	p1 = T21.rowRange(0, 3).colRange(0, 3) * p1;
	double d11 = (p2d2.x * t1.at<double>(2) - t1.at<double>(0)) / (p1.at<double>(0) - p1.at<double>(2) * p2d2.x);
	double d12 = (p2d2.y * t1.at<double>(2) - t1.at<double>(1)) / (p1.at<double>(1) - p1.at<double>(2) * p2d2.y);
	double d1 = (d11 + d12) / 2;
	p1 = d1 * p1 + t1;
	TRACE("depth 1 : %f %f %f\n", d11, d12, d1);
	TRACE("point 1 : %f %f\n", p1.at<double>(0) / p1.at<double>(2), p1.at<double>(1) / p1.at<double>(2));
	cv::Mat p2(3, 1, CV_64F);
	cv::Mat t2 = T12.rowRange(0, 3).col(3);
	p2.at<double>(0) = p2d2.x;
	p2.at<double>(1) = p2d2.y;
	p2.at<double>(2) = 1;
	p2 = T12.rowRange(0, 3).colRange(0, 3) * p2;
	double d21 = (p2d1.x * t2.at<double>(2) - t2.at<double>(0)) / (p2.at<double>(0) - p2.at<double>(2) * p2d1.x);
	double d22 = (p2d1.y * t2.at<double>(2) - t2.at<double>(1)) / (p2.at<double>(1) - p2.at<double>(2) * p2d1.y);
	double d2 = (d21 + d22) / 2;
	TRACE("depth 2 : %f\n", d2);
	TRACE("point 2 : %f %f\n", p2.at<double>(0) / p2.at<double>(2), p2.at<double>(1) / p2.at<double>(2));
	/*-------------------------test end-------------------*/
	cv::Mat p4d;
	cv::triangulatePoints(T1.rowRange(0, 3), T2.rowRange(0, 3), p2ds1, p2ds2, p4d);
	cv::Mat ta = T1 * p4d;
	cv::Mat a = K1 * ta.rowRange(0, 3);
	cv::Mat tb = T2 * p4d;
	cv::Mat b = K2 * tb.rowRange(0, 3);
	double da = a.at<double>(2);
	double db = b.at<double>(2);
	TRACE("p4f four : %f\n", p4d.at<double>(3));
	TRACE("%.5f %.5f %.5f\n", a.at<double>(0) / da, a.at<double>(1) / da, a.at<double>(2) / da);
	TRACE("%.5f %.5f %.5f\n", b.at<double>(0) / db, b.at<double>(1) / db, b.at<double>(2) / db);
	return p4d;
}