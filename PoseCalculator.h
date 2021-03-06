#pragma once
#include "5point.h"
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
class PoseCalculator
{
public:
	PoseCalculator(void);
	~PoseCalculator(void);
	void getMatch(const Mat &img_1, const Mat &img_2);
	void computePose(const vector<Point2f> &inPoints1, const vector<Point2f> &inPoints2, const Mat &K, Mat &R, Mat &T);
	void computePoseFromImage(const cv::Mat &img_1, cv::Mat &img_2, const cv::Mat &K, cv::Mat &R, cv::Mat &t);

};

