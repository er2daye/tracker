#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "Camera.h"
class Camera;
cv::Point2f pixel2cam(const cv::Point2f &pt, cv::Mat &K);
cv::Mat triangulation(const cv::Point2f &pt1, const cv::Point2f &pt2, Camera *c1, Camera *c2);
//cv::Mat triangulation(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, Camera *c1, Camera *c2);