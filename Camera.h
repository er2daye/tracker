#pragma once
#include "CnComm.h"
#include "parameter.h"
#include "CvvImage.h"
#include "stdafx.h"
#include "kcf\kcftracker.h"
#include "GeneralDef.h"
#include "PTZButton.h"
#include "Triangulation.h"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <Eigen/Dense>
using namespace cv;
using namespace std;
class Camera
{
public:
	//int idx;
	Camera(void);
	//Camera(int idx_, int IDC_CAMERA_SHOW_, int height_, int width_, CnComm *com = NULL);
	~Camera(void);
	Mat frame;
	bool isTracking;
	bool bBoxInit;
	bool drawBox;
	cv::Rect boundingBox;
	cv::Rect preBoundingBox;
	cv::Point2f centerOfBox;
    cv::Point2f centerOfImage;
	CPoint topCornerOfShowBox;
	CPoint bottomCornerOfShowBox;
	double widthOfScreen;
	double heightOfScreen;
	int IDC_CAMERA_SHOW;
	int mouseState;

	// track : update boundingBox, centerofBox
	KCFTracker tracker;
	void TrackerInit();
	bool TrackFromImage(Mat nowFrame);
	void TrackFromReal3D(const cv::Mat &pt3d);

	// Set Real3d From triangulation
	// Update real3d, offset, depth
	void SetReal3D(const cv::Mat &pt);
	// Get Real3d from estimate by boundingbox
	cv::Mat GetReal3D();
	bool move;
	// camera in*** parameters 3*3
	cv::Mat K;
	// camera pose 4*4
	cv::Mat T;
	// camera delta rotate 3*3
	cv::Mat R;
	// center of object in real world
	cv::Mat real3d;
	// offset of real3d from triangulation and image point
	cv::Point2f offset;
	double depth;

	float angleAlly;
	vector<pair<double, double> > path;

	//0: not tracking 1: already to tracking 2: tracking 3:lost but in 4:lost
	int state;
	void UpdateFrame(const cv::Mat &image);
	virtual void CameraMove() = 0;
	virtual void OpenPlatform() = 0;

	virtual cv::Mat GetFrame() = 0;
	virtual void Move(double deg) = 0;
	virtual void Reset() = 0;
	virtual void Open() = 0;

	/*
	//camera2
	bool isopen;
	VideoCapture cap;
	CnComm *m_com;
	Mat GetFrame();
	void OpenArm();
	void Open();
	void Close();
	void Reset();
	void Send(string data);
	void Recive();
	void MoveToAbsolutePosition(double x, double y, double z, double speed, int model);
    void MoveToPolarPosition(double r, double s, double h, double speed, int model);
	void StopMovement();
	*/
};

