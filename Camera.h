#pragma once
#include "CnComm.h"
#include "parameter.h"
#include "CvvImage.h"
#include "stdafx.h"
#include "kcf\kcftracker.h"
#include "GeneralDef.h"
#include "PTZButton.h"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;
class Camera
{
public:
	Camera(void);
	~Camera(void);
	Mat frame;
	bool isTracking;
	bool bBoxInit;
	bool drawBox;
	cv::Rect boundingBox;
	cv::Point2f centerOfBox;
    cv::Point2f centerOfWindow;
	CPoint topCornerOfShowBox;
	CPoint bottomCornerOfShowBox;
	double widthOfScreen;
	double heightOfScreen;
	int IDC_CAMERA_SHOW;
	int mouseState;
	KCFTracker tracker;
	void CapturePicture(int cameraModel);
	void TrackerInit();
	void TrackFromImage(Mat nowFrame);
	void CameraMove(int cameraModel);
	
	//camera2
	VideoCapture cap;
	CnComm m_com;
	void Open();
	void Close();
	void Reset();
	void Send(string data);
	void Recive();
	void MoveToAbsolutePosition(double x, double y, double z, double speed, int model);
    void MoveToPolarPosition(double r, double s, double h, double speed, int model);
	

	

	
};

