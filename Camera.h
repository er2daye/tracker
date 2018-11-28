#pragma once
#include "CnComm.h"
#include "parameter.h"
#include "CvvImage.h"
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
	CnComm m_com;
	VideoCapture cap;
	void Open();
	void Close();
	void Reset();
	void Send(string data);
	void Recive();
	void MoveToAbsolutePosition(double x, double y, double z, double speed, int model);
    void MoveToPolarPosition(double r, double s, double h, double speed, int model);
	void CapturePicture();
	
};

