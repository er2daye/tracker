#pragma once
#include "camera.h"
class FCamera :
	public Camera
{
public:
	FCamera(void);
	FCamera(int IDC_CAMERA_SHOW_, int height_, int width_);
	~FCamera(void);
	void OpenPlatform();
	void CameraMove();
	void CameraMoveFromReal3D(cv::Mat pt3d);
	Mat GetFrame();
	void Move(double deg);
	void Reset();
	void Open();

	VideoCapture cap;
};

