#pragma once
#include "camera.h"
class HCamera :
	public Camera
{
public:
	HCamera(void);
	HCamera(int IDC_CAMERA_SHOW_, int height_, int width_, CnComm *com = NULL);
	~HCamera(void);
	void CameraMove();
	void CameraMoveFromReal3D(cv::Mat pt3d);
	Mat GetFrame();
	void Move(double deg);
	void Reset();
	void Open();
};

