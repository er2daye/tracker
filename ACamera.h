#pragma once
#include "camera.h"
class ACamera :
	public Camera
{
public:
	ACamera(void);
	ACamera(int IDC_CAMERA_SHOW_, int height_, int width_, CnComm *com = NULL);
	~ACamera(void);
	void CameraMove();
	void CameraMoveFromReal3D(cv::Mat pt3d);
	Mat GetFrame();
	void Move(double deg);
	void Reset();
	void Open();
	void OpenPlatform();

	bool isopen;
	VideoCapture cap;
	CnComm *m_com;
	void OpenArm();
	void Close();
	void Send(string data);
	void Recive();
	void MoveToAbsolutePosition(double x, double y, double z, double speed, int model);
    void MoveToPolarPosition(double r, double s, double h, double speed, int model);

	thread *p_recive;

};

