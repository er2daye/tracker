#include "stdafx.h"
#include "HCamera.h"


HCamera::HCamera(void)
{
}


HCamera::~HCamera(void)
{
}

HCamera::HCamera(int IDC_CAMERA_SHOW_, int height_, int width_, CnComm *com) {
	IDC_CAMERA_SHOW = IDC_CAMERA_SHOW_;
	heightOfScreen = height_;
	widthOfScreen = width_;
	state = 0;
	move = true;
	real3d = cv::Mat(3, 1, CV_64F);
	for (int i = 0; i < 3; i++)
		real3d.at<double>(i) = -INF_DOUBLE;

	double qw = 0, qx = 0, qy = 0,  qz = 0;
	double tx = 0, ty= 0 , tz = 0;
	string input_t, input_q;

	K = cv::Mat::zeros(3, 3, CV_64F);
	K.at<double>(0, 0) = 1419.743;
	K.at<double>(1, 1) = 1419.660;
	K.at<double>(0, 2) = 656.47;
	K.at<double>(1, 2) = 337.67;
	K.at<double>(2, 2) = 1;


	sscanf(input_t.c_str(), "%lf,%lf,%lf", &tx, &ty, &tz);
	sscanf(input_q.c_str(), "%lf,%lf,%lf,%lf", &qw, &qx, &qy, &qz);
	Eigen::Quaterniond qeigen = Eigen::Quaterniond(qw, qx, qy, qz);
	Eigen::Matrix3d Reigen = qeigen.toRotationMatrix();
	T = cv::Mat::zeros(4, 4, CV_64F);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			T.at<double>(i, j) = Reigen(i, j);
		}
	}
	T.at<double>(0, 3) = tx;
	T.at<double>(1, 3) = ty;
	T.at<double>(2, 3) = tz;
	T.at<double>(3, 3) = 1;
	TrackerInit();
	mouseState = 0;
	drawBox = false;
	topCornerOfShowBox = CPoint(0, 0);
	bottomCornerOfShowBox = CPoint(0, 0);
	angleAlly = 0;

}

void HCamera::CameraMove() {
	return;
}
void HCamera::CameraMoveFromReal3D(cv::Mat pt3d) {
	return;
}
cv::Mat HCamera::GetFrame() {
	cv::Mat frame;
	NET_DVR_CapturePictureBlock(0, "./capture/1.jpg", 100);
	frame = imread("./capture/1.jpg");
	return frame;
}

void HCamera::Move(double deg) {
	return;
}
void HCamera::Reset() {
	return;
}
void HCamera::Open() {
	return;
}