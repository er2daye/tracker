#include "stdafx.h"
#include "FCamera.h"


FCamera::FCamera(void)
{
}


FCamera::~FCamera(void)
{
}

FCamera::FCamera(int IDC_CAMERA_SHOW_, int height_, int width_) {
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
	K.at<double>(0, 0) = 1433.715;
	K.at<double>(1, 1) = 1432.291;
	K.at<double>(0, 2) = 647.95;
	K.at<double>(1, 2) = 347.36;
	K.at<double>(2, 2) = 1;
	//0.994572, -0.0687067, 0.0765522, -0.0156602
	//-0.218924, -3.18991, -3.07441
	input_t = "-0.379886, -3.43864, -3.63572";
	input_q = "0.995834, -0.0815083, 0.0398277, -0.00915502";


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

void FCamera::OpenPlatform() {
	return;
}

void FCamera::CameraMove() {
	return;
}
void FCamera::CameraMoveFromReal3D(cv::Mat pt3d) {
	return;
}
cv::Mat FCamera::GetFrame() {
	cv::Mat frame;
	cap >> frame;
	return frame;
}

void FCamera::Open() {
	cap.open(0);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280.0);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720.0);
}
void FCamera::Move(double deg) {
	return;
}
	
void FCamera::Reset() {
	return;
}