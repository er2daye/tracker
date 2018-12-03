#include "stdafx.h"
#include "Camera.h"
#include <Eigen/Dense>


Camera::Camera(void)
{
	state = 0;
	move = true;
	TrackerInit();
	mouseState = 0;
	drawBox = false;
	topCornerOfShowBox = CPoint(0, 0);
	bottomCornerOfShowBox = CPoint(0, 0);
	m_com = NULL;
	angleAlly = 0;
	real3d = cv::Mat(3, 1, CV_64F);
	for (int i = 0; i < 3; i++)
		real3d.at<double>(i) = -INF_DOUBLE;
}

Camera::Camera(int idx_, int IDC_CAMERA_SHOW_, int height_, int width_, CnComm *com) 
	: idx(idx_), IDC_CAMERA_SHOW(IDC_CAMERA_SHOW_), heightOfScreen(height_), widthOfScreen(width_), m_com(com), state (0) {
	move = true;
	real3d = cv::Mat(3, 1, CV_64F);
	for (int i = 0; i < 3; i++)
		real3d.at<double>(i) = -INF_DOUBLE;

	if (idx == 1) {
		K = cv::Mat::zeros(3, 3, CV_64F);
		K.at<double>(0, 0) = 1419.743;
		K.at<double>(1, 1) = 1419.660;
		K.at<double>(0, 2) = 656.47;
		K.at<double>(1, 2) = 337.67;
		K.at<double>(2, 2) = 1;
	}
	double qw = 0, qx = 0, qy = 0,  qz = 0;
	double tx = 0, ty= 0 , tz = 0;
	string input_t, input_q;
	if (idx == 2) {
		K = cv::Mat::zeros(3, 3, CV_64F);
		K.at<double>(0, 0) = 1433.715;
		K.at<double>(1, 1) = 1432.291;
		K.at<double>(0, 2) = 647.95;
		K.at<double>(1, 2) = 347.36;
		K.at<double>(2, 2) = 1;
		//0.99663, 0.0818709, 0.00111595, -0.00485828
		//0.99663, 0.0818709, 0.00111595, -0.00485828
		//-0.800911, 2.1634, 4.9686
		input_t = "-0.520842, 1.70941, 4.49782";
		input_q = "0.999804, -0.0097383, 0.0148609, 0.0087686";
	}
	if (idx == 3) {
		K = cv::Mat::zeros(3, 3, CV_64F);
		K.at<double>(0, 0) = 1433.715;
		K.at<double>(1, 1) = 1432.291;
		K.at<double>(0, 2) = 647.95;
		K.at<double>(1, 2) = 347.36;
		K.at<double>(2, 2) = 1;
		//0.994572, -0.0687067, 0.0765522, -0.0156602
		//-0.218924, -3.18991, -3.07441
		input_t = "-0.29141, -3.26278, -3.7479";
		input_q = "0.98468, -0.145097, 0.0966935, 0.00160772";
	}
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

Camera::~Camera(void)
{
}

void Camera::TrackerInit() {
	bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = false;
    bool LAB = false;

    tracker = KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	isTracking = false;
	boundingBox = cv::Rect(-1, -1, -1, -1);
	preBoundingBox = cv::Rect(-1, -1, -1, -1);
	bBoxInit = false;

}

/*
void Camera::CapturePicture(int cameraModel) {
	if(cameraModel == 1) {
		
	}
	if(cameraModel == 2) {
		Mat frame;
		cap >> frame;
		imwrite("./capture/2.jpg", frame);
	}
	
    
}
*/
void Camera::OpenArm() {
	if(m_com->IsOpen()){
		m_com->Close();
	}
	if (!m_com->Open(comId, 115200, NOPARITY, 8, ONESTOPBIT)) {
			MessageBox(NULL, "Fail", "connect", MB_OK);
	}
}

void Camera::Open(){
		if (idx == 2) cap.open(1);
		else if (idx == 3) cap.open(0);
		cap.set(CAP_PROP_FRAME_WIDTH, 1280.0);
		cap.set(CAP_PROP_FRAME_HEIGHT, 720.0);
}

void Camera::UpdateFrame(const cv::Mat &image) {
	image.copyTo(frame);
}

cv::Mat Camera::GetFrame() {
	cv::Mat frame;
	if (idx != 1) {
		cap >> frame;
	}
	else {
		NET_DVR_CapturePictureBlock(0, "./capture/1.jpg", 100);
		frame = imread("./capture/1.jpg");
	}
	return frame;
}


void Camera::Reset(){
	MoveToAbsolutePosition(110, 0, 40, ARM_SPEED, 1);
}
void Camera::Close(){
	if(m_com && m_com->IsOpen()){
		m_com->Close();
	}
}

void Camera::Send(string data){
	if(!m_com || !m_com->IsOpen()) return;
	const char *buf = data.c_str();
	m_com->Write(buf); 
}
void Camera::MoveToAbsolutePosition(double x, double y, double z, double speed, int model) {
    if (!m_com || !m_com->IsOpen()) {
		MessageBox(NULL, "ERROR","connect failed", MB_OK);
        return;
    }
    string code;
    if (model == 1) {
        code = "G0 X" + to_string(x)+ " Y" + to_string(y) + " Z" + to_string(z) + " F" + to_string(speed) + "\n";
    } else if (model == 2) {
        code = "G2204 X" + to_string(x)+ " Y" + to_string(y) + " Z" + to_string(z) + " F" + to_string(speed) + "\n";
    }
    Send(code);
}

void Camera::MoveToPolarPosition(double r, double s, double h, double speed, int model) {
    if (!m_com || !m_com->IsOpen()) {
		MessageBox(NULL, "ERROR","connect failed", MB_OK);
        return;
    }
    string code;
    if (model == 1) {
        code = "G2201 S" + to_string(s)+ " R" + to_string(r) + " H" + to_string(h) + " F" + to_string(speed) + "\n";
    } else if (model == 2) {
        code = "G2205 S" + to_string(s)+ " R" + to_string(r) + " H" + to_string(h) + " F" + to_string(speed) + "\n";
    }
    Send(code);
}

void Camera::StopMovement() {
    if (!m_com || !m_com->IsOpen()) {
		MessageBox(NULL, "ERROR","connect failed", MB_OK);
        return;
    }
    string code;
	code = "V0\n";
    Send(code);
} 



bool Camera::TrackFromImage(Mat nowFrame) {
	bool isok;
    if (!bBoxInit) {
        tracker.init(boundingBox, nowFrame);
        bBoxInit = true;
		isok = true;
    } else {
		float value;
        cv::Rect currentBox = tracker.update(nowFrame, value, isok);
		TRACE("Here ------------ kcf value : %f\n", value);
		if (!isok) {
			return false;
		}

		boundingBox = currentBox;
    }


	centerOfImage = cv::Point2f(frame.cols / 2, frame.rows / 2);
	centerOfBox = cv::Point2f(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
	return true;
}

void Camera::TrackFromReal3D(const cv::Mat &pt3d) {
	//without scale
	cv::Mat pt = T.rowRange(0, 3).colRange(0, 3) * pt3d + T.rowRange(0, 3).col(3);
	cv::Mat ptimage = K * pt;
	double depth = ptimage.at<double>(2);
	centerOfBox = cv::Point2f(ptimage.at<double>(0) / depth, ptimage.at<double>(1) / depth) - offset;
	boundingBox.x = centerOfBox.x - boundingBox.width / 2;
	boundingBox.y = centerOfBox.y - boundingBox.height / 2;
	tracker._roi = boundingBox;

	return;
}

void Camera::CameraMove(int cameraModel) {
	cv::Point2f object2d = pixel2cam(centerOfBox, K);
	cv::Point2f window2d = pixel2cam(centerOfImage, K);
	float lobj = sqrt(object2d.x * object2d.x + 1);
	float lwin = sqrt(window2d.x * window2d.x + 1);
	float cross = (object2d.x * window2d.x + 1) / lobj / lwin;
	float angley = acosf(cross);
	if (object2d.x > 0) angley *= -1;
	TRACE("object in image2d %f %f\n", centerOfBox.x, centerOfBox.y);
	TRACE("image2d %f %f\n", centerOfImage.x, centerOfImage.y);
	TRACE("object2d %f %f\n", object2d.x, object2d.y);
	TRACE("angle %f\n", angley);
	TRACE("angle degree %f\n", angley / acos(-1) * 180);
	angley = angley / acos(-1) * 180;
	double tdeg = angleAlly + angley;
	double tdis = 1.0 * (preBoundingBox.width + preBoundingBox.height) / (boundingBox.width + boundingBox.height);
	path.push_back(make_pair(tdeg / 180 * acos(-1), tdis));

	//if you don't want to move
	//move = false;

	if (idx == 2 && move) {
		//9 / 60 * 30 = 4.5
		if (fabs(angley) < 0.5) {
			TRACE("angle all %f\n", angleAlly);
			return;
		}
		if (fabs(angley) < 2) move = !move;
		if (fabs(angley) > 0.6) angley = angley / fabs(angley) * 0.6;
		MoveToPolarPosition(angley, 0, 0, 12000, 2);
		angleAlly += angley;
		Eigen::Matrix3d angleMatrix = Eigen::AngleAxisd(angley / 180 * acos(-1), Eigen::Vector3d::UnitY()).matrix();
		cv::Mat Rnow = cv::Mat::eye(4, 4, CV_64F);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) Rnow.at<double>(i, j) = angleMatrix(i, j);
		}
		T *= Rnow;
	}
	else if (!move) {
		move = !move;
	}
	TRACE("angle all %f\n", angleAlly);
	
	//move = !move;
	puts("test");
}

void Camera::SetReal3D(const cv::Mat &pt) {
	real3d = T.rowRange(0, 3).colRange(0, 3) * pt + T.rowRange(0, 3).col(3);
	preBoundingBox = boundingBox;
	depth = real3d.at<double>(2);
	cv::Mat proj3d = K * real3d;
	cv::Point2f proj2d = cv::Point2f(proj3d.at<double>(0) / proj3d.at<double>(2), proj3d.at<double>(1) / proj3d.at<double>(2));
	offset = proj2d - centerOfBox;
}

cv::Mat Camera::GetReal3D() {
	double tdis = 1.0 * (preBoundingBox.width + preBoundingBox.height) / (boundingBox.width + boundingBox.height);
	double nowdepth = tdis * depth;
	cv::Point2f object2d = pixel2cam(centerOfBox + offset, K);
	cv::Mat pt(3, 1, CV_64F);
	pt.at<double>(0) = object2d.x * nowdepth;
	pt.at<double>(1) = object2d.y * nowdepth;
	pt.at<double>(2) = nowdepth;
	cv::Mat T_inv = T.inv();
	pt = T_inv.rowRange(0, 3).colRange(0, 3) * pt + T_inv.rowRange(0, 3).col(3);
	TRACE("Point guess %f %f %f \n", pt.at<double>(0), pt.at<double>(1), pt.at<double>(2));
	return pt;
}

void Camera::CameraMoveFromReal3D(int cameraModel, cv::Mat pt3d) {
	cv::Mat pt = T.rowRange(0, 3).colRange(0, 3) * pt3d + T.rowRange(0, 3).col(3);
	cv::Point2f pc = pixel2cam(centerOfImage, K);
	cv::Mat ptimage = K * pt;
	double depth = ptimage.at<double>(2);
	centerOfBox = cv::Point2f(ptimage.at<double>(0) / depth, ptimage.at<double>(1) / depth) - offset;
	boundingBox.x = centerOfBox.x - boundingBox.width / 2;
	boundingBox.y = centerOfBox.y - boundingBox.height / 2;
	float lobj = sqrt(pow(pt.at<double>(0), 2) + pow(pt.at<double>(2), 2));
	float lwin = sqrt(pc.x * pc.x + 1);
	float cross = (pc.x * pt.at<double>(0) + pt.at<double>(2)) / lobj / lwin;
	float angley = acosf(cross);
	if (pt.at<double>(0) > 0) angley *= -1;
	TRACE("object in image2d %f %f\n", centerOfBox.x, centerOfBox.y);
	TRACE("image2d %f %f\n", centerOfImage.x, centerOfImage.y);
	TRACE("object3d %f %f %f\n", pt.at<double>(0), pt.at<double>(1), pt.at<double>(2));
	TRACE("angle %f\n", angley);
	TRACE("angle degree %f\n", angley / acos(-1) * 180);
	angley = angley / acos(-1) * 180;
	double tdeg = angleAlly + angley;
	double tdis = 1.0 * (preBoundingBox.width + preBoundingBox.height) / (boundingBox.width + boundingBox.height);
	path.push_back(make_pair(tdeg / 180 * acos(-1), tdis));

	move = false;

	if (idx == 2 && move) {
		//9 / 60 * 30 = 4.5
		if (fabs(angley) < 0.5) {
			TRACE("angle all %f\n", angleAlly);
			return;
		}
		if (fabs(angley) < 2) move = !move;
		if (fabs(angley) > 0.1) angley = angley / fabs(angley) * 0.1;
		MoveToPolarPosition(angley, 0, 0, 9000, 2);
		angleAlly += angley;
		Eigen::Matrix3d angleMatrix = Eigen::AngleAxisd(angley / 180 * acos(-1), Eigen::Vector3d::UnitY()).matrix();
		cv::Mat Rnow = cv::Mat::eye(4, 4, CV_64F);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) Rnow.at<double>(i, j) = angleMatrix(i, j);
		}
		T *= Rnow;
	}
	else if (!move) {
		move = !move;
	}
	TRACE("angle all %f\n", angleAlly);
	
	//move = !move;
	puts("test");

}