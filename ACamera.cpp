#include "stdafx.h"
#include "ACamera.h"


ACamera::ACamera(void)
{
}

ACamera::ACamera(int IDC_CAMERA_SHOW_, int height_, int width_, CnComm *com) {
	IDC_CAMERA_SHOW = IDC_CAMERA_SHOW_;
	heightOfScreen = height_;
	widthOfScreen = width_;
	m_com = com;
	state = 0;
	move = true;
	isopen = false;
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
	//0.99663, 0.0818709, 0.00111595, -0.00485828
	//0.99663, 0.0818709, 0.00111595, -0.00485828
	//-0.800911, 2.1634, 4.9686
	input_t = "-0.474525, 1.83571, 4.19936";
	input_q = "0.99579, 0.0712218, -0.0568529, 0.00989473";

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
ACamera::~ACamera(void)
{
}

void ACamera::OpenPlatform() {
	if(m_com->IsOpen()){
		m_com->Close();
	}
	if (!m_com->Open(comId, 115200, NOPARITY, 8, ONESTOPBIT)) {
			MessageBox(NULL, "Fail", "Open platform", MB_OK);
	}
	else isopen = true;
	p_recive = new thread(&ACamera::Recive, this);
}

void ACamera::CameraMove() {
	cv::Point2f object2d = pixel2cam(centerOfBox, K);
	cv::Point2f window2d = pixel2cam(centerOfImage, K);
	float lobj = sqrt(object2d.x * object2d.x + 1);
	float lwin = sqrt(window2d.x * window2d.x + 1);
	float cross = (object2d.x * window2d.x + 1) / lobj / lwin;
	float angley = acosf(cross);
	if (object2d.x > 0) angley *= -1;
	if (angley > 3.14 || angley < -3.14) return;
	TRACE("object in image2d %f %f\n", centerOfBox.x, centerOfBox.y);
	TRACE("image2d %f %f\n", centerOfImage.x, centerOfImage.y);
	TRACE("object2d %f %f\n", object2d.x, object2d.y);
	TRACE("angle %f\n", angley);
	TRACE("angle degree %f\n", angley / acos(-1) * 180);

	//if you don't want to move
	//move = false;
	//Recive();

	if (move) {
		angley = angley / acos(-1) * 180;
		double tdeg = angleAlly + angley;
		double tdis = 1.0 * (preBoundingBox.width + preBoundingBox.height) / (boundingBox.width + boundingBox.height);
		path.push_back(make_pair(tdeg / 180 * acos(-1), tdis));
		//9 / 60 * 30 = 4.5
		if (fabs(angley) < 0.5) {
			TRACE("angle all %f\n", angleAlly);
			return;
		}
		//if (fabs(angley) < 2) move = !move;
		if (fabs(angley) > 5) angley = angley / fabs(angley) * 5;
		MoveToPolarPosition(angley, 0, 0, 6000, 2);
		angleAlly += angley;
		Eigen::Matrix3d angleMatrix = Eigen::AngleAxisd(angley / 180 * acos(-1), Eigen::Vector3d::UnitY()).matrix();
		cv::Mat Rnow = cv::Mat::eye(4, 4, CV_64F);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) Rnow.at<double>(i, j) = angleMatrix(i, j);
		}
		T *= Rnow;
		move = false;
	}
	else if (!move) {
		//move = !move;
	}
	TRACE("angle all %f\n", angleAlly);
	
	//move = !move;
	puts("test");
}


void ACamera::Reset(){
	MoveToAbsolutePosition(110, 0, 40, ARM_SPEED, 1);
}
void ACamera::Close(){
	if(m_com && m_com->IsOpen()){
		m_com->Close();
	}
}

void ACamera::Send(string data){
	if(!m_com || !m_com->IsOpen()) return;
	const char *buf = data.c_str();
	m_com->Write(buf); 
}
void ACamera::MoveToAbsolutePosition(double x, double y, double z, double speed, int model) {
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

void ACamera::MoveToPolarPosition(double r, double s, double h, double speed, int model) {
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

cv::Mat ACamera::GetFrame() {
	cv::Mat frame;
	cap >> frame;
	return frame;
}

void ACamera::CameraMoveFromReal3D(cv::Mat pt3d) {
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

	if (move) {
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
 void ACamera::Move(double deg) {
	MoveToPolarPosition(deg, 0, 0, 12000, 2);
 }

 void ACamera::Open() {
	cap.open(1);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280.0);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720.0);
	
 }
void ACamera::Recive() {
	while (true) {
		if (isopen) {
			char cRx[1024];
			memset(cRx, 0, sizeof(cRx));
			CString strTemp;
			m_com->Read(cRx, 1024);
			strTemp.Format("%s", cRx);
			if (cRx[0] == 'o' && cRx[1] == 'k') {
				move = true;
			}

			Sleep(500);
			TRACE("test recive : %s\n", cRx);
		}
	}
}