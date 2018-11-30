#include "stdafx.h"
#include "Camera.h"


Camera::Camera(void)
{
	move = true;
	TrackerInit();
	mouseState = 0;
	drawBox = false;
	topCornerOfShowBox = CPoint(0, 0);
	bottomCornerOfShowBox = CPoint(0, 0);
	m_com = NULL;
	angleAlly = 0;
}

Camera::Camera(int idx_, int IDC_CAMERA_SHOW_, int height_, int width_, CnComm *com) 
	: idx(idx_), IDC_CAMERA_SHOW(IDC_CAMERA_SHOW_), heightOfScreen(height_), widthOfScreen(width_), m_com(com) {
	move = true;
	if (idx == 1) {
		K = cv::Mat::zeros(3, 3, CV_32F);
		K.at<float>(0, 0) = 1419.743;
		K.at<float>(1, 1) = 1419.660;
		K.at<float>(0, 2) = 656.47;
		K.at<float>(1, 2) = 337.67;
		K.at<float>(2, 2) = 1;
	}
	if (idx == 2) {
		K = cv::Mat::zeros(3, 3, CV_32F);
		K.at<float>(0, 0) = 1433.715;
		K.at<float>(1, 1) = 1432.291;
		K.at<float>(0, 2) = 647.95;
		K.at<float>(1, 2) = 347.36;
		K.at<float>(2, 2) = 1;
	}
	if (idx == 3) {
		K = cv::Mat::zeros(3, 3, CV_32F);
		K.at<float>(0, 0) = 1433.715;
		K.at<float>(1, 1) = 1432.291;
		K.at<float>(0, 2) = 647.95;
		K.at<float>(1, 2) = 347.36;
		K.at<float>(2, 2) = 1;
	}
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
	MoveToAbsolutePosition(200, 0, 100, ARM_SPEED, 1);
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



void Camera::TrackFromImage(Mat nowFrame) {
    if (!bBoxInit) {
        tracker.init(boundingBox, nowFrame);
        bBoxInit = true;
    } else {
        boundingBox = tracker.update(nowFrame);
    }
}

void Camera::CameraMove(int cameraModel) {
	cv::Point2f cB = cv::Point2f(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
    cv::Point2f cW = cv::Point2f(frame.cols / 2, frame.rows / 2);
	cv::Point2f object2d = pixel2cam(cB, K);
	cv::Point2f window2d = pixel2cam(cW, K);
	float lobj = sqrt(object2d.x * object2d.x + 1);
	float lwin = sqrt(window2d.x * window2d.x + 1);
	float cross = (object2d.x * window2d.x + 1) / lobj / lwin;
	float angley = acosf(cross);
	if (object2d.x > 0) angley *= -1;
	TRACE("object in image2d %f %f\n", cB.x, cB.y);
	TRACE("image2d %f %f\n", cW.x, cW.y);
	TRACE("object2d %f %f\n", object2d.x, object2d.y);
	TRACE("angle %f\n", angley);
	TRACE("angle degree %f\n", angley / acos(-1) * 180);
	angley = angley / acos(-1) * 180;
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
	}
	else if (!move) {
		move = !move;
	}
	TRACE("angle all %f\n", angleAlly);
	
	//move = !move;
	puts("test");


	
}