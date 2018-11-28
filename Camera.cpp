#include "stdafx.h"
#include "Camera.h"


Camera::Camera(void)
{
	TrackerInit();
	mouseState = 0;
	drawBox = false;
	topCornerOfShowBox = CPoint(0, 0);
	bottomCornerOfShowBox = CPoint(0, 0);
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
void Camera::CapturePicture(int cameraModel) {
	if(cameraModel == 1) {
		
	}
	if(cameraModel == 2) {
		Mat frame;
		cap >> frame;
		imwrite("./capture/2.jpg", frame);
	}
	
    
}
void Camera::Open(){
	if(m_com.IsOpen()){
		m_com.Close();
	}
	m_com.Open(comId, 115200, NOPARITY, 8, ONESTOPBIT);
	if(m_com.IsOpen()){
		MessageBox(NULL, "OK", "connect", MB_OK);
	}
}
void Camera::Reset(){
	MoveToAbsolutePosition(200, 0, 100, ARM_SPEED, 1);
}
void Camera::Close(){
	if(m_com.IsOpen()){
		m_com.Close();
	}
}

void Camera::Send(string data){
	if(!m_com.IsOpen()) return;
	const char *buf = data.c_str();
	m_com.Write(buf); 
}
void Camera::MoveToAbsolutePosition(double x, double y, double z, double speed, int model) {
    if (!m_com.IsOpen()) {
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

void Camera::TrackFromImage(Mat nowFrame) {
    if (!bBoxInit) {
        tracker.init(boundingBox, nowFrame);
        bBoxInit = true;
    } else {
        boundingBox = tracker.update(nowFrame);
    }
}

void Camera::CameraMove(int cameraModel) {
	cv::Point2f cB = centerOfBox;
    cv::Point2f cW = centerOfWindow;


	
}