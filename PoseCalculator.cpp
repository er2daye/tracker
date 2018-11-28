#include "stdafx.h"
#include "PoseCalculator.h"


PoseCalculator::PoseCalculator(void)
{
}


PoseCalculator::~PoseCalculator(void)
{
}

void PoseCalculator::getMatch(Mat &img_1, Mat &img_2) {
	vector<KeyPoint> keypoints_1, keypoints_2;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2, 4);
	orb->detect(img_1, keypoints_1);
	orb->detect(img_2, keypoints_2);
	Mat descriptors_1, descriptors_2;// descriptors_3;
	orb->compute(img_1, keypoints_1, descriptors_1);
    orb->compute(img_2, keypoints_2, descriptors_2);
	vector<DMatch> matches12;

	VideoCapture capture;
	matches12.clear();
	BFMatcher matcher;
	vector<DMatch> tmp_matches;
	vector<vector<cv::DMatch> > knnMatches;
	knnMatches.clear();
	matcher.knnMatch(descriptors_1, descriptors_2, knnMatches, 10);

	vector<Point2f> kp1, kp2;
	kp1.clear();
	kp2.clear();
	vector<Point2f> kp3, kp4;
	kp1.clear();
	kp2.clear();
    kp3.clear();
    kp4.clear();
	for (size_t i = 0; i < knnMatches.size(); i++) {
		//i = 5;
		//for (int j = 0; j < 10; j++) {
			//tmp_matches.push_back(knnMatches[i][j]);
		//}
		//break;
		const cv::DMatch &bestMatch = knnMatches[i][0];
		const cv::DMatch &betterMatch = knnMatches[i][1];

        float distanceRadio = bestMatch.distance / betterMatch.distance;
        if (distanceRadio < 1 / 1.1) {
            tmp_matches.push_back(bestMatch);
        }
	}
	for (auto match : tmp_matches) {
		kp1.push_back(keypoints_1[match.queryIdx].pt);
		kp2.push_back(keypoints_2[match.trainIdx].pt);
	}

	//matches = tmp_matches;
    //return ;
	vector<uchar> inliers;
	inliers.resize(tmp_matches.size());
	//findHomography(kp1, kp2, CV_RANSAC, 3, inliers, 10, 0.999);
    findFundamentalMat(kp1, kp2, CV_FM_RANSAC,  3, 0.5, inliers);
	double score = 0, num = 0;
	for (size_t i = 0; i < inliers.size(); i++) {
		if (inliers[i] == true) {
			matches12.push_back(tmp_matches[i]);
			num++;
		}
	}

	vector<Point2f> kpts1, kpts2;
    for (size_t i = 0; i < matches12.size(); i++) {
        int l = matches12[i].queryIdx;
        int r = matches12[i].trainIdx;
        kpts1.push_back(keypoints_1[l].pt);
        kpts2.push_back(keypoints_1[r].pt);
    }
    cv::Mat K, R, t;
    //input : kpts1, kpts2, K
    //output : R,t 
    computePose(kpts1, kpts2, K, R, t);
}
void PoseCalculator::computePose(const vector<Point2f> &inPoints1, const vector<Point2f> &inPoints2, const Mat &K, Mat &R, Mat &T) {
	if(inPoints1.size() != inPoints2.size() || inPoints1.size() < 5)
		return;
	
	// Points Number
	int pnum = inPoints1.size();

	// convert vector Points to Mat, in homogeneous coordinate
	Mat homoPts1 = Mat(3, pnum,CV_32F);
	Mat homoPts2 = Mat(3, pnum,CV_32F);

	for(int i = 0; i < pnum; i++)
	{
		homoPts1.at<float>(0,i) = inPoints1[i].x;
		homoPts1.at<float>(1,i) = inPoints1[i].y;
		homoPts1.at<float>(2,i) = 1;

		homoPts2.at<float>(0,i) = inPoints2[i].x;
		homoPts2.at<float>(1,i) = inPoints2[i].y;
		homoPts2.at<float>(2,i) = 1;
	}

	Mat invK = K.inv();

	// invK * points
	homoPts1 = invK * homoPts1;
	homoPts2 = invK * homoPts2;

	// Convert Mat Points to Array for Solve5PointEssential, array[2*i] = x, array[2*i + 1] = y
	double *pts1 = new double[pnum * 2];
	double *pts2 = new double[pnum * 2];
	for(int i = 0; i < pnum; i++)
	{
		pts1[i*2] = homoPts1.at<float>(0,i);
		pts1[i*2 + 1] = homoPts1.at<float>(1,i);

		pts2[i*2] = homoPts2.at<float>(0,i);
		pts2[i*2 + 1] = homoPts2.at<float>(1,i);
	}

	// solve five points algorithm
	vector <cv::Mat> E; // essential matrix
    vector <cv::Mat> P;
	vector<int> inliers;
	
	bool ret = Solve5PointEssential(pts1,pts2,pnum,E,P,inliers);

	delete[] pts1;
	delete[] pts2;

	// find best index
	int best_index = -1;
    if(ret) {
        for(size_t i=0; i < E.size(); i++) {
            if(cv::determinant(P[i](cv::Range(0,3), cv::Range(0,3))) < 0) P[i] = -P[i];
			if(best_index == -1 || inliers[best_index] < inliers[i]) best_index = i;
        }
    }
    else {
        cout << "Could not find a valid essential matrix" << endl;
		R = Mat::eye(3, 3, CV_32F);
		T = Mat::zeros(3,1,CV_32F);
		return;
    }
	cv::Mat PMatrix = P[best_index];
	cv::Mat Ematrix = E[best_index];


	R = PMatrix.colRange(0,3);
	T = PMatrix.colRange(3,4);
}