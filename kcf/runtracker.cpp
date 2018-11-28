#include "mainwindow.h"
#include <QApplication>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.h"

//#include <dirent.h>

using namespace std;
using namespace cv;


int main(int argc, char* argv[]){

    //if (argc > 5) return -1;

    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = false;
    bool LAB = false;

    /*
    for(int i = 0; i < argc; i++){
        if ( strcmp (argv[i], "hog") == 0 )
            HOG = true;
        if ( strcmp (argv[i], "fixed_window") == 0 )
            FIXEDWINDOW = true;
        if ( strcmp (argv[i], "singlescale") == 0 )
            MULTISCALE = false;
        if ( strcmp (argv[i], "show") == 0 )
            SILENT = false;
        if ( strcmp (argv[i], "lab") == 0 ){
            LAB = true;
            HOG = true;
        }
        if ( strcmp (argv[i], "gray") == 0 )
            HOG = false;
    }
    */

    // Create KCFTracker object
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

    // Frame readed
    Mat frame;

    // Tracker results
    Rect result;

    // Path to list.txt
    /*
    ifstream listFile;
    string fileName = "images.txt";
    listFile.open(fileName);
    */
    // Read groundtruth for the 1st frame
    ifstream groundtruthFile;
    string groundtruth = "D:/Data/tracker_data/Car4/groundtruth_rect.txt";
    groundtruthFile.open(groundtruth);
    string firstLine;
    getline(groundtruthFile, firstLine);
    groundtruthFile.close();

    istringstream ss(firstLine);

    // Read groundtruth like a dumb
    float x1, y1, x2, y2, x3, y3, x4, y4;
    char ch;
    ss >> x1;
    ss >> y1;
    ss >> x2;
    ss >> y2;
    ss >> x3;
    ss >> y3;
    ss >> x4;
    ss >> y4;


    // Using min and max of X and Y for groundtruth rectangle
    float xMin =  x1;//min(x1, min(x2, min(x3, x4)));
    float yMin =  y1;//min(y1, min(y2, min(y3, y4)));
    float width = x2;//max(x1, max(x2, max(x3, x4))) - xMin;
    float height = y2;//max(y1, max(y2, max(y3, y4))) - yMin;


    // Read Images
    //ifstream listFramesFile;
    string listFrames = "D:/Data/tracker_data/Car4/img/";
    //listFramesFile.open(listFrames);
    string frameName;


    // Write Results
    ofstream resultsFile;
    string resultsPath = "output.txt";
    resultsFile.open(resultsPath);

    // Frame counter nFrames


    for (int nFrames = 1; nFrames < 659; nFrames++) {
        char str[10];
        sprintf(str, "%04d.jpg", nFrames);
        frameName = str;
        frameName = listFrames + frameName;

        // Read each frame from the list
        frame = imread(frameName, CV_LOAD_IMAGE_COLOR);

        // First frame, give the groundtruth to the tracker
        if (nFrames == 1) {
            tracker.init( Rect(xMin, yMin, width, height), frame );
            rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 1, 8 );
            resultsFile << xMin << "," << yMin << "," << width << "," << height << endl;
        }
        // Update
        else{
            result = tracker.update(frame);
            rectangle( frame, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 255, 255 ), 1, 8 );
            resultsFile << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
        }


        if (!SILENT){
            imshow("Image", frame);
            //waitKey(0);
        }
    }
    resultsFile.close();

    //listFile.close();

}
