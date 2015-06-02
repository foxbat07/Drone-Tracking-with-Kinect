#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxSecondWindow.h"
#include "ofxUI.h"
#include "ofxXmlSettings.h"
#include "ofxCv.h"
// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

// uncomment this to read from two kinects simultaneously

//#define USE_TWO_KINECTS

using namespace ofxCv;
using namespace cv;

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	void meshInitialize();
    
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    void recordMesh();
    bool checkPointWithinLimits( ofVec3f point);
    
    
    //void writePointsToFile();
    void createNewPointFile();
    void addPointsToFile(ofVec3f point);
    void stopFileWrite();
    bool writingToFile = false;
    
    float calculateCorrectedMean(Mat stripMat);
    
    
    
	ofxKinect kinect;
    ofEasyCam easyCam;
    ofxCvGrayscaleImage kinectDepthImage;
    ofxCvGrayscaleImage kinectThresholdedImage;
    ofxCv::ContourFinder  contourFinder;
    
    ofxCvBlob kinectBlobDetector;
    
    ofxSecondWindow debugWindow;
    ofxUICanvas *gui1;
    ofMesh mesh;
    ofMesh trackingMesh;
    ofMesh tempMesh;
    
    ofImage droneImage;
    
    
    
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	

	
	bool bDrawPointCloud = true;
	
	int w = 640;
	int h = 480;
	int angle;

    
    int step =7;
    
    float nearThreshold ;
    float farThreshold ;
    float minArea;
    float maxArea;
    float threshold;
    float persistence;
    float maxDistance;
    
    
    
    float kBottomThreshold;
    float kTopThreshold;
    float kLeftThreshold;
    float kRightThreshold;
    float colorMapping;
    
    float kFrontThreshold;
    float kBackThreshold;
    bool  isTrackingOn;
    bool record =false;
    
    float stripWidth = 5;
    
    
    vector<cv::Rect> droneRects;
    
    vector<Mat> droneMats;
    
    Mat kinectThresholdMat;
    
    
    vector<vector< Scalar > > globalDroneAverages;
    
    
    
    

};
