#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxUI.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
#ifdef USE_THREE_KINECTS
    ofxKinect kinect2;
	ofxKinect kinect3;
#endif
#ifdef USE_FOUR_KINECTS
    ofxKinect kinect2;
	ofxKinect kinect3;
	ofxKinect kinect4;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
    
    // my stuff
    int multX;
	int multY;
    
    // gui
    ofxUISuperCanvas *gui;
    void setGUI();
    void guiEvent(ofxUIEventArgs &e);
    
	// used for viewing the point cloud
	ofEasyCam easyCam;
};
