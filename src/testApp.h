#pragma once
#define USE_ONE_KINECT	

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxUI.h"
#include "ofxBezel.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
    void urlResponse(ofHttpResponse & response);
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    
	

    
#ifdef USE_NO_KINECT
#endif
#ifdef USE_ONE_KINECT
    ofxKinect kinect;
#endif
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
	
    //Kinect 1
    
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	float nearThreshold;
	float farThreshold;
    float minArea;
    float maxArea;
	
	int angle;
    
    // my stuff
    int multX;
	int multY;
    
    bool calibrateMode;
    bool bezelHelperMode;
    
    ofImage testPattern;
    ofImage bezelHelper;
    
    

    
    // gui
    ofxUISuperCanvas *gui;
    void setGUI();
    void guiEvent(ofxUIEventArgs &e);
    
	// used for viewing the point cloud
	ofEasyCam easyCam;
    
    // bezel
    
    ofFbo fbo;
    ofxBezel bezel;
    ofVideoPlayer player;
};
