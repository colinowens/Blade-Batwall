#pragma once

#define ALL_KINECTS



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
	
    //Kinect 1
    
    ofxKinect kinect1;
    ofxCvGrayscaleImage grayImage1;
    ofxCvGrayscaleImage grayThreshNear1;
    ofxCvGrayscaleImage grayThreshFar1;
    ofxCvContourFinder contourFinder1;
    bool bThreshWithOpenCV1;
    bool bDrawPointCloud1;
    float nearThreshold1;
    float farThreshold1;
    float minArea1;
    float maxArea1;
    int kinect1X = 0;
    int kinect1Y = 0;
    int kinect1W = 400;
    int kinect1H = 300;
    ofRectangle cur1;
    
#ifdef ALL_KINECTS
    //Kinect 2
    
    ofxKinect kinect2;
    ofxCvGrayscaleImage grayImage2;
    ofxCvGrayscaleImage grayThreshNear2;
    ofxCvGrayscaleImage grayThreshFar2;
    ofxCvContourFinder contourFinder2;
    bool bThreshWithOpenCV2;
    bool bDrawPointCloud2;
    float nearThreshold2;
    float farThreshold2;
    float minArea2;
    float maxArea2;
    int kinect2X = 0;
    int kinect2Y = 0;
    int kinect2W = 400;
    int kinect2H = 300;
    ofRectangle cur2;
 #endif
      
    // my stuff
    float multX;
	float multY;
    
    bool calibrateMode;
    bool bezelHelperMode;
    bool bKinectImage;
    bool bCVImage;
    bool bCentroid;
    bool bScreenOrderMode;
    
    
    ofImage testPattern;
    ofImage bezelHelper;
    

    float gapWidth =140;
    
    float pointerX;
    float pointerY;
    
    float projectorWidth;
    float projectorHeight;
    
    float blobX;
    float blobY;
    
    bool anyBlobs;
    
    int durationTime = 3000;
    int startTime;
    int futureTime;
    bool timeNotFinished = true;
    
    
    // gui
    ofxUISuperCanvas *gui;
    void setGUI();
    void guiEvent(ofxUIEventArgs &e);
    
    
    // bezel
    
    ofFbo fbo;
    ofxBezel bezel;
    ofVideoPlayer player;
};
