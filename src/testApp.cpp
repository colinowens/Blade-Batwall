#include "testApp.h"

// I made a comment!!!!


//--------------------------------------------------------------
void testApp::setup() {
    
#ifdef USE_ONE_KINECT
	//ofSetLogLevel(OF_LOG_VERBOSE);
	kinect.setRegistration(true);
	kinect.init(false,false);
    kinect.open("B00364721963039B");	// open a kinect using it's unique serial #
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
#endif
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
#ifdef USE_THREE_KINECTS
	kinect2.init();
	kinect2.open();
    kinect3.init();
	kinect3.open();
#endif
#ifdef USE_FOUR_KINECTS
    kinect2.init();
	kinect2.open();
    kinect3.init();
	kinect3.open();
	kinect4.init();
	kinect4.open();
#endif
#ifdef USE_ONE_KINECT
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
#endif
    
    // GUI
    
    setGUI();
    gui->loadSettings("GUI/guiSettings.xml");
    
    testPattern.loadImage("images/testpattern.jpg");
    bezelHelper.loadImage("images/bezelHelper.png");
    
    fbo.allocate(ofGetWidth(), ofGetHeight());
    bezel.setup(0.0f, 150.0f, 0, 2);
    
    
}

//--------------------------------------------------------------
void testApp::update() {

//	ofBackground(0, 0, 0);
    
#ifdef USE_ONE_KINECT
    kinect.update();
	if(kinect.isFrameNew()) {
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		grayImage.flagImageChanged();
		contourFinder.findContours(grayImage, minArea, maxArea, 1, false);
	}
#endif
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
#ifdef USE_THREE_KINECTS
	kinect2.update();
    kinect3.update();
#endif
#ifdef USE_FOUR_KINECTS
    kinect2.update();
    kinect3.update();
	kinect4.update();
#endif
    
    
    
}

//--------------------------------------------------------------
void testApp::draw() {
    fbo.begin(); // put all your code after this line
    
    ofBackground(0, 0, 0);
    
    ofSetColor(255);
	if(calibrateMode)testPattern.draw(0, 0, ofGetScreenWidth()*2, ofGetScreenHeight());
    if(bezelHelperMode)bezelHelper.draw(0, 0, ofGetScreenWidth()*2, ofGetScreenHeight());
    
	
    
	
#ifdef USE_ONE_KINECT
    // draw from the live kinect
    kinect.drawDepth(10, 10, 400, 300);
    kinect.draw(2420, 10, 400, 300);
    
    grayImage.draw(10, 320, 400, 300);
    contourFinder.draw(10, 320, 400, 300);
#endif
#ifdef USE_TWO_KINECTS
    kinect.draw(2420, 10, 400, 300);
    kinect2.draw(420, 320, 400, 300);
#endif
	
#ifdef USE_ONE_KINECT
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
    
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
    
    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
	ofDrawBitmapString(reportStream.str(), 20, 652);
#endif
    fbo.end();
    bezel.draw(&fbo);
    
    
}

void testApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.getName();
	int kind = e.getKind();
	cout << kind << name << endl;
}
void testApp::setGUI()
{
    vector<string> names;
	names.push_back("RAD1");
	names.push_back("RAD2");
	names.push_back("RAD3");
    
	gui = new ofxUISuperCanvas("Setup",1000,0,300,300);
    gui->addSpacer();
    gui->addLabel("Press 'g' to Hide GUIs", OFX_UI_FONT_SMALL);
    
    
    gui->addSpacer();
    gui->addLabel("KINECT 1");
    gui->addSlider("Near threshold", 0.0, 255.0, &farThreshold);
	gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
	gui->addSlider("Far threshold", 0.0, 255.0, &nearThreshold);
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
#ifdef USE_ONE_KINECT
    gui->addSlider("Min Area", 0.0, (kinect.width*kinect.height)/2, &minArea);
    gui->addSlider("Max Area", 0.0, (kinect.width*kinect.height)/2, &maxArea);
#endif
    
    
	gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    gui->addSpacer();
    gui->addLabelToggle("TEST PATTERN", &calibrateMode);
    gui->addSpacer();
    gui->addLabelToggle("CALIBRATE", &bezelHelperMode);
    gui->addSpacer();
    string textString = "SOUND IMAGE MOTION 2014";
    gui->addSpacer();
    gui->addTextArea("textarea", textString, OFX_UI_FONT_SMALL);
    gui->autoSizeToFitWidgets();
	ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);
}


//--------------------------------------------------------------
void testApp::exit() {
    gui->saveSettings("GUI/guiSettings.xml");
    delete gui;
	
#ifdef USE_ONE_KINECT
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
#endif
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
#ifdef USE_THREE_KINECTS
	kinect2.close();
    kinect3.close();
#endif
#ifdef USE_FOUR_KINECTS
	kinect2.close();
    kinect3.close();
    kinect4.close();
#endif
    
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
		//	kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
		//	kinect.setCameraTiltAngle(angle); // go back to prev tilt
		//	kinect.open();
			break;
			
		case 'c':
		//	kinect.setCameraTiltAngle(0); // zero the tilt
		//	kinect.close();
			break;
			
		case '1':
		//	kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
		//	kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
		//	kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
		//	kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
		//	kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
		//	kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
		//	kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
		//	kinect.setCameraTiltAngle(angle);
			break;
        case 'f':
            ofToggleFullscreen();
            break;
        case 'g':
            gui->toggleVisible();
            break;
        case 's':
            gui->saveSettings("GUI/guiSettings.xml");
            break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}
//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
