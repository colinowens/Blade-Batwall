// This is the current version

#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
    /*
    FILE *in; 
    if(!(in = popen("ls /dev/tty.*", "r"))){ 
        return;
    }
    char line[512];
    std::string result; 
    while (fgets(line, 512, in))
        result += line;

    vector <string> v;
    v = ofSplitString(result, "\n"); 
    
    cout << "devices" << endl;
    for(int i = 0; i < v.size(); i++){
        cout << v[i] << endl;
        cout << "hi" << endl;
    }
    */
    //serial.listDevices();
    

	//ofSetLogLevel(OF_LOG_VERBOSE);
	kinect1.init(false,false);
    kinect1.open();
    
#ifdef ALL_KINECTS
	kinect2.init(false,false);
	kinect2.open();
#endif

    
	//colorImg1.allocate(kinect1.width, kinect1.height);
	grayImage1.allocate(kinect1.width, kinect1.height);
	grayThreshNear1.allocate(kinect1.width, kinect1.height);
	grayThreshFar1.allocate(kinect1.width, kinect1.height);
	nearThreshold1 = 230;
	farThreshold1 = 70;
	bThreshWithOpenCV1 = true;

#ifdef ALL_KINECTS
    //colorImg2.allocate(kinect2.width, kinect2.height);
    grayImage2.allocate(kinect2.width, kinect2.height);
    grayThreshNear2.allocate(kinect2.width, kinect2.height);
    grayThreshFar2.allocate(kinect2.width, kinect2.height);
    nearThreshold2 = 230;
    farThreshold2 = 70;
    bThreshWithOpenCV2 = true;
#endif


	ofSetFrameRate(60);
	
    // GUI
    
    setGUI();
    gui->loadSettings("GUI/guiSettings.xml");
    
    testPattern.loadImage("images/testpattern.jpg");
    bezelHelper.loadImage("images/bezelHelper.png");
    
    fbo.allocate(ofGetWidth(), ofGetHeight());
    bezel.setup(0.0f, gapWidth, 0, 2);
    
}

//--------------------------------------------------------------
void testApp::update() {
    
    bezel.setColumnSpacer( gapWidth);
    
    // kinect 1
    
    kinect1.update();
	if(kinect1.isFrameNew()) {
		grayImage1.setFromPixels(kinect1.getDepthPixels(), kinect1.width, kinect1.height);
		if(bThreshWithOpenCV1) {
			grayThreshNear1 = grayImage1;
			grayThreshFar1 = grayImage1;
			grayThreshNear1.threshold(nearThreshold1, true);
			grayThreshFar1.threshold(farThreshold1);
			cvAnd(grayThreshNear1.getCvImage(), grayThreshFar1.getCvImage(), grayImage1.getCvImage(), NULL);
		} else {
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage1.getPixels();
			
			int numPixels = grayImage1.getWidth() * grayImage1.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold1 && pix[i] > farThreshold1) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		grayImage1.flagImageChanged();
		contourFinder1.findContours(grayImage1, minArea1, maxArea1, 1, false);
        cur = contourFinder1.blobs[0].boundingRect;

	}

#ifdef ALL_KINECTS
    // kinect 2
    kinect2.update();
    if(kinect2.isFrameNew()) {
        grayImage2.setFromPixels(kinect2.getDepthPixels(), kinect2.width, kinect2.height);
        if(bThreshWithOpenCV2) {
            grayThreshNear2 = grayImage2;
            grayThreshFar2 = grayImage2;
            grayThreshNear2.threshold(nearThreshold2, true);
            grayThreshFar2.threshold(farThreshold2);
            cvAnd(grayThreshNear2.getCvImage(), grayThreshFar2.getCvImage(), grayImage2.getCvImage(), NULL);
        } else {
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage2.getPixels();
            
            int numPixels = grayImage2.getWidth() * grayImage2.getHeight();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold2 && pix[i] > farThreshold2) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        grayImage2.flagImageChanged();
        contourFinder2.findContours(grayImage2, minArea2, maxArea2, 2, false);
    }
    
    #endif
 

    
    
    
}

//--------------------------------------------------------------
void testApp::draw() {
    fbo.begin(); // put all your code after this line
    

    
    ofBackground(0, 0, 0);
    
    ofSetColor(255);
	if(calibrateMode) { testPattern.draw(0, 0, 1024, 768); testPattern.draw(1024, 0, 1024, 768); }
    if(bezelHelperMode) { bezelHelper.draw(0, 0, 1024, 768); bezelHelper.draw(1024, 0, 1024, 768); }
    if(bCentroid) {
        ofSetColor(255,0,0,127);
        ofRect(cur.x, cur.y, cur.width, cur.height);
        
        
  
    
    } 
    
    // draw from the live kinect
    if(bKinectImage)kinect1.drawDepth(kinect1X, kinect1Y, 1024, 768);
    if(bCVImage)contourFinder1.draw(kinect1X, kinect1Y, 1024, 768);
    if(bScreenOrderMode) {
        ofSetColor(255);

        ofDrawBitmapString("1",1024/2, 768/2);
    }


/*
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
    
    if(kinect1.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect1.getMksAccel().x, 2) << " / "
        << ofToString(kinect1.getMksAccel().y, 2) << " / "
        << ofToString(kinect1.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV1 <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold1 << " (press: + -)" << endl
	<< "set far threshold " << farThreshold1 << " (press: < >) num blobs found " << contourFinder1.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect1.isConnected() << endl
    << ", serial: " << endl;
	ofDrawBitmapString(reportStream.str(), 20, 652);
*/
    fbo.end(); // these two lines of code
    bezel.draw(&fbo); // go at the very end
    
}

void testApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.getName();
	int kind = e.getKind();
  
    
    if(kind == OFX_UI_WIDGET_TOGGLE)
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        cout << name << "\t value: " << toggle->getValue() << endl;
     if (name == "Kinect 1 1" && toggle->getValue() ==true) {
            
            kinect1X = 0;
            kinect1Y = 0;
            
        }
        if (name == "Kinect 1 2" && toggle->getValue() ==true) {
        
            kinect1X = 1024;
            kinect1Y = 0;
        }
        if (name == "Kinect 1 3" && toggle->getValue() ==true) {
            
            kinect1X = 2048;
            kinect1Y = 0;
        }
        if (name == "Kinect 1 4" && toggle->getValue() ==true) {
            
            kinect1X = 3072;
            kinect1Y = 0;
        }
    }
}

void testApp::setGUI()
{
    vector<string> names;
    
    gui = new ofxUISuperCanvas("Setup",1000,0,300,300);
    gui->addSpacer();
    gui->addLabel("Press 'g' to Hide GUIs", OFX_UI_FONT_SMALL);
    
    // Kinect 1
    gui->addSpacer();
    gui->addLabel("KINECT 1");
    gui->addSpacer();
    gui->addToggle("Kinect 1 1", true)->setLabelVisible(true);
    gui->addToggle("Kinect 1 2", false)->setLabelVisible(true);
  //  gui->addToggle("Kinect 1 3", false)->setLabelVisible(true);
  //  gui->addToggle("Kinect 1 4", false)->setLabelVisible(true);
    gui->addSpacer();
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    gui->addSlider("Near threshold", 0.0, 255.0, &farThreshold1);
	gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
	gui->addSlider("Far threshold", 0.0, 255.0, &nearThreshold1);
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    gui->addSlider("Min Area", 0.0, (kinect1.width*kinect1.height)/2, &minArea1);
    gui->addSlider("Max Area", 0.0, (kinect1.width*kinect1.height)/2, &maxArea1);
    gui->addSpacer();
    gui->addSpacer();
#ifdef ALL_KINECTS    
    // Kinect 2
    gui->addSpacer();
    gui->addLabel("KINECT 2");
    gui->addSpacer();
    gui->addToggle("Kinect 2 1", true)->setLabelVisible(false);
    gui->addToggle("Kinect 2 2", false)->setLabelVisible(true);
    gui->addSpacer();
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    gui->addSlider("Near threshold", 0.0, 255.0, &farThreshold2);
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    gui->addSlider("Far threshold", 0.0, 255.0, &nearThreshold2);
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    gui->addSlider("Min Area", 0.0, (kinect2.width*kinect2.height)/2, &minArea2);
    gui->addSlider("Max Area", 0.0, (kinect2.width*kinect2.height)/2, &maxArea2);
    gui->addSpacer();
    gui->addSpacer();
#endif

    
    //bottom panel
    gui->addSlider("Gap Width", 0.0, 500, &gapWidth);
    gui->addSpacer();
	gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    gui->addSpacer();
    gui->addLabelToggle("DEPTH CAMERA", &bKinectImage);
    gui->addSpacer();
    gui->addLabelToggle("CV IMAGE", &bCVImage);
    gui->addSpacer();
    gui->addLabelToggle("CENTROID", &bCentroid);
    gui->addSpacer();
    gui->addLabelToggle("SCREEN ORDER", &bScreenOrderMode);
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
	

	kinect1.setCameraTiltAngle(0); // zero the tilt on exit
	kinect1.close();

#ifdef ALL_KINECTS   
	kinect2.close();
    /*
    kinect3.close();
    kinect4.close();
     */
#endif
    
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV1 = !bThreshWithOpenCV1;
			break;
			
		case'p':
			bDrawPointCloud1 = !bDrawPointCloud1;
			break;
			
		case '>':
		case '.':
			farThreshold1 ++;
			if (farThreshold1 > 255) farThreshold1 = 255;
			break;
			
		case '<':
		case ',':
			farThreshold1 --;
			if (farThreshold1 < 0) farThreshold1 = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold1 ++;
			if (nearThreshold1 > 255) nearThreshold1 = 255;
			break;
			
		case '-':
			nearThreshold1 --;
			if (nearThreshold1 < 0) nearThreshold1 = 0;
			break;
			
		case 'w':
		//	kinect1.enableDepthNearValueWhite(!kinect1.isDepthNearValueWhite());
			break;
			
		case 'o':
		//	kinect1.setCameraTiltAngle(angle); // go back to prev tilt
		//	kinect1.open();
			break;
			
		case 'c':
		//	kinect1.setCameraTiltAngle(0); // zero the tilt
		//	kinect1.close();
			break;
			
		case '1':
		//	kinect1.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
		//	kinect1.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
		//	kinect1.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
		//	kinect1.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
		//	kinect1.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
		//	kinect1.setLed(ofxKinect::LED_OFF);
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
