#include "ofApp.h"
//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	// enable depth->video image calibration
	kinect.setRegistration(true);    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();		// opens first available kinect
    
    droneImage.loadImage("droneImage.jpg");
    debugWindow.setup("debug Window", 0, 0, 1280/2, 960/2, false);
    
    //setting up the kinect CV images
    nearThreshold = 230;
    farThreshold = 10;
    minArea = 1000;
    maxArea = 70000;
    threshold = 15;
    persistence = 15;
    maxDistance = 32;
    kLeftThreshold = -1000;
    kRightThreshold = 1000;
    kTopThreshold = 1000;
    kBottomThreshold = -1000;
    kFrontThreshold = 0;
    kBackThreshold = 6000;
    colorMapping =100;
    
    
    kinectDepthImage.allocate(kinect.width, kinect.height);
    kinectThresholdedImage.allocate(kinect.width, kinect.height);

    
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif

	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	 bDrawPointCloud = true;
     kinect.update();
     mesh.setMode(OF_PRIMITIVE_POINTS);
     trackingMesh.setMode(OF_PRIMITIVE_POINTS);
     tempMesh.setMode(OF_PRIMITIVE_POINTS);
    
    
     for ( int i = 0 ; i< w ; i++)
        {
            for ( int j = 0 ; j< h ; j++)
                {
                    ofColor c = kinect.getColorAt(i, j);
                    mesh.addColor(c);
                    mesh.addVertex(kinect.getWorldCoordinateAt(i, j)*10);
                }
        }
    int noOfVertices = mesh.getNumVertices();
    cout<< "number of vertices are:"<< noOfVertices;
    
    
    /*
    
    //meshCopy.clear();
    //meshCopy.setMode(OF_PRIMITIVE_POINTS);
    meshCopy.enableIndices();
    meshCopy.setMode(OF_PRIMITIVE_LINES);

    
    for ( int i = 0 ; i< mesh.getNumVertices(); i+=step)
        
    {
        ofColor c;
        c.set(0, 100, 255);
        meshCopy.addColor(c);
        meshCopy.addVertex( mesh.getVertex(i));
        
    }
    
    cout<<"meshcopy vetices are: "<< meshCopy.getNumVertices();

    
    int height = h;
    int width = w;
    
    for (int y = 0; y<height-1; y++){
        for (int x=0; x<width-1; x++){
            mesh.addIndex(x+y*width);               // 0
            mesh.addIndex((x+1)+y*width);           // 1
            mesh.addIndex(x+(y+1)*width);           // 10
            
            mesh.addIndex((x+1)+y*width);           // 1
            mesh.addIndex((x+1)+(y+1)*width);       // 11
            mesh.addIndex(x+(y+1)*width);           // 10
        }
    }
    */
    
    // setup gui
    gui1 = new ofxUISuperCanvas("PANEL 1: OpenCV");
    gui1->setHeight(800);
    gui1->setName("parameters");
    gui1->addLabel("kinect");
    gui1->addSpacer();
    gui1->addSlider("nearThresh", 0, 255, &nearThreshold);
    gui1->addSlider("farThresh", 0, 255, &farThreshold);
    gui1->addLabel("contours");
    gui1->addSpacer();
    gui1->addSlider("minArea", 0, 5000, &minArea);
    gui1->addSlider("maxArea", 5000, 50000, &maxArea);
    gui1->addSlider("threshold", 1, 100, &threshold);
    gui1->addSlider("persistence", 1, 100, &persistence);
    gui1->addSlider("maxDistance", 1, 100, &maxDistance);
    gui1->addSlider("front Threshold", 0, 6000, &kFrontThreshold);
    gui1->addSlider("back Threshold", 0, 10000, &kBackThreshold);
    gui1->addSlider("top Threshold", 0, 1000, &kTopThreshold);
    gui1->addSlider("bottom Threshold", -1000, 0, &kBottomThreshold);
    gui1->addSlider("left Threshold", 0, 1000, &kLeftThreshold);
    gui1->addSlider("right Threshold", -1000, 0, &kRightThreshold);
    gui1->addSlider("colorMapping", 10, 2000, &colorMapping);
    
    gui1->addSlider("strip width", 2, 10,  &stripWidth );
  
    gui1->addToggle("isTrackingOn", false);
    
    gui1->setWidgetColor(OFX_UI_WIDGET_COLOR_FILL, ofColor(0,128,192,200));
    gui1->setWidgetColor(OFX_UI_WIDGET_COLOR_FILL_HIGHLIGHT, ofColor(0,128,128, 200));
    
    
    
   

}

//--------------------------------------------------------------
void ofApp::update() {         
	
	ofBackground(100, 100, 100);

    mesh.clearVertices();
    mesh.clearColors();
    
	kinect.update();
    //trackingMesh.append(mesh);
	for ( int i = 0 ; i< w ; i++)
            {
                for ( int j = 0 ; j< h ; j++)
                    {
                        bool isItIn = false;
                        isItIn = checkPointWithinLimits( kinect.getWorldCoordinateAt(i, j) );
                        if( isItIn == true)
                            {
                            ofColor c ;
                            c.setHsb(kinect.getWorldCoordinateAt(i, j).z /colorMapping, 255, 255 );
                                
                            //mesh.setColor(j*w+i ,c);
                            //mesh.setVertex( j* w + i  , kinect.getWorldCoordinateAt(i, j) );
                                
                            mesh.addColor(c);
                            mesh.addVertex(kinect.getWorldCoordinateAt(i, j) );
                                                       }
                

                    }
 
            }
    
    
    kinectDepthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    kinectThresholdedImage = kinectDepthImage;
    
    kinectThresholdedImage.threshold(nearThreshold,true);
    kinectThresholdedImage.threshold(farThreshold,true);
    contourFinder.setMinArea(minArea);
    contourFinder.setMaxArea(maxArea);
    contourFinder.setThreshold(threshold);
    contourFinder.getTracker().setPersistence(persistence);
    contourFinder.getTracker().setMaximumDistance(maxDistance);
    
    // determine found contours
    contourFinder.findContours(kinectThresholdedImage);
    droneRects =  contourFinder.getBoundingRects();
    
    kinectThresholdMat = toCv(kinectThresholdedImage.getPixelsRef());
    droneMats.clear();
    globalDroneAverages.clear();

    
    for ( int i = 0 ; i < droneRects.size() ; i++ )
    {
        //droneMats[i] = kinectThresholdMat(droneRects[i]);
        Mat tempMat;
        kinectThresholdMat(droneRects[i]).copyTo(tempMat);
        droneMats.push_back(tempMat);
        
    }
    
    
    for ( int  i = 0 ; i < droneMats.size() ; i ++)
    {
        
        vector< Scalar > droneAverages;
        vector<Mat> droneAverageMats;
        droneAverages.clear();
        
        Scalar oldAvg( 0);
        Scalar curAvg;
        Scalar gradient;
        
        cout<<endl<< "gradient: ";

        
        for ( int j = 0 ; j< droneMats[i].cols- int(stripWidth)  ; j+= int(stripWidth) )
        {
            Mat tempMat;
            
            droneMats[i]( cv::Rect(j,0,int(stripWidth) ,droneMats[i].rows-1)).copyTo(tempMat);
            
            curAvg = mean(tempMat);
            
            droneAverages.push_back( curAvg );
            
            gradient = curAvg - oldAvg;
            cout<<" " << gradient.val[0];
            
            globalDroneAverages.push_back(droneAverages);
            //droneAverages.push_back(  mean(droneMats[i]( cv::Rect(j,0,5,droneMats[i].rows) ) ) ) ;
            
            
            oldAvg = curAvg;
            
        }
        
        
    }
    
    //cout<<endl<<"size of the vector: " << globalDroneAverages.size() << endl;
    
    
    
    
//    contourFinder.size();

#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}




//--------------------------------------------------------------
void ofApp::draw() {
    
    kinectDepthImage.draw(0,480);
    kinectThresholdedImage.draw(0,0);
    ofPushMatrix();
    ofTranslate(0,0);
    contourFinder.draw();
    ofPopMatrix();
    
    /*
    if ( droneRects.size()!=0)
    {
        for (int i = 0 ; i < droneRects.size() ; i++)
        {
            cout<<endl<< "point i:  "<< i << " " << droneRects[i].tl().x <<" "<< droneRects[i].tl().y<<" " << droneRects[i].br().x <<" "<< droneRects[i].br().y <<endl ;
            
        }
    }
     */
    
    ofPushMatrix();
    //ofTranslate(640,0);
    for ( int i = 0 ; i < droneMats.size(); i++)
        drawMat(droneMats[i], 640, i * 200);
    
    
    ofPopMatrix();
    ofDrawBitmapString( ofToString(record) , 20, 400);
    debugWindow.begin();
    ofBackground(0, 0, 0);
    glPointSize(1);
    //glLineWidth(1);
    easyCam.begin();
    ofPushMatrix();
    ofScale(1, -1, -1);
    //ofTranslate( 0 ,0 ,0);
    
    //ofTranslate(ofGetWidth()/2, ofGetHeight()/2);
    ofEnableDepthTest();
    mesh.draw();
    //kinect.draw(0, 0, 640, 480);
    //kinect.drawDepth(0, 0, 640, 480);
    
    ofDisableDepthTest();
    ofPopMatrix();
    easyCam.end();
    debugWindow.end();
}



//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		
        case'w':
            writingToFile = !writingToFile;
            break;
            
        case 'r':
            record =!record;
            
            
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
					
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}



bool ofApp::checkPointWithinLimits(ofVec3f point)
{
    bool itsIn = false;
    //if( point.x > kLeftThreshold && point.x < kRightThreshold && point.y < kTopThreshold && point.y > kBottomThreshold && point.z > kFrontThreshold && point.z < kBackThreshold )
    if( point.z > kFrontThreshold && point.z < kBackThreshold )
        {
            itsIn = true;
        }
    
    return itsIn;
}








