#include "ofApp.h"
//--------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_ERROR);
    // enable depth->video image calibration
    kinect.setRegistration(true);
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    kinect.open();		// opens first available kinect
    
    // droneImage.loadImage("droneImage.jpg");
    //debugWindow.setup("debug Window", 0, 0, 1280/2, 960/2, false);
    
    setupInitialParameters();
    
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
    kinectAngle = 0;
    kinect.setCameraTiltAngle(kinectAngle);
    
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
    
    
    setupofxUIGUI();
    
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofBackground(100, 100, 100);
    //openCv params
    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    
    
    
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
    
    droneColorImage =kinect.getPixelsRef() ;
    droneColorMat   = toCv(droneColorImage.getPixelsRef());
    
    
    
    
    kinectThresholdedImage = kinectDepthImage;
    
    kinectThresholdedImage.threshold(nearThreshold,true);
    kinectThresholdedImage.threshold(farThreshold,true);
    
    contourFinder.setMinArea(minArea);
    contourFinder.setMaxArea(maxArea);
    contourFinder.setThreshold(threshold);
    contourFinder.getTracker().setPersistence(persistence);
    contourFinder.getTracker().setMaximumDistance(maxDistance);
    
    // determine found contours
    //kinectThresholdedImage.invert();
    
    contourFinder.findContours(kinectThresholdedImage);
    //contourFinder.findContours(kinectDepthImage);
    droneRects =  contourFinder.getBoundingRects();
    
    //kinectThresholdMat = toCv(kinectThresholdedImage.getPixelsRef());
    kinectThresholdMat = toCv(kinectDepthImage.getPixelsRef());   // use the full depth image

    
    calculateDroneGradientUsingDepth();
    calculateOrientationUsingColor();
    
    
    
    
    
    
    //    contourFinder.size();
    
#ifdef USE_TWO_KINECTS
    kinect2.update();
#endif
}




//--------------------------------------------------------------
void ofApp::draw() {
    
    drawDebugView();
    
    
    ofPopMatrix();
    ofDrawBitmapString( ofToString(record) , 20, 400);
    
    //drawSecondWindow();
    
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
            kinect.setCameraTiltAngle(kinectAngle); // go back to prev tilt
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
            kinectAngle++;
            if(kinectAngle>30) kinectAngle=30;
            kinect.setCameraTiltAngle(kinectAngle);
            break;
            
        case OF_KEY_DOWN:
            kinectAngle--;
            if(kinectAngle<-30) kinectAngle=-30;
            kinect.setCameraTiltAngle(kinectAngle);
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




void ofApp::calculateAngle( vector< Scalar > grads )
{
    
    //float angle = 0 ;
    //int switchAt = 0 ;
    
    //switchAt.clear();
    //angle.clear();
    
    for ( int  i = 0 ; i < grads.size() ; i++ )
    {
        
        if ( ( grads[i].val[0] > 0 && grads[i + 1].val[0] < 0 )  || (grads[i].val[0] < 0 && grads[i + 1].val[0] > 0 )  )
        {
            switchAt.push_back(i);
            
            float actualAngle = ofMap( float(i) / float ( grads.size()  ) , 0, 1, 0, 180);
            cout<<endl <<"angle: "<<actualAngle ;
            
            angle.push_back( float (actualAngle));
            cout<<"switch at: "<< i;
            
        }
        
    }
    
}



double ofApp::medianMat(cv::Mat Input)
{
    Input = Input.reshape(0,1); // spread Input Mat to single row
    std::vector<double> vecFromMat;
    Input.copyTo(vecFromMat); // Copy Input Mat to vector vecFromMat
    std::nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 2, vecFromMat.end());
    return vecFromMat[vecFromMat.size() / 2];
}






void ofApp::calculateDroneGradientUsingDepth()
{
    
    droneMats.clear();
    globalDroneAverages.clear();
    globalGradients.clear();
    
    switchAt.clear();
    angle.clear();
    dronePositionVector.clear();
    
    
    
    
    for ( int i = 0 ; i < droneRects.size() ; i++ )
    {
        //droneMats[i] = kinectThresholdMat(droneRects[i]);
        Mat tempMat;
        kinectThresholdMat(droneRects[i]).copyTo(tempMat);
        cv::Point dronePos =  droneRects[i].tl();
        
        
        
        if ( tempMat.cols > 2 * tempMat.rows )              // to ensure that only the drone shows
        {
            dronePositionVector.push_back(dronePos);
            droneMats.push_back(tempMat);

        }
        
    }
    
    
    for ( int  i = 0 ; i < droneMats.size() ; i ++)
    {
        
        vector< Scalar > droneAverages;
        vector<Scalar >  localGradients;
        vector<Mat> droneAverageMats;
        droneAverages.clear();
        localGradients.clear();
        droneAverageMats.clear();
        droneWC.clear();
        
        //stripMedianValues.clear();
        
        
        
        Scalar oldAvg(0);
        Scalar curAvg;
        Scalar gradient;
        float tempAngle;
        
        
        cout<<endl<< "gradient: ";
        
        for ( int j = 0 ; j< droneMats[i].cols- int(stripWidth)  ; j+= int(stripWidth) )
        {
            Mat tempMat;
            Mat tempHistogram;
            
            droneMats[i]( cv::Rect(j,0,int(stripWidth) ,droneMats[i].rows-1)).copyTo(tempMat);
            
            blur(tempMat, 3);
            //curAvg = mean(tempMat);           // change of plans trying to use median
            
            // Scalar tempStrip;
            curAvg.val[0] =  medianMat(tempMat);
            // curAvg = tempStrip;
            droneAverages.push_back( curAvg );
            
            gradient = curAvg - oldAvg;
            
            
            if ( j!=0 )
                localGradients.push_back(gradient);
            //droneAverages.push_back(  mean(droneMats[i]( cv::Rect(j,0,5,droneMats[i].rows) ) ) ) ;
            
            
            oldAvg = curAvg;
            
        }
        calculateAngle(localGradients);
        globalGradients.push_back(localGradients);
        globalDroneAverages.push_back(droneAverages);
        
    }
    
    
    if ( droneMats.size()>0)
    {
        // calculate drone in world coordinate
        
        for ( int  i = 0  ; i< droneMats.size() ; i ++)
        {
            ofVec3f cvg = calculateWorldCoordiante( dronePositionVector[i].x +  droneMats[i].cols/2 , dronePositionVector[i].y +  droneMats[i].rows/2  );   //need to fix, find droneMat actual position
            
            cout<<endl <<"point in space:  "<<cvg.x<<"  "<<cvg.y<<"  "<<cvg.z << endl;
            
            droneWC.push_back(cvg);
            
            
        }
        
    }

    
    //cout<<endl<<"size of the vector: " << globalDroneAverages.size() << endl;
}





void ofApp::drawDebugView()
{
    
    
    kinectThresholdedImage.draw(0,0);
    kinectDepthImage.draw(0,480);
    kinect.draw(640, 480, 640, 480);
    
    
    ofPushMatrix();
    ofTranslate(0,0);
    contourFinder.draw();
    ofPopMatrix();
    if ( ofGetFrameNum()%100 ==0 )
        cout<<endl <<"cf size:  " << contourFinder.size();
    
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
    {
        drawMat(droneMats[i], 640, i * 200);
        
        ofSetColor(0, 100,200 );
        ofTranslate(640 , i* 200 );
        ofSetLineWidth(5);
        if ( switchAt.size() >0  && droneMats[i].rows > 0 )
            ofLine(stripWidth * switchAt[i] , 0, stripWidth * switchAt[i], droneMats[i].rows );
        //ofLine(stripWidth  , 0, stripWidth , droneMats[i].rows );
        
        ofSetColor(255);
    }
    
    ofPopMatrix();
    
    if ( droneMats.size() >0 )
    {
        ofDrawBitmapString("number of contours: " + ofToString( contourFinder.size() ), ofGetScreenWidth() - 200, 40);

        
        for ( int  i = 0 ; i< droneMats.size() ; i++  )
        {
            //ofSetColor(0, 100,200 );

            ofDrawBitmapString("angle: " + ofToString( angle[i] ), ofGetScreenWidth() - 200,60 + 20 * i );
            ofDrawBitmapString("switch at: " + ofToString( switchAt[i] ), ofGetScreenWidth() - 200,120 + 20 * i );
            
            if ( droneWC.size() >0 )
            ofDrawBitmapString("drone WC: " + ofToString(  droneWC[i].x) + "  "+ ofToString(  droneWC[i].y) + "  "+ ofToString(  droneWC[i].z ) + "  ", ofGetScreenWidth() - 400, 240 + 20 * i );
            
        }
        
    }
    
    
    
}
void ofApp::setupofxUIGUI()
{
    
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
    gui1->addSlider("minArea", 5, 2000, &minArea);
    gui1->addSlider("maxArea", 2500, 20000, &maxArea);
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



ofVec3f ofApp::calculateWorldCoordiante(int x, int y )
{
    ofVec3f pointInSpace;
    
    //using kinects function
    //pointInSpace  = kinect.getWorldCoordinateAt(x, y);
    
    
    //using function from net
    pointInSpace.z  = kinect.getDistanceAt(x, y);
    float a  = 0.00173667;
    pointInSpace.x = ( x - kinect.width/2 ) * a *  pointInSpace.z;
    pointInSpace.y = ( y - kinect.height/2 ) * a *  pointInSpace.z;
    
    //cout<<endl <<"point in space:  "<<pointInSpace.x<<"  "<< pointInSpace.y<<"  "<<pointInSpace.z << endl;
    return pointInSpace;
    
}



void ofApp::setupInitialParameters()
{
    //setting up the kinect CV images
    nearThreshold = 5;
    farThreshold = 250;
    minArea = 400;
    maxArea = 9000;
    threshold = 1;
    persistence = 15;
    maxDistance = 32;
    
    kLeftThreshold = -1000;
    kRightThreshold = 1000;
    kTopThreshold = 1000;
    kBottomThreshold = -1000;
    kFrontThreshold = 0;
    kBackThreshold = 6000;
    
    colorMapping =100;
    

}



void ofApp::drawSecondWindow()
    {
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



void ofApp::calculateOrientationUsingColor()
{
    droneColorMats.clear();
    
    if ( droneMats.size()> 0)
    {
        for ( int i = 0 ; i < droneMats.size() ; i++ )
        {
            //droneMats[i] = kinectThresholdMat(droneRects[i]);
            Mat tempMat;
            
            droneColorMat(droneRects[i]).copyTo(tempMat);
            droneColorMats.push_back(tempMat);
            
        }
        
        
        
        
        
        
        
    }
    
    
    
}



