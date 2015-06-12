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
    debugWindow.setup("Drone Space", 0, 0, 1920, 1080, false);
    
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
    mesh.clear();
    mesh.setMode(OF_PRIMITIVE_LINE_STRIP);
   // mesh.setMode(OF_PRIMI)
    //dronePathVector.clear();
    
    setupofxUIGUI();
    
    ofVec3f testPoint1 = ofVec3f(0,0,0);
    ofVec3f testPoint2 = ofVec3f(400,400,400);
    //mesh.addVertex(testPoint1);
    //mesh.addVertex(testPoint2);
    
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofBackground(100, 100, 100);
    //openCv params
    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    
    
    
    kinect.update();
    
    contourFinderFunction();
    calculateDroneGradientUsingDepth();
    calculateOrientationUsingColor();
    
    
    //    contourFinder.size();
    
#ifdef USE_TWO_KINECTS
    kinect2.update();
#endif
}




//--------------------------------------------------------------
void ofApp::draw() {
    ofBackground(0);
    
    easyCam.begin();
    drawSecondWindow();
    easyCam.end();
    
    //ofDrawBitmapString( ofToString(record) , 20, 400);
    
    debugWindow.begin();
    ofBackground(128,128);
    drawDebugView();
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
    
    for ( int  i = 1 ; i < grads.size() -1 ; i++ )      /// ignoring the first and last 1
    {
        
        if ( ( grads[i].val[0] > 0 && grads[i + 1].val[0] < 0 )  || (grads[i].val[0] < 0 && grads[i + 1].val[0] > 0 )  )
        {
            switchAt.push_back(i);
            
            float actualAngle = ofMap( float(i) / float ( grads.size()  ) , 0, 1, 0, 180);
            //cout<<endl <<"angle: "<<actualAngle ;
            
            angle.push_back( float (actualAngle));
            //cout<<"switch at: "<< i;
            
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
    droneRectsFiltered.clear();
    
    
    
    
    
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
            droneRectsFiltered.push_back(droneRects[i]);
            
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
        
        
        //cout<<endl<< "gradient: ";
        
        for ( int j = 0 ; j< droneMats[i].cols- int(stripWidth)  ; j+= int(stripWidth) )
        {
            Mat tempMat;
            Mat tempHistogram;
            
            droneMats[i]( cv::Rect(j,droneMats[i].rows/4,int(stripWidth) ,droneMats[i].rows / 2)).copyTo(tempMat);
            
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
            
            int scaleXY = 5;
            int scaleZ = 10;
            droneWC.push_back(cvg);
            cvg.z = (cvg.z-2000) /scaleZ;
            cvg.y = -cvg.y/scaleXY;
            cvg.x = -cvg.x/scaleXY;

            if ( cvg.x !=0.0f && cvg.y !=0.0f && cvg.z !=0.0f )
                mesh.addVertex(cvg);
            //dronePathVector.push_back(cvg);
            
            
            
            
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
        drawMat(droneMats[i], 640, i * 200 + 20 );
        drawMat(droneColorMats[i], 640+ 300 , i * 200 + 20 );
        /*
        for ( int i = 0;i < color_id.size() ; i++ )
        {
            ofDrawBitmapString( ofToString(  color_id[i] ) , 640+300 + 20 * i , 400 );
        }
         */
        
        
        ofPushMatrix();
        ofSetColor(0, 100,200 );
        ofTranslate(640 , i* 200 );
        ofSetLineWidth(5);
        if ( switchAt.size() >0  && droneMats[i].rows > 0 )
            ofLine(stripWidth * switchAt[i] , 0, stripWidth * switchAt[i], droneMats[i].rows );
        ofTranslate(300, 0);
        
        if ( global_changes[i].size()> 0 )
            ofLine(global_changes[i][0]  , 0, global_changes[i][0] , droneColorMats[i].rows );
        
        //ofLine(stripWidth  , 0, stripWidth , droneMats[i].rows );
        ofPopMatrix();
        
        ofSetColor(255);
    }
    
    ofPopMatrix();
    //ofDrawBox(400,400, -300, 300, 300, 300);

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
    
    gui1->addSlider("strip width", 2, 20,  &stripWidth );
    
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
        ofPushMatrix();
    
        ofRotateY(ofGetFrameNum() * 0.05);
    //creating bounding box
    //drawDronePath();
    ofSetColor( 255 );
    ofNoFill();
    ofSetLineWidth(1);
    ofDrawBox(0,0,0 , bs,bs,bs);
    ofLine(0, 0, -bs/2 , 0,0,bs/2);
    ofLine(0, -bs/2,0 , 0, bs/2 , 0 );
    ofLine( -bs/2,0,0 , bs/2 ,0 ,0 );
        
    // creating drone path
    ofPushMatrix();
    ofSetColor(20, 150, 200,200);
    ofSetLineWidth(4);
    mesh.draw();
    ofPopMatrix();
        
    ofPushMatrix();
     //   ofVec3f currentDronePosition = mesh.getVertex(mesh.getNumVertices());
    
    int numVertices = mesh.getNumVertices();
    if ( numVertices > 3)
        {
        ofVec3f currentDronePosition = mesh.getVertex(numVertices -1 );
            
        
        if( droneMats.size()  > 0  )
            {
            //ofRotate(angle[0], 0, 1, 0);
            ofNoFill();
            ofSetColor(255, 0,0 );
            ofPushMatrix();
            ofTranslate(currentDronePosition.x, currentDronePosition.y, currentDronePosition.z);
            ofRotateY( previousAngle/2 + angle[0]/2 );   //taking average of two
            ofDrawBox( 0,0,0 , droneW,droneH,droneW );
            ofPushMatrix();
            
            previousAngle = angle[0];
            }
            
        ofPopMatrix();
                }
    
        
        ofSetColor( 255);
        ofPopMatrix();
    }


/*
void ofApp::calculateOrientationUsingColor()
{
    droneColorMats.clear();
    
    if ( droneRectsFiltered.size()> 0)
    {
        for ( int i = 0 ; i < droneRectsFiltered.size() ; i++ )
        {
            //droneMats[i] = kinectThresholdMat(droneRects[i]);
            Mat tempMat;
            droneColorMat(droneRectsFiltered[i]).copyTo(tempMat);
            droneColorMats.push_back(tempMat);
            
        }
        
    
        
    }
    
    
    
}


*/


void ofApp::calculateOrientationUsingColor()
{
    droneColorMats.clear();
    global_changes.clear();
    
    //Define Color Ranges
    //Scalar h_min0 = Scalar(0,100,150) ; Scalar h_max0 = Scalar(255,255,255);
    Scalar h_min1 = Scalar(0,100,162) ; Scalar h_max1 = Scalar(18,204,255);  // red 0+1
    Scalar h_min2 = Scalar(26,107,42) ; Scalar h_max2 = Scalar(36,204,255); // yellow
    Scalar h_min3 = Scalar(82,56, 42) ; Scalar h_max3 = Scalar(109,111,255);        //blue
    Scalar h_min4 = Scalar(50,16, 102) ; Scalar h_max4 = Scalar(90,102,172);        //green
    
    
    if ( droneRectsFiltered.size()> 0)
    {
        for ( int i = 0 ; i < droneRectsFiltered.size() ; i++ )
        {
            //droneMats[i] = kinectThresholdMat(droneRects[i]);
            Mat tempMat;
            droneColorMat(droneRectsFiltered[i]).copyTo(tempMat);
            droneColorMats.push_back(tempMat);
            
        }
        
        for( int i = 0 ; i < droneRectsFiltered.size() ; i++ )
        {
            Mat HSV;
            color_id.clear();
            
            cvtColor(droneColorMats[i],HSV,CV_BGR2HSV);
                        std::vector<int> change_indexes;
            int old_color;
            
            
            int startValue = int(stripWidth) * 2 ;
            for ( int j = startValue ; j< droneColorMats[i].cols- int(stripWidth)  ; j+= int(stripWidth) )
            {
                Mat tempMat;
                Mat threshold0, threshold1, threshold2, threshold3, threshold4 ;
                std::vector<int> n_colors;
                int color;
                
            
        
                HSV( cv::Rect(j, droneMats[i].rows/4,int(stripWidth) ,droneMats[i].rows/2)).copyTo(tempMat);
                
               // inRange(tempMat, h_min0, h_max0, threshold0);
                inRange(tempMat, h_min1, h_max1, threshold1);
                inRange(tempMat, h_min2, h_max2, threshold2);
                inRange(tempMat, h_min3, h_max3, threshold3);
                inRange(tempMat, h_min4, h_max4, threshold4);
                
                
                n_colors.push_back(countNonZero(threshold1) );
                n_colors.push_back(countNonZero(threshold2));
                n_colors.push_back(countNonZero(threshold3));
                n_colors.push_back(countNonZero(threshold4));
                
                color = max_element(n_colors.begin(),n_colors.end()) - n_colors.begin();
                
                if(color != old_color && j > startValue)
                    change_indexes.push_back(j);
                
                color_id.push_back(color);
                old_color = color;
                cout << "Color "<< j << " is:" << color<<endl;
                
            }
            
            
            int max = 0;
            int second_common = -1;
            int most_common = -1;
            int old_mc = -1;
            int max_old;
            map<int,int> m;
            for (auto vi = color_id.begin(); vi != color_id.end(); vi++) {
                m[*vi]++;
                if (m[*vi] > max) {
                    max = m[*vi];
                    //if(max_old!=max)
                    //   second_common = most_common;
                    most_common = *vi;
                    if(most_common != old_mc)
                        second_common = most_common;
                    old_mc = most_common;
                    
                    max_old = max;
                }
            }
            
            cout<<endl<<"Most Common:" << most_common << endl<<"Second Common:" << second_common;
            
            
            //Define global_changes as vector< vector< int > > ;
            global_changes.push_back(change_indexes);
            

        }
    }
}



void ofApp::drawDronePath()
{
  
    ofSetColor( 255,0,0 );
    ofFill();
    ofDrawBox(400,400, 0, 300, 300, 300);
    ofPushMatrix();
    mesh.draw();
    ofPopMatrix();
    ofSetColor( 255);

    
    
}



void ofApp::contourFinderFunction()
{
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

}




