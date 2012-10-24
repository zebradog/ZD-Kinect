/*
 TuioKinect - A simple TUIO hand tracker for the Kinect 
 Copyright (c) 2010 Martin Kaltenbrunner <martin@tuio.org>
 Modified by Matt Cook <matt@lookitscook.com>
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include "TuioKinect.h"

ofxCvKalman *tuioPointSmoothed[32];

TuioPoint updateKalman(int id, TuioPoint tp) {
	if (id>=16) return NULL;
	if(tuioPointSmoothed[id*2] == NULL) {
		tuioPointSmoothed[id*2] = new ofxCvKalman(tp.getX());
		tuioPointSmoothed[id*2+1] = new ofxCvKalman(tp.getY());
	} else {
		tp.update(tuioPointSmoothed[id*2]->correct(tp.getX()),tuioPointSmoothed[id*2+1]->correct(tp.getY()));
	}
	
	return tp;
}

void clearKalman(int id) {
	if (id>=16) return;
	if(tuioPointSmoothed[id*2]) {
		delete tuioPointSmoothed[id*2];
		tuioPointSmoothed[id*2] = NULL;
		delete tuioPointSmoothed[id*2+1];
		tuioPointSmoothed[id*2+1] = NULL;
	}
}

/*********************************/
/* The UserGenerator Callbacks
 /*********************************/
// Callback: A user was found
void XN_CALLBACK_TYPE NewUserDetected(UserGenerator& rGenerator,XnUserID nID ,void* pCookie) {
	printf("New User %d\n", nID);
}

// Callback: An existing user was lost
void XN_CALLBACK_TYPE UserLostDetected(UserGenerator& rGenerator ,XnUserID nID ,void* pCookie) {
	printf("Lost user %d\n", nID);
}

//--------------------------------------------------------------
void TuioKinect::setup()
{
	ofSetWindowTitle("TuioKinect");
	
	ofSetVerticalSync(true);
	
	nRetVal = XN_STATUS_OK;
	
	std::string sFile = ofToDataPath("openni/config/ofxopenni_config.xml",true);
	
    /*****************
	 /* Create context
	 /****************/
	context = new Context;
	
	/*****************
	 /* initialise context from XML
	 /****************/
	nRetVal = context->InitFromXmlFile(sFile.c_str(), &errors);
	
	if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		OF_EXIT_APP(0);
	}
	else if (nRetVal != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		OF_EXIT_APP(0);
	}
	
	/*****************
	 /* create image Generator
	 /****************/
	nRetVal = context->FindExistingNode(XN_NODE_TYPE_IMAGE, imageGenerator);
	CHECK_RC(nRetVal, "Find image generator");
	
	/*****************
	 /* create Depth Generator
	 /****************/
	nRetVal = context->FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");
	
	/*****************
	 /* create User Generator
	 /****************/
	nRetVal = context->FindExistingNode(XN_NODE_TYPE_USER, userGenerator);
	CHECK_RC(nRetVal, "Find user generator");
	
	/*****************
	 /* create UserGenerator Callbacks
	 /****************/
	XnCallbackHandle user_cb_handle;
	userGenerator.RegisterUserCallbacks(NewUserDetected,UserLostDetected,this ,user_cb_handle);
	
	
	//sync the viewport
	nRetVal = depthGenerator.GetAlternativeViewPointCap().SetViewPoint(imageGenerator);
	CHECK_RC(nRetVal, "SetViewPoint");
	
	//XnStatus nRetVal = context->SetGlobalMirror(true);
	//CHECK_RC(nRetVal, "toggleMirror");
	
    imagePixels = new unsigned char[640 * 480*3];	

	for(int i= 0; i<16; i++) userTracked[i] = false;
	
	nRetVal = context->StartGeneratingAll();	
	
	depthImage.allocate(640, 480);
	thresholdImage.allocate(640, 480);
	colorImage.allocate(640, 480);
	grayImage.allocate(640, 480);

	thresholdOffset = 150;
	
	TuioTime::initSession();	
	tuioServer = new TuioServer("192.168.0.108",3333); //external client
	//tuioServer = new TuioServer(); //local client, localhost

	tuioServer->setSourceName("TuioKinect");
	tuioServer->enableObjectProfile(false);
	tuioServer->enableBlobProfile(false);
	
	for (int i=0;i<32;i++)
		tuioPointSmoothed[i] = NULL;
	
	ofSetFrameRate(30);
	
}

//--------------------------------------------------------------
void TuioKinect::update()
{

	/****************************
	 /* update the OpenNI pipeline
	 /****************************/
	
	nRetVal = context->WaitAndUpdateAll();
	
	if (nRetVal != XN_STATUS_OK) {
        printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
        return;
    }
	
	for(int i= 0; i<16; i++) userTracked[i] = false;
	
	// get depthData
	depthGenerator.GetMetaData(depthMetaData);
	unsigned short* pix = (unsigned short*) depthMetaData.Data();
	
	// get UserPixels
    userGenerator.GetUserPixels(0, sceneMetaData);
	unsigned short *userPix = (unsigned short*)sceneMetaData.Data();
	
	// get imageData
	xn::ImageMetaData imageMetaData;
	imageGenerator.GetMetaData(imageMetaData);
	unsigned char *imagePix = (unsigned char*) imageMetaData.Data();	
	float deepest = 0.0;
	
	thresholdImage.mirror(false, true);
	
	unsigned char * tpix = thresholdImage.getPixels();
	unsigned char * dpix = depthImage.getPixels(); 	
	
	colorImage.setFromPixels(imagePixels, 640, 480);
	colorImage.mirror(false, true);
	colorImage.convertToGrayscalePlanarImage(grayImage, 0);
	
	
	for(int i= 0; i<16; i++) userGenerator.GetCoM(i,userPos[i]); //get center of mass for up to 16 users
	
	//determine the deepest (closest.. ?) point of the user silhouette
	for (int i =0 ; i < 640 * 480; i++) {
		if(userPix[i]>0 && pix[i]>deepest) deepest = pix[i];
	}
	
	//loop over every pixel.
	for (int i =0 ; i < 640 * 480; i++) {
		
                //display the silhouette (dpix[])... I think
		char depthVal = CLAMP(255-pix[i] / (deepest/(220)), 0, 255);
		
		imagePixels[i*3] = imagePix[i*3];
		imagePixels[i*3+1] = imagePix[i*3+1];
		imagePixels[i*3+2] = imagePix[i*3+2];
		
		if (userPix[i]>0){
			dpix[i] = depthVal;
		}else{
			dpix[i] = 0;
		}
		//set the hand blob pixel (tpix[]) to white if in the silhouete (userPix[]) 
		// and is closer than the center of mass (userPos[userPix[i]].Z) plus a threshold (thresholdOffset)
                //otherwise set it to black
		if (userPix[i]>0 && pix[i] < userPos[userPix[i]].Z - thresholdOffset) {
			tpix[i] = 255;
		}else{
			tpix[i] = 0;
		}
    }
	
	//update the cv image
	depthImage.flagImageChanged();
	thresholdImage.flagImageChanged();
	
	contourFinder.findContours(thresholdImage, (640*480)/512, (640*480)/8, 32, false, false);
	
	TuioTime frameTime = TuioTime::getSessionTime();
	tuioServer->initFrame(frameTime);
	
	std::vector<ofxCvBlob>::iterator blob;
	for (blob=contourFinder.blobs.begin(); blob!= contourFinder.blobs.end(); blob++) {
		float xpos = (*blob).centroid.x;
		float ypos = (*blob).centroid.y;
				
		TuioPoint tp(xpos/640,ypos/480);
		
		//if ((tp.getY() > 0.8) && (tp.getX()>0.25) && (tp.getX()<0.75)) continue;
		
		TuioCursor *tcur = tuioServer->getClosestTuioCursor(tp.getX(),tp.getY());
		if ((tcur==NULL) || (tcur->getDistance(&tp)>0.1)) { 
			tcur = tuioServer->addTuioCursor(tp.getX(), tp.getY());
			updateKalman(tcur->getCursorID(),tcur);
		} else {
			TuioPoint kp = updateKalman(tcur->getCursorID(),tp);
			tuioServer->updateTuioCursor(tcur, kp.getX(), kp.getY());
		}
	}

	tuioServer->stopUntouchedMovingCursors();
	
	std::list<TuioCursor*> dead_cursor_list = tuioServer->getUntouchedCursors();
	std::list<TuioCursor*>::iterator dead_cursor;
	for (dead_cursor=dead_cursor_list.begin(); dead_cursor!= dead_cursor_list.end(); dead_cursor++) {
		clearKalman((*dead_cursor)->getCursorID());
	}
	
	tuioServer->removeUntouchedStoppedCursors();
	tuioServer->commitFrame();
}

//--------------------------------------------------------------
void TuioKinect::draw()
{
	ofBackground(100, 100, 100);	
	
	ofSetColor(255, 255, 255);
	ofFill();
	
	depthImage.draw(15, 15, 400, 300);
	grayImage.draw(425, 15, 400, 300);
	thresholdImage.draw(15, 325, 400, 300);
	contourFinder.draw(15, 325, 400, 300);

	//draw hand circles
	ofSetColor(255,255, 255);
	ofRect(425, 325, 400, 300);
	std::list<TuioCursor*> alive_cursor_list = tuioServer->getTuioCursors();
	std::list<TuioCursor*>::iterator alive_cursor;
	for (alive_cursor=alive_cursor_list.begin(); alive_cursor!= alive_cursor_list.end(); alive_cursor++) {
		TuioCursor *ac = (*alive_cursor);
		
		int xpos = 425+ac->getX()*400;
		int ypos = 325+ac->getY()*300;
		ofSetColor(0, 0, 0);
		ofCircle(xpos,ypos ,15);
		char idStr[32];
		sprintf(idStr,"%d",ac->getCursorID());
		ofDrawBitmapString(idStr, xpos+15, ypos+20);
		
	}
	
}

//--------------------------------------------------------------
void TuioKinect::exit(){
	
}

//--------------------------------------------------------------
void TuioKinect::keyPressed (int key)
{}

//--------------------------------------------------------------
void TuioKinect::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void TuioKinect::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void TuioKinect::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void TuioKinect::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void TuioKinect::windowResized(int w, int h)
{}