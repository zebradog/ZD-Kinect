/*
 TuioKinect - A simple TUIO hand tracker for the Kinect 
 Copyright (c) 2010 Martin Kaltenbrunner <martin@tuio.org>
 
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

#ifndef _TUIO_KINECT
#define _TUIO_KINECT

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCvKalman.h"
#include "ofxCvHaarFinder.h"
#include "TuioServer.h"
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>

#ifdef USE_GUI 
#include "ofxSimpleGuiToo.h"
#endif

#define CHECK_RC(rc, what)								\
if (rc != XN_STATUS_OK)									\
{														\
printf("%s failed: %s\n", what, xnGetStatusString(rc));	\
}

using namespace TUIO;
using namespace xn;

class TuioKinect : public ofBaseApp
{

	public:

		void setup();
		void update();
		void draw();
        void exit();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);

		ofxCvGrayscaleImage	thresholdImage;
		ofxCvGrayscaleImage	depthImage;
		ofxCvColorImage		colorImage;
		ofxCvGrayscaleImage	grayImage;
		
		ofxCvContourFinder 	contourFinder;
		TuioServer *tuioServer;

		int thresholdOffset; //in millimeters
	
		Context * context;
	
		DepthGenerator depthGenerator;
		UserGenerator userGenerator;
		ImageGenerator	imageGenerator;
	
		DepthMetaData depthMetaData;
		SceneMetaData sceneMetaData;
	
		unsigned char * imagePixels;
	
		xn::EnumerationErrors errors;	
		XnStatus nRetVal;
	
		XnPoint3D userPos[16];
		bool userTracked[16];

};

#endif
