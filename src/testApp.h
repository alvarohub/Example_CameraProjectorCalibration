#pragma once

#include "ofMain.h"
#include "ofxCv.h"

// ==================================================================
// WE NEED TO DEFINE HERE the size of the computer screen and the projector screen. This cannot be done using ofGetScreenWidth() and the like
// because in order to properly calibrate camera and projector, we need OF in "extended desktop mode".
// Of course, it would be possible to use a single screen (the projector) to perform the whole calibration (but we need a way to check if the 
// printed pattern is visible - SOUND?).

#define COMPUTER_DISP_WIDTH 1440
#define COMPUTER_DISP_HEIGHT 900

// RESOLUTION OF CAMERA AND PROJECTOR: 
// - ideally, this should also be in a file (in particular if we want to calibrate several cameras/projectors). 
// Resolution of the projector:
#define PROJ_WIDTH   800 
#define PROJ_HEIGHT 600

// Resolution of the camera (or at least resolution at which we want to calibrate it):
#define CAM_WIDTH 640
#define CAM_HEIGHT 480

// ==================================================================

enum CalibState {CAMERA_ONLY, CAMERA_AND_PROJECTOR_PHASE1, CAMERA_AND_PROJECTOR_PHASE2, AR_DEMO};

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	
	ofVideoGrabber cam;
	ofImage undistorted;
    
 	ofPixels previous; 
	ofPixels diff;
	float diffMean;
	
	float lastTime;
	bool active, displayAR;
    bool newBoardAquired;
    bool dynamicProjection;
    bool dynamicProjectionInside;
	
    // For tests:
    ofVideoPlayer 		eyeMovie;
    
    // VARIABLES and METHODS THAT SHOULD BELONG TO A STEREO-CALIBRATION OBJECT (probably using multiple cameras and projectors)
	ofRectangle viewportComputer, viewportProjector;
    ofxCv::Calibration calibrationCamera, calibrationProjector;
    CalibState stateCalibration;
    
    //Extrinsics (should belong to the Stereo calibration object)
    cv::Mat rotCamToProj, transCamToProj; // in fact, there should be one pair of these for all the possible pairs camera-projector, camera-camera, projector-projector. 
    string extrinsics;
    void saveExtrinsics(string filename, bool absolute = false) const;
    void loadExtrinsics(string filename, bool absolute = false);
    
};
