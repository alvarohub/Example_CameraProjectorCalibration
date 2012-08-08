#include "testApp.h"

using namespace ofxCv;
using namespace cv;

const float diffThreshold = 3.0; // maximum amount of movement between successive frames (must be smaller in order to add a board)
const float timeThreshold = 1.0; // minimum time between snapshots (seconds)

const int preCalibrateCameraTimes = 20; // this is for calibrating the camera BEFORE starting projector calibration. 
const int startCleaningCamera = 8; // start cleaning outliers after this many samples (10 is ok...). Should be < than preCalibrateCameraTimes
const float maxErrorCamera=0.2;

const float maxErrorProjector=0.25;
const int startCleaningProjector = 8;
const int startDynamicProjectorPattern=5; // after this number of projector/camera calibration, the projection will start following the 
// printed pattern to facilitate larger exploration of the field of view. If this number is larger than minNumGoodBoards, then this will never 
// happen automatically. 
const int minNumGoodBoards=20; // after this number of simultaneoulsy acquired "good" boards, IF the projector total reprojection error is smaller than a certain threshold, we end calibration (and move to AR mode automatically)


// ****** INITIAL MODE ******
CalibState InitialMode=AR_DEMO;//CAMERA_AND_PROJECTOR_PHASE1;//CAMERA_ONLY; //;// AR_DEMO;

void testApp::setup() {
	ofSetVerticalSync(true);
    
    cam.listDevices();
	cam.initGrabber(CAM_WIDTH, CAM_HEIGHT);
	imitate(undistorted, cam);
	imitate(previous, cam);
	imitate(diff, cam);
    
#ifdef MOVIE_PLAY
    eyeMovie.loadMovie("movies/ojo.mov");
	eyeMovie.play();
#endif
    
    // (1) Load the pattern data to recognize, for camera and for projector:
    calibrationCamera.loadCalibrationShape("settingsPatternCamera.yml");
    calibrationProjector.loadCalibrationShape("settingsProjectionPatternPixels.yml");
    
    // (2) Setting the imager size:
    // Note: in case of projector, this is NOT the size of the acquired camera image: this needs to be set manually (of from a file)
    //       To make things more clear, we will do this also for objects of type camera. 
    calibrationCamera.setImagerResolution(cv::Size(CAM_WIDTH, CAM_HEIGHT));
    calibrationProjector.setImagerResolution(cv::Size(PROJ_WIDTH, PROJ_HEIGHT));
	
    // (3) Define viewports for each display:
    // ATTENTION: I cannot use ofGetScreenWidth() and the like, because we need to put OF in "extended desktop" mode!
    viewportComputer.set(0,0,COMPUTER_DISP_WIDTH, COMPUTER_DISP_HEIGHT);
    viewportProjector.set(COMPUTER_DISP_WIDTH,0,PROJ_WIDTH, PROJ_HEIGHT);
    
    // Other graphics settings:
    ofBackground(0,0,0);
    ofEnableAlphaBlending();
    ofEnableSmoothing();
    ofSetCircleResolution(30);
    
    // INITIAL MODE:
    initialization(InitialMode);
}

void testApp::initialization(CalibState initialmode) {
    // Initialization of the state machine (note: THIS PROGRAM will mostly be a METHOD of a STEREO CALIBRATION CLASS)
	lastTime = 0;
	
    manualAcquisition = false; 
    manualGetImage=false;
    
    newBoardAquired=false;
    dynamicProjection=false;
    dynamicProjectionInside=false;
    displayAR=false;
    
    switch (initialmode) {
        case CAMERA_ONLY: // (1) calibrate camera before anything else
            stateCalibration=CAMERA_ONLY;
            break;
            
        case CAMERA_AND_PROJECTOR_PHASE1: // (2) load pre-calibrated camera and start calibration projector and computing extrinsics
        case CAMERA_AND_PROJECTOR_PHASE2: 
            calibrationCamera.load("calibrationCamera.yml");
            
            // Attention! the load method, also load a set of image/object points! I don't need that: it will break the SIMULTANEOUS stereo calib 
            calibrationCamera.deleteAllBoards();
            
            stateCalibration=CAMERA_AND_PROJECTOR_PHASE1;
            break;
            
        case AR_DEMO: // camera, projector and extrinsics are loaded from file:
            calibrationCamera.load("calibrationCamera.yml");
            // attention! load also load a set of image/object points! I don't need that:
            calibrationCamera.deleteAllBoards();
            
            calibrationProjector.load("calibrationProjector.yml");
            // attention! load also load a set of image/object points! I don't need that:
            calibrationProjector.deleteAllBoards();
            
            loadExtrinsics("CameraProjectorExtrinsics.yml");
            
            stateCalibration=AR_DEMO;
            
            break;
            
        default:
            break;
    }
}

void testApp::update() {
	cam.update();
#ifdef MOVIE_PLAY
    eyeMovie.idleMovie();
#endif
    
	if(cam.isFrameNew()) {		
		Mat camMat = toCv(cam); // current image to process
        
		Mat prevMat = toCv(previous);
		Mat diffMat = toCv(diff);
		absdiff(prevMat, camMat, diffMat);	
        diffMean = mean(Mat(mean(diffMat)))[0];
		camMat.copyTo(prevMat);
        
		float curTime = ofGetElapsedTimef();
        
        //(a) First, add and preprocess the image (this will threshold, color segmentation, etc as specified in the pattern 
        // calibration files). This is done regardless of the manual acquisition mode, because we want to be able to check 
        // the pre-precess images. 
        calibrationCamera.addImageToProcess(camMat);
        calibrationProjector.addImageToProcess(camMat);
        
        //(b) detect the patterns, and perform calibration or stereo calibration:
        switch(stateCalibration) {
                // CAMERA ONLY ---------------------------------------------------------------------------------------------
            case CAMERA_ONLY:
                
                if(( manualAcquisition && manualGetImage) ||
                   (!manualAcquisition && (curTime - lastTime > timeThreshold && diffMean < diffThreshold) )) {
                    
                    if (calibrationCamera.generateCandidateImageObjectPoints()) {
                        
                        // Add the candidate image/object points to the board vector list:
                        calibrationCamera.addCandidateImagePoints();
                        calibrationCamera.addCandidateObjectPoints(); 
                        
                        calibrationCamera.calibrate(); // this use all the previous boards stored in vector arrays, AND recompute each board rotations and tranlastions in the board vector list.
                        cout << "Camera re-calibrated" << endl;
                        
                        // Clean the list of boards using reprojection error test:
                        if(calibrationCamera.size() > startCleaningCamera) {
                            calibrationCamera.clean(maxErrorCamera);
                        }
                        
                        // For visualization: undistort image from camera:
                        /*
                         if(calibrationCamera.size() > 0) {
                         calibrationCamera.undistort(toCv(cam), toCv(undistorted));
                         undistorted.update(); // << this is confusing for me (how the actual data is updated, etc). 
                         }
                         */
                        
                        // Test end CAMERA_ONLY calibration:
                        // (note: puting this after cleaning, means we want a certain number of "good" boards before moving on)
                        if ((calibrationCamera.size()>=preCalibrateCameraTimes)) {
                            // Save latest camera calibration:
                            calibrationCamera.save("calibrationCamera.yml");
                            
                            // DELETE all the object/image points, because now we are going to get them for both the projector and camera:
                            calibrationCamera.deleteAllBoards();
                            
                            // Start stereo calibration (for camera and projector):
                            stateCalibration=CAMERA_AND_PROJECTOR_PHASE1; 
                        }
                        
                        // Reset timer, as well as manual flag:
                        lastTime = curTime;
                        manualGetImage=false;
                    }
                    
                }
                break;
                
                // CAMERA AND PROJECTOR ---------------------------------------------------------------------------------------------
                // Notes: In case of camera/projector calibration, we need to proceed in TWO PHASES to give time to the projected image 
                // to refresh before trying to detect it (in case of "dynamic projected pattern"). Otherwise we may be detecting the OLD 
                // projected pattern, but using the newer image points (which completely breaks the calibration of course)
                
            case CAMERA_AND_PROJECTOR_PHASE1: 
                // PHASE 1 goal is just to set the projector image points to be projected, so the projector 
                // will project something to be detected in the draw function. The first time, this is using the recorded pattern, but later 
                // (as the projector gets calibrated) we can use some arbitrary points "closer" to the printed pattern.
                // In this later case (dynamic pattern), if the printed pattern is not visible, then we won't go to phase 2. 
                
                cout << " ****** PHASE 1 ********** " << endl;
                
                
                // Dynamic or static projection? :               
                if (calibrationProjector.size()==0) dynamicProjection=false; // this is necessary in case all the board are deleted because 
                // of large reprojection error (even if we FORCED dynamicProjection to true using the keyboard). In that case, there won't be 
                // any board rot/translation computed form the point of view of the projector. 
                else if (calibrationProjector.size()>startDynamicProjectorPattern) dynamicProjection=true; // note that dynamic projection can be set manually too, or using this threshold on the number of boards. 
                
                // Now, set the IMAGE points of the projector, either using a stored pattern, or from the reprojected 3d points of the 
                // printed chessboard.
                if (dynamicProjection) {
                    
                    cout << "USING DYNAMIC PROJECTION PATTERN" << endl;
                    
                    // First, check if we can detect the printed pattern to reajust the projection
                    // IMPORTANT NOTE: we may prefer AVOIDING the manual acquisition test or the timer, so as to move the 
                    // projection in "real time":
                  //  if(( manualAcquisition && manualGetImage) ||
                  //     (!manualAcquisition && (curTime - lastTime > timeThreshold && diffMean < diffThreshold) )) {
                        
                        if  (calibrationCamera.generateCandidateImageObjectPoints()) { // generate image points from the detected pattern, and 
                            //object points from the stored pattern, for the CAMERA.
                            
                            cout << "Printed pattern detected" << endl;
                            
                            // We assume now that the camera is well calibrated: do NOT recalibrate again, simply compute latest board pose:                        
                            calibrationCamera.computeCandidateBoardPose();  
                            
                            // NOTE: we don't add anything to the board vector arrays FOR THE CAMERA (image/object) because we need to be sure
                            // we also get this data for the projector calibration object before calling stereo calibration
                            // However, we can already use the candidate points to show image and reprojection for that board.  
                            
                            // In this case, we can modify the candidate projector image points to follow the printed board if the projector has
                            // been partially calibrated. This is important to effectively explore the "image space" for the projector, and 
                            // improve the calibration. 
                            // IMPORTANT: This can be done using the computed extrinsics, or the board rot/trans computed from the latest
                            // projector calibration; we will use the latest board pose, in camera and projector coordinates, as well as 
                            // the current post in camera coordinates, to deduce the current pose in projector coordinates: this is better than
                            // using the current "global computed" extrinsics (which may not have been yet computed, or recently "cleaned"). 
                            
                            //(make a special function with "displacement" parameter to project inside or outside the printed pattern?):
                            vector<Point3f> auxObjectPoints;
                            Point3f posOrigin, axisX, axisY;  
                            axisX=calibrationCamera.candidateObjectPoints[1]-calibrationCamera.candidateObjectPoints[0];
                            axisY=calibrationCamera.candidateObjectPoints[calibrationCamera.myPatternShape.getPatternSize().width]-calibrationCamera.candidateObjectPoints[0];
                            if (dynamicProjectionInside) 
                                //pattern inside the printed chessboard:
                                posOrigin=calibrationCamera.candidateObjectPoints[0]+(axisX-axisY)*0.5;
                            else
                                // pattern outside the printed chessboard:
                                posOrigin=calibrationCamera.candidateObjectPoints[0]-axisY*(calibrationCamera.myPatternShape.getPatternSize().width-2);
                            
                            auxObjectPoints=Calibration::createObjectPointsDynamic(posOrigin, axisX, axisY, calibrationProjector.myPatternShape);
                            // Note: a method "setCandidateDynamicObjectPoints" is not needed, because the actual candidate OBJECT points will be computed from the camera image. But perhaps it would be better to have it, to avoid calling a static method. 
                            
                            vector<Point2f> followingPatternImagePoints;
                            // Remember: we will use the rot/trans of the PREVIOUS BOARD as stored by the projector calibration object, and
                            // not the (yet not good) extrinsics, which is what we are looking for by the way. So, since we don't use the 
                            // extrinsics, we need to determine the new rot/trans from the camera "delta" motion, which presumably, is quite 
                            // good (camera is well calibrated). Note that even if the final pose in projector coordinates is not good, we don't
                            // care: we are just trying to get the projected point "closer" to the printed pattern to facilitate "exploration"
                            // of the space - points will we will precisely detected with the camera. 
                            
                            Mat Rc1, Tc1, Rc1inv, Tc1inv, Rc2, Tc2, Rp1, Tp1, Rp2, Tp2;
                            // Previous bord position in projector coordinate frame:
                            Rp1=calibrationProjector.boardRotations.back();
                            Tp1=calibrationProjector.boardTranslations.back();
                            // Previous board position in camera coordinate frame:
                            Rc1=calibrationCamera.boardRotations.back();
                            Tc1=calibrationCamera.boardTranslations.back();
                            // Latest board position in camera coordiante frame (not yet in the vector list!!):
                            Rc2=calibrationCamera.candidateBoardRotation;
                            Tc2=calibrationCamera.candidateBoardTranslation;
                            
                            
                            Mat auxRinv=Mat::eye(3,3,CV_32F);
                            Rodrigues(Rc1,auxRinv);
                            auxRinv=auxRinv.inv(); // or transpose, the same since it is a rotation matrix!
                            Rodrigues(auxRinv, Rc1inv);
                            Tc1inv=-auxRinv*Tc1;
                            Mat Raux, Taux;
                            composeRT(Rc2, Tc2, Rc1inv, Tc1inv, Raux, Taux);
                            composeRT(Raux, Taux, Rp1, Tp1, Rp2, Tp2);
                            
                            followingPatternImagePoints=calibrationProjector.createImagePointsFrom3dPoints(auxObjectPoints, Rp2, Tp2); 
                            // Set image points to display:
                            calibrationProjector.setCandidateImagePoints(followingPatternImagePoints);
                            
                            // Then project, and go to phase 2:
                            stateCalibration=CAMERA_AND_PROJECTOR_PHASE2; 
                            
                        } 
                        else {
                            cout << "Printed pattern not visible" << endl;
                            cout << endl << "======= MOVE THE PRINTED PATTERN =====" << endl; 
                            // Note: no need to reset manual acquisition of timer, because we know that we need to look for something. 
                        }
                        
         //           }
                    
                }
                else 
                {
                    cout << "USING FIXED PROJECTION PATTERN" << endl;
                    //(a) Set the candidate points (for projection) using the fixed pattern:
                    calibrationProjector.setCandidateImagePoints(); 
                    
                    // Project, and go to phase 2:
                    stateCalibration=CAMERA_AND_PROJECTOR_PHASE2; 
                }      
                
                break;
                
            case  CAMERA_AND_PROJECTOR_PHASE2: 
                // PHASE 2: here, we will check if BOTH camera and projector patterns are visible in the current acquired image. 
                // IF NOT, then we will revert to phase one:
                cout << " ****** PHASE 2 ********** " << endl;
                
                if(( manualAcquisition && manualGetImage) ||
                   (!manualAcquisition && (curTime - lastTime > timeThreshold && diffMean < diffThreshold) )) {
                    
                    cout << "Trying to detect projected and printed patterns simultaneously:" << endl;
                    
                    // First, detect printed pattern and compute candidate board pose:
                    if (calibrationCamera.generateCandidateImageObjectPoints()) {
                        cout << "Printed pattern detected." << endl;
                        
                        calibrationCamera.computeCandidateBoardPose();  
                        
                        // If this succeeded, use this board pose and the camera to detect the candidate object points for the projector:
                        if (calibrationProjector.generateCandidateObjectPoints(calibrationCamera)) {
                            //Note: generateCandidateObjectPoints compute the candidate objectPoints (if these are detected by the camera), but not the image points. These were assumed to be set already on the projector calibration object (and displayed!). 
                            
                            cout << "Projected pattern detected" << endl;
                            
                            // If the object points for the projector were detected, add those points as well as the image points to the
                            // list of boards image/object for BOTH the camera and projector calibration object, including the rotation and 
                            // translation vector for the camera frame: 
                            
                            // add to CAMERA board list (only for stereo calibration):
                            calibrationCamera.addCandidateImagePoints();
                            calibrationCamera.addCandidateObjectPoints();
                            calibrationCamera.addCandidateBoardPose();
                            
                            // add to PROJECTOR board list (for projector recalibration and stereo calibration):
                            calibrationProjector.addCandidateImagePoints();
                            calibrationProjector.addCandidateObjectPoints();
                            // Note: the rotation and translation vectors for the projector are not yet added: these CANNOT 
                            // properly be computed using PnP algorithm (as calibrationProjector.computeCandidateBoardPose()), because we are 
                            // precisely trying to get the projector instrinsics! This will be done by the projector.calibrate() method...
                            
                            cout << "Re-calibrating projector..." << endl;
                            // PUT THIS IN ANOTHER THREAD????
                            calibrationProjector.calibrate(); // this will recompute the projector instrinsics, as well as the board translation and rotation for all the boards - including the latest one. NOTE: it will also update the candidateBoardRotation and candidateBoardTranslation just because we may want these matrices for drawing (but we don't need to "add" them to the vector lists, because this is already done by the openCV calibration method). 
                            cout << "Projector re-calibrated." << endl;
                            
                            
                            // Cleaning: this needs to be done SIMULTANEOUSLY for projector and camera. 
                            // However, the test is only done on the projector reprojection error if the camera intrinsics are fixed (because the 
                            // stereo calibration will be done by generating image points for the camera based only on the OBJECT POINTS OF THE PROJECTOR)
                            if(calibrationProjector.size() > startCleaningProjector) {
                                calibrationProjector.simultaneousClean(calibrationCamera, maxErrorProjector);
                            }
                            
                            // Now we can run the stereo calibration (output: rotCamToProj and transCamToProj). Note: we call stereo calibration
                            // with FIXED INTRINSICS for both the camera and projector. 
                            // NOTE: perhaps we can start running this after a few projector calibrations????
                            cout << "Performing stereo calibration..." << endl;
                            calibrationProjector.stereoCalibrationCameraProjector(calibrationCamera, rotCamToProj, transCamToProj);
                            cout << "Stereo Calibration performed." << endl;
                            
                            // Everything went fine: go to AR MODE if we finished calibration, or revert to PHASE 1 (and indicate that a "stereo board" was properly aquired)    
                            
                            if (calibrationProjector.size()>minNumGoodBoards) {
                                // SAVE THE INTRINSICS and EXTRINSINCS when TOTAL reprojection error is lower than a certain threshold (this is 
                                // automatically assured if we called simultaneousClean before this lines of code), 
                                // and when we get a sufficiently large amount of boards, and move to AR_DEMO:
                                calibrationProjector.save("calibrationProjector.yml"); 
                                saveExtrinsics("CameraProjectorExtrinsics.yml");
                                stateCalibration=AR_DEMO; 
                            } else {
                                // Otherwise, proceed refining the calibration:
                                stateCalibration=CAMERA_AND_PROJECTOR_PHASE1; 
                                newBoardAquired=true;
                                cout << endl << "======= YOU CAN MOVE THE PRINTED PATTERN TO EXPLORE IMAGE SPACE =====" << endl; 
                                lastTime = curTime;
                                manualGetImage=false;
                            }
                            
                        }  
                        else {
                            cout << "Projected pattern not visible!" << endl;
                            cout << "You need to move the board so that the PROJECTED pattern is visible too." << endl; 
                            // REVERT TO PHASE1
                            stateCalibration=CAMERA_AND_PROJECTOR_PHASE1; 
                            // No need to reset manual acquisition or timer, because we are looking for something new. But we may want, in
                            // case of manual mode, to be able to fix the board before hit a key:
                            manualGetImage=false; 
                        }
                        
                    } else {
                        cout << "Printed pattern not visible!" << endl;
                        cout << "You need to move the board so that the PRINTED pattern is visible." << endl;
                        // REVERT TO PHASE1:
                        stateCalibration=CAMERA_AND_PROJECTOR_PHASE1; 
                        // No need to reset manual acquisition or timer, because we are looking for something new. But we may want, in
                        // case of manual mode, to be able to fix the board before hit a key:
                        manualGetImage=false; 
                    }
                    
                } 
                else { // this means that the timer or image difference is not yet enough, or we didn't choose to manually acquire
                    // the image: just move to phase one to continue the moving the board around. 
                     stateCalibration=CAMERA_AND_PROJECTOR_PHASE1; 
    
                }
                break;
                
            case AR_DEMO:
                
                // We assume here that projector and camera are calibrated, as well as extrinsics
                // We can detect things using the pattern, or something else (say, a rectangular A4 page). The important thing is to get the 
                // transformation from OBJECT to CAMERA. We will use the EXTRINSICS to get the correspondance from OBJECT to PROJECTOR. 
                if (calibrationCamera.generateCandidateImageObjectPoints()) { 
                    cout << "Chessboard pattern recognized" << endl;
                    calibrationCamera.computeCandidateBoardPose();  // transformation from board to camera computed here
                }
                
                break;
                
                // TO DO: final "AR demo" mode with projector and camera calibrated (no need to recalibrate projector, nor the extrinsics).
                // We can use the chessboard, or just an A4 page using my modified CountourFinder. Project a movie?
                // ...
        }
		
	}
}

void testApp::draw() {
    stringstream intrinsicsProjector, intrinsicsCamera;
    
    ofSetWindowPosition(0,0); 
    
    // NOTE: we take the convention here of (0,0) at upper-left point (this is 
    // the reverse of OpenGL)
    ofViewport(viewportComputer);
    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0,viewportComputer.width, viewportComputer.height, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    
    int posTextY=CAM_HEIGHT+20, posTextX=10;
    
    // Draw current acquired image:
    ofSetColor(255);
    cam.draw(0,0, CAM_WIDTH, CAM_HEIGHT);
    
    // Draw preprocessed images for camera and projector board detection (we need to do the preprocessing BEFORE calling the detection functions):
    calibrationCamera.drawPreprocessedImage(CAM_WIDTH, 0, CAM_WIDTH/2, CAM_HEIGHT/2);
    calibrationProjector.drawPreprocessedImage(CAM_WIDTH, CAM_HEIGHT/2, CAM_WIDTH/2, CAM_HEIGHT/2);
    
    // Signal the acquisition of a new board:
    if (newBoardAquired) {
        drawHighlightString(" *** NEW BOARD ACQUIRED ***", CAM_WIDTH+20, CAM_HEIGHT-40, cyanPrint,  ofColor(255));
    }
    
    
    // Draw detected points (for CAMERA CALIBRATION BOARD):
    calibrationCamera.drawCandidateImagePoints(0,0, CAM_WIDTH, CAM_HEIGHT, ofColor(255,0,0));
    // Draw reprojected points (for CAMERA CALIBRATION BOARD):
    calibrationCamera.drawCandidateReprojection(0,0, CAM_WIDTH, CAM_HEIGHT, ofColor(255,255,0));
    // Draw axis of detected board:
    calibrationCamera.drawCandidateAxis(0,0, CAM_WIDTH, CAM_HEIGHT);
    
    // Draw undistorted image:
    //undistorted.draw(CAM_WIDTH, 0);
    
    intrinsicsCamera << "Camera fov: " << toOf(calibrationCamera.getDistortedIntrinsics().getFov());
    drawHighlightString(intrinsicsCamera.str(), posTextX, posTextY, yellowPrint, ofColor(0));
    drawHighlightString("Reproj error camera: " + ofToString(calibrationCamera.getReprojectionError()) + " from " + ofToString(calibrationCamera.size()), posTextX, posTextY+20, magentaPrint);
    
    intrinsicsProjector << "Projector fov: " << toOf(calibrationProjector.getDistortedIntrinsics().getFov()) ;//<< " distCoeffs: " << calibrationProjector.getDistCoeffs();
    drawHighlightString(intrinsicsProjector.str(), posTextX, posTextY+50, yellowPrint, ofColor(0));
    drawHighlightString("Reproj error projector: " + ofToString(calibrationProjector.getReprojectionError()) + " from " + ofToString(calibrationProjector.size()), posTextX, posTextY+70, magentaPrint);
    
    switch(stateCalibration) {
        case CAMERA_ONLY:
            drawHighlightString(" *** CALIBRATING CAMERA ***", COMPUTER_DISP_WIDTH-300, 40, cyanPrint,  ofColor(255));
            break;
        case CAMERA_AND_PROJECTOR_PHASE1:
        case CAMERA_AND_PROJECTOR_PHASE2:
            drawHighlightString(" *** CALIBRATING CAMERA + PROJECTOR ***", COMPUTER_DISP_WIDTH-400, 40, cyanPrint,  ofColor(255));
            
            if (dynamicProjection) 
                drawHighlightString("Using DYNAMIC Projection", COMPUTER_DISP_WIDTH-400, 60, cyanPrint,  ofColor(255));
            else 
                drawHighlightString("Using FIXED Projection", COMPUTER_DISP_WIDTH-400, 60, cyanPrint,  ofColor(255));
            
            
            // Extrinsics camera-projector:
            // NOTE: in the future, the object STEREO should have a flag "isReady" to test if it is possible to proceed with some things...
            // For the time being, I assume that if BOTH camera and projector have been calibrated once, then we also called the stereo calibration...
            if (calibrationCamera.isReady()&&calibrationProjector.isReady()&&(calibrationProjector.size()>0)) {
                
                extrinsics="";
                Mat rotCamToProj3x3;
                Rodrigues(rotCamToProj, rotCamToProj3x3);
                for (int i=0; i<3; i++) {
                    for (int j=0; j<3; j++)  extrinsics += ofToString(rotCamToProj3x3.at<double>(i,j),3) + "  "; 
                    extrinsics += ofToString(transCamToProj.at<double>(0,i),3)+" \n";
                }
                drawHighlightString(extrinsics, posTextX, posTextY+100, yellowPrint, ofColor(0));
                
                if (displayAR) {
                    // ============================ DRAW ON THE PROJECTOR DISPLAY (SET AS CONTIGUOUS SCREEN) ===================================
                    // NOTE: we can use the position parameter on the draw functions, or put (0,0) and do a translation here, or use two viewports...
                    //ofPushMatrix();
                    //ofTranslate(COMPUTER_DISP_WIDTH,0); // width of the computer screen (not the window spanning computer+projector display)
                    //... 
                    // ofPopMatrix();
                    
                    ofViewport(viewportProjector);
                    glMatrixMode(GL_PROJECTION);
                    glLoadIdentity();
                    gluOrtho2D(0,viewportProjector.width, viewportProjector.height, 0);
                    glMatrixMode(GL_MODELVIEW);
                    glLoadIdentity();
                    
                    vector<Point2f> testPoints;
                    // (a) Project over the printed chessboard using the board position directly in projector reference frame (this assumes projector calibration was possible):
                    testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationCamera.candidateObjectPoints,  calibrationProjector.candidateBoardRotation, calibrationProjector.candidateBoardTranslation); // no final Re and te means identity transform
                    calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(255,0,0), 3); // make static function
                    calibrationProjector.drawCandidateAxis(0,0, PROJ_WIDTH, PROJ_HEIGHT);
                    
                    // (b) Project over the projector points (not directly, but using object points and board transformation from the projector:
                    testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationProjector.candidateObjectPoints,  calibrationProjector.candidateBoardRotation, calibrationProjector.candidateBoardTranslation);// no final Re and te means identity transform
                    calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(0,255,0), 9);
                    
                    
                    // Now, the REAL tests - i.e. true AR using camera/projector calibration:
                    // (a) Project over the printed chessboard using EXTRINSICS (this assumes projector calibration as well as stereo calibration):            
                    testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationCamera.candidateObjectPoints,  calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation, rotCamToProj, transCamToProj);
                    calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(255, 100,0,100), 8);
                    
                    // (b) Project over the projector pattern, but using the points seen by the camera, and the EXTRINSICS:
                    testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationProjector.candidateObjectPoints,  calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation,rotCamToProj, transCamToProj); 
                    calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(0,255,0,100), 8);
                }
            }
            
            //... And ALWAYS project pattern points (I do it at last, to avoid having occlusion):
            // NOTE: we draw disks in IMAGE plane, meaning that they won't look like circles but ellipses in the real world... this is not a problem 
            // here, and may be preferable to use openGL rendering (which would render real circles in 3d world), because the camera is supposed to
            // be somehow near the projector: then the circles would look like circles whatever the plane inclination, which is better for running 
            // the "detect circles" openCV routine. 
            ofViewport(viewportProjector);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluOrtho2D(0,viewportProjector.width, viewportProjector.height, 0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            if (newBoardAquired==false) {
                // NOTE: the size of the dots depends on the distance if we use openCV circles... if we have an estimate of the extrinsics, it is 
                // better to do a good back-projection and use OpenGL to draw circles, then mantaining their real size.
                calibrationProjector.drawCandidateProjectorPattern(0,0, PROJ_WIDTH, PROJ_HEIGHT, ofColor(255,255,255,255), 6);//calibrationProjector.myPatternShape.squareSize/4);
            }
            else  { // just indicate that the board was acquired (short flash);
                calibrationProjector.drawCandidateProjectorPattern(0,0, PROJ_WIDTH, PROJ_HEIGHT, ofColor(255,100,0,255),calibrationProjector.myPatternShape.squareSize/2);
                newBoardAquired=false;
            }
            
            break;
            
        case AR_DEMO:
            // Two ways to do this: either use openCV to compute the projection or transformation of images, or faster, using OPENGL and properly setting the 
            // perspective and modelview matrix. 
            drawHighlightString(" *** AR DEMO MODE ***", COMPUTER_DISP_WIDTH-300, 40, cyanPrint,  ofColor(255));
            
            extrinsics="";
            Mat rotCamToProj3x3;
            Rodrigues(rotCamToProj, rotCamToProj3x3);
            for (int i=0; i<3; i++) {
                for (int j=0; j<3; j++)  extrinsics += ofToString(rotCamToProj3x3.at<double>(i,j),1) + "  "; 
                extrinsics += ofToString(transCamToProj.at<double>(0,i),4)+" \n";
            }
            drawHighlightString(extrinsics, posTextX, posTextY+100, yellowPrint, ofColor(0));
            
            if (calibrationCamera.candidatePatternDetected()) { 
                
                
                // First, get the 3d coordinates of the FOUR conrners in CHESSBOARD coordinates:
                vector<Point3f> corners; // define the four corners of the chessboard:
                corners.push_back(calibrationCamera.candidateObjectPoints[0]);
                corners.push_back(calibrationCamera.candidateObjectPoints[calibrationCamera.myPatternShape.getPatternSize().width-1]);
                corners.push_back(calibrationCamera.candidateObjectPoints[calibrationCamera.myPatternShape.getPatternSize().width*
                                                                          calibrationCamera.myPatternShape.getPatternSize().height-1]);
                corners.push_back(calibrationCamera.candidateObjectPoints[calibrationCamera.myPatternShape.getPatternSize().width*
                                                                          (calibrationCamera.myPatternShape.getPatternSize().height-1)]);
                
                
                //(a) ========================= Draw using OpenGL ========================= 
                // PROBLEM: For some unknown reason, the settings using openGL places the z=0 plane somehow below the real board plane. The 
                // difference is noticeable (projecting using OpenCV projection works very well, but using openGL the points/images are not
                // properly on the plane. One possible explanation is the problem with the task bar in OF_FULLSCREEN mode... 
                // (1) Set viewport:
                ofViewport(viewportProjector);
                // (2) Set perspective matrix (using the projector intrinsics):
                calibrationProjector.setOpenGLProjectionMatrix();
                // (3) Set the proper modelview:
                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();
                gluLookAt(0, 0, 0,   // position of camera
                          0, 0, 1,   // looking towards the point (0,0,1) 
                          0, -1, 0); // orientation
                
                Mat finalR, finalT;
                composeRT(calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation, // from model to camera
                          rotCamToProj, transCamToProj, // extrinsics (from camera to projector)
                          finalR, finalT); 
                applyMatrix(makeMatrix(finalR, finalT));
                ofScale(calibrationCamera.myPatternShape.squareSize, calibrationCamera.myPatternShape.squareSize, 0);
                
                //Draw something (image, whatever):
                ofSetColor(0,255,0); ofNoFill();
                ofSetLineWidth(2);
                ofRect(0,0,calibrationCamera.myPatternShape.getPatternSize().width-1, calibrationCamera.myPatternShape.getPatternSize().height-1);
            
#ifdef MOVIE_PLAY
                // Draw small images on the white squares of the chessboard:
                ofSetColor(255);
                for(int i = 0; i < calibrationCamera.myPatternShape.patternSize.height-1; i++)
                    for(int j = 0; j < calibrationCamera.myPatternShape.patternSize.width/2-1+i%2; j++) {
                        eyeMovie.draw(j*2+(i+1)%2,i,1,1); // note: ofScale ensures that each square is normalized
                    }
#endif
                
                //(b) ========================= Draw using OpenCV =========================
                // (just for checking compatibility). Note: if we draw circles, the circles would NOT be in perspective here!
                ofViewport(viewportProjector);
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                gluOrtho2D(0,viewportProjector.width, viewportProjector.height, 0);
                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();
                
                // draw viewport limits:
                ofSetLineWidth(5);
                ofSetColor(255, 255, 255);
                ofRect(0,0, viewportProjector.width, viewportProjector.height);
                
                // Project all corners of the printed chessboard using EXTRINSICS:
                vector<Point2f> testPoints;
                testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationCamera.candidateObjectPoints,  calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation, rotCamToProj, transCamToProj);
                calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(255, 0,0,255), 5);
                
                // Project the FOUR corners using the points seen by the camera, and the EXTRINSICS:
                testPoints=calibrationProjector.createImagePointsFrom3dPoints(corners,  calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation,rotCamToProj, transCamToProj); 
                calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(255,255,0,255), 8);
                
                // Draw axis (and indicate (0,0)):
                ofSetColor(255, 0, 255);
                ofLine(testPoints[0].x, testPoints[0].y, testPoints[1].x, testPoints[1].y);
                ofSetColor(0, 255, 255);
                ofLine(testPoints[0].x, testPoints[0].y, testPoints[3].x, testPoints[3].y);

            }
            break;
            
    }
    
}


// Save calibration parameters (along with the detected feature image points):
// NOTE: this should be a method of the STEREO CALIBRATION CLASS!
void testApp::saveExtrinsics(string filename, bool absolute) const {
    FileStorage fs(ofToDataPath(filename, absolute), FileStorage::WRITE);
    fs << "Rotation_Vector" << rotCamToProj;
    Mat rotCamToProj3x3;
    Rodrigues(rotCamToProj, rotCamToProj3x3);
    fs << "Rotation_Matrix" << rotCamToProj3x3;
    fs << "Translation_Vector" << transCamToProj;
    // Saved as a 4x4 openGL like modelviw matrix:
    ofMatrix4x4 mat4x4=makeMatrix(rotCamToProj, transCamToProj);
    fs << "OpenGL_Mat" << "[";
    for(int i = 0; i < 4; i++) { 
        fs << "{:" << "row" << "[:";
        for( int j = 0; j < 4; j++ ) fs << mat4x4(i,j);
        fs << "]" << "}";
    }
    fs << "]";
}


void testApp::loadExtrinsics(string filename, bool absolute) {
    FileStorage fs(ofToDataPath(filename, absolute), FileStorage::READ);
    fs["Rotation_Vector"] >> rotCamToProj;
    fs["Translation_Vector"] >> transCamToProj;
}


// =========== THINGS THAT WILL BELONG TO THE STEREO-CALIBRATION OBJECT ====================

void testApp::keyPressed(int key) {
    
    // Reset initialization:
    if(key == '1') {initialization(CAMERA_ONLY);}
    if(key == '2') {initialization(CAMERA_AND_PROJECTOR_PHASE1);}
    if(key == '3') {initialization(AR_DEMO);}
    
    if (key == 'm') manualAcquisition=!manualAcquisition;
	if (key == ' ') manualGetImage = !manualGetImage; 
    
    if (key=='p') dynamicProjection=!dynamicProjection; // toggle between fixed or dynamic (following) projection. 
    if (key=='o') dynamicProjectionInside=!dynamicProjectionInside;
    
    if (key=='d') displayAR=!displayAR; // this is just for test to see how it is going. But better not to use during 
    // calibration, because it interferes with the detection. 
}


