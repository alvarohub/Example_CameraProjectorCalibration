Example_CameraProjectorCalibration
==================================

How to use:

Print the chessboard_8x5.pdf and glue it to a larger white board as in the video (https://www.youtube.com/watch?v=pCq7u2TvlxU). Note the orientation of the pattern: if you glue it the other way, the dynamically projected pattern will be outside the board). 

The actual size of the board is not important. If you want to get real distances (say, in cm or mm in the extrinsics), then you want to measure the size of the elemental squares and indicate it in the .yml file settingsPatternCamera.yml:

%YAML:1.0
xCount: 4
yCount: 5
squareSize: 30  << -- here, size in cm of the elemental square
posX: 400
posY: 250
patternType: 2
color: 255

Also, check some comments on the header of the testApp.h file. In particular, it is important that you #define the size of the computer screen and the projector screen:

#define COMPUTER_DISP_WIDTH 1440
#define COMPUTER_DISP_HEIGHT 900

// RESOLUTION OF CAMERA AND PROJECTOR: 
#define PROJ_WIDTH   800 
#define PROJ_HEIGHT 600

// Resolution of the camera (or at least resolution at which we want to calibrate it):
#define CAM_WIDTH 640
#define CAM_HEIGHT 480

The process goes in three phases:

(a) calibrating the camera (you may change the number of "good" pre-calibration boards by checking it on the testApp.cpp file, as well as the meaning of "good" by playing with the value of maxErrorCamera):

const int preCalibrateCameraTimes = 15; // this is for calibrating the camera BEFORE starting projector calibration. 
const int startCleaningCamera = 8; // start cleaning outliers after this many samples (10 is ok...). Should be < than preCalibrateCameraTimes
const float maxErrorCamera=0.3;

(b) The camera is then used to compute the 3d position of the projected pattern (asymmetric circles) by backprojection, as long as both patterns are visible at the same time (acquisition is automatic) and is indicated as a red flash on the projected pattern. In this phase, stereo calibration is performed to obtain the camera-projector extrinsics. The process of projector calibration goes exactly like that for the camera, meaning that you need to set the maxErrorProjector, as well as the minimum number of "good" boards).  There is an additional parameter "startDynamicProjectorPattern": this is the minimum of good boards before starting projecting "dynamically". When this number of boards is reached, the projection will follow the printed pattern and you can start moving the board around so as to better explore the whole field of view space (and get better and more accurate calibration): 

const float maxErrorProjector=0.30;
const int startCleaningProjector = 6;
const int startDynamicProjectorPattern=5; <<-- after this number of projector/camera calibration, the projection will start following the printed pattern to facilitate larger exploration of the field of view. If this number is larger than minNumGoodBoards, then this will never happen automatically. 

const int minNumGoodBoards=15; <<-- after this number of simultaneoulsy acquired "good" boards, IF the projector total reprojection error is smaller than a certain threshold, we end calibration (and move to AR mode automatically)

(c) Once minNumGoodBoards are acquired for both the camera and projector, extrinsics and intrinsics are saved, and the program goes into a simple AR demo mode, projecting some dots over the printed pattern. Note that this may affect the printed board detection, but it is just for trying (normally you would use another kind of fiducial, for instance a marker or the corners of a board, and project on the side or inside the board, not OVER the printed fiducials...)


----------------------------------------------------------------------------------------------------------------
Things to do:

- solve inconsistence between openGL and openCV "manual" projection. Something to do with the full screen mode in dual screen?

- pre-processing of acquired images with color based segmentation (this data should be in the pattern calibration file)
- color picker 
- 3d representation of camera/projector configuration in a separate viewport (with axis and mouse controlled rotation to check some things, like camera orientation, etc)
- openCV 2.4 would avoid slowing down image acquisition when the board is not on the image. 


- DEMOS:
 - use the modified rectangle finder (with tracking) to do AR. 