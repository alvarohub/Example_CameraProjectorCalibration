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

----------------------------------------------------------------------------------------------------------------
Things to do:

- solve inconsistence between openGL and openCV "manual" projection. Something to do with the full screen mode in dual screen?

- pre-processing of acquired images with color based segmentation (this data should be in the pattern calibration file)
- color picker 
- 3d representation of camera/projector configuration in a separate viewport (with axis and mouse controlled rotation to check some things, like camera orientation, etc)
- openCV 2.4 would avoid slowing down image acquisition when the board is not on the image. 


- DEMOS:
 - use the modified rectangle finder (with tracking) to do AR. 