#include "testApp.h"
#include "ofAppGlutWindow.h"



int main() {
	ofAppGlutWindow window;
	//ofSetupOpenGL(&window, 1440+PROJ_WIDTH, 990, OF_WINDOW); // width computer screen + width projector screen / height projector
    ofSetupOpenGL(&window, 1440+PROJ_WIDTH, 990, OF_FULLSCREEN); // width computer screen + width projector screen / height projector
	ofRunApp(new testApp());
}
