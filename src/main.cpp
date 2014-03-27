#define SPAN_MONITORS

#include "ofMain.h"
#include "testApp.h"
#ifdef SPAN_MONITORS 
    #include "ofAppGLFWWindow.h"
#endif

int main() {


    #ifdef SPAN_MONITORS
        ofAppGLFWWindow window;
        window.setMultiDisplayFullscreen(true);
        ofSetupOpenGL(&window,1024,768,OF_FULLSCREEN);
        ofRunApp(new testApp());

    #else
        ofSetupOpenGL(1024, 768, OF_WINDOW);
     
        ofRunApp(new testApp());
    #endif
}
