#include "ofMain.h"
#include "ofApp.h"

#include "System.h"


//========================================================================
int main(int argc,char **argv){
  ofSetupOpenGL(640,480 ,OF_WINDOW);			// <-------- setup the GL context

  //slam2
	
  //cv::Mat im(pFrame);
  //SLAM.TrackMonocular(im,tframe);
  //SLAM.Shutdown();

  // this kicks off the running of my app
  // can be OF_WINDOW or OF_FULLSCREEN
  // pass in width and height too:
  ofApp *pofApp = new ofApp();
  if(argc > 1){
    if(strcmp("debug",argv[0])) {
      pofApp->enableDebug();
      cout <<"enable debug mode." <<endl;
    }
  }
  //pofApp->setSlam(&SLAM);
  //pofApp->m_pSlam = &SLAM;
  ofRunApp(pofApp);
  cout <<"main over." <<endl;
}
