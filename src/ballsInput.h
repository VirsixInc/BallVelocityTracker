#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include "ofxXmlSettings.h"
#include "ofxMSAInteractiveObject.h"
#include "ofxSimpleGuiToo.h"
#include "corners.h"
#include "ofMath.h"
#include "BallTracker.h"

class testApp : public ofBaseApp {
    
public:
    enum State {
        ConfigBackground, Main, ConfigScreen
    };
    void setup();
    void update();
    void draw();
    void exit();
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    //	void mouseReleased(int x, int y, int button);
    //	void windowResized(int w, int h);
    
private:
    void UpdateImages();
    void ConfigureScreen();
    void ThresholdImages();
    
    void resetPoints();
    void savePoints();
    
    void SaveBackground();
    
    void CheckOSCMessage();
    
    void SendMessage(string message);
    
    Corners contCorners;
    
    State state;
    float threshold;
    
    ofxCv::ContourFinder contourFinder;
    
    ofxOscSender sender;
    ofxOscReceiver receiver;
    
    ofxKinect kinect;
    
    ofxCvGrayscaleImage grayImageDiff, grayImage, grayScale, grayWarp;
    ofxCvColorImage colImg, colScaleImg, warpedColImg;
    
    BallTracker ballTracker;
    
    ofxSimpleGuiToo gui;
    
    vector<ofRectangle> rects;
    
    int persistence;
    float maxDistance;
    int minContArea, maxContArea;
    float velSmoothRate;
    
    bool saveBk, configured;
    
    int camWidth, camHeight, kinWidth, kinHeight;
    int range, depthThresh;
    int port;
    
    int timer;

    ofPoint src[4];
    ofPoint dest[4];
    
    int selectedCorner;
};
