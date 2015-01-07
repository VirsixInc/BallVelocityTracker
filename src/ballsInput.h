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

const float SMOOTHING_SPEED = 0.25f;

class BallTrackerFollower : public ofxCv::RectFollower {
protected:
    ofColor color;
    cv::Rect prev;
public:
    BallTrackerFollower() {
        color = ofColor::black;
        velocity.x = velocity.y = 0.0f;
    }
    void setup(const cv::Rect& track) {
        prev = track;
    }
    void update(const cv::Rect& track) {
//        cv::Vec2f prevPos(prev.x + prev.width / 2.0f, prev.y + prev.height / 2.0f);
//        cv::Vec2f curPos(track.x + track.width / 2.0f, track.y + track.height / 2.0f);
//        ofVec2f newVel = ofxCv::toOf(curPos - prevPos);
        
//        ofLogNotice("prevpos: " + ofToString(ofxCv::toOf(prevPos)) + " curPos: " + ofToString(ofxCv::toOf(curPos)) + " newVel: " + ofToString(newVel));
        
//        velocity = (velocity + newVel)/2.0f;
        
//        prev = track;
    }
    void kill() { };
    void draw() { };
    
    void UpdateVelocity(ofVec2f newVel) {
        if(velocity.x == 0.0f && velocity.y == 0.0f) { // Gimicky and wrong, but edge-y enough that its ok. Supposed to be for 1st frame of follower
            velocity = newVel;
        } else {
            velocity.x = ofLerp(velocity.x, newVel.x, SMOOTHING_SPEED);
            velocity.y = ofLerp(velocity.y, newVel.y, SMOOTHING_SPEED);
        }
//        velocity.normalize(); // Prob shouldnt norm vel
    }
    
    ofVec2f velocity;
};

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
    ofxCv::RectTrackerFollower<BallTrackerFollower> tracker;
    
    int persistence;
    float maxDistance;
    
    ofxOscSender sender;
    ofxOscReceiver receiver;
    
    ofxKinect kinect;
    
    ofxCvGrayscaleImage grayImageDiff, grayImage, grayScale, grayWarp;
    ofxCvColorImage colImg, colScaleImg, warpedColImg;
    
//    ofxCvContourFinder contours;
    
    ofxSimpleGuiToo gui;
    
    bool saveBk, configured;
    
    int camWidth, camHeight, kinWidth, kinHeight;
    int range, depthThresh;
    int minContArea, maxContArea;
    int port;
    
    int timer;

    ofPoint src[4];
    ofPoint dest[4];
    
    int selectedCorner;
};
