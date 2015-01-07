#include "ballsInput.h"

//--------------------------------------------------------------
void testApp::setup() {
    sender.setup("localhost", 9999);
    receiver.setup(7600);
    
    kinect.setRegistration(true); // Syncs up depth and RGB
    kinect.init();
    kinect.open();                // Opens first available kinect
    
    camWidth = 320;
    camHeight = 240;
    kinWidth = kinect.getWidth();
    kinHeight = kinect.getHeight();
    
    colImg.allocate(camWidth, camHeight);
    colScaleImg.allocate(camWidth, camHeight);
    warpedColImg.allocate(camWidth, camHeight);
    
    grayImage.allocate(camWidth, camHeight);
    grayScale.allocate(camWidth, camHeight);
    grayWarp.allocate(camWidth, camHeight);
    grayImageDiff.allocate(camWidth, camHeight);
    
    ofSetFrameRate(60);
    
    src[0] = ofPoint(0,0);
    src[1] = ofPoint(camWidth,0);
    src[2] = ofPoint(camWidth,camHeight);
    src[3] = ofPoint(0,camHeight);
    
    dest[0] = ofPoint(0,0);
    dest[1] = ofPoint(camWidth,0);
    dest[2] = ofPoint(camWidth,camHeight);
    dest[3] = ofPoint(0,camHeight);
    
    state = ConfigBackground;
    
    timer = 0;
    
    gui.addSlider("Depth Thresh", depthThresh, 0, 255);
    gui.addSlider("Range", range, 0, 80);
    gui.addSlider("minContArea", minContArea, 0, 1000);
    gui.addSlider("maxContArea", maxContArea, 0, 2000);
    gui.addToggle("Configured", configured);
    gui.addToggle("Save Background", saveBk);
    gui.addSlider("Persistence", persistence, 0, 100);
    gui.addSlider("maxDist", maxDistance, 0.1, 200.0);
    gui.loadFromXML();
}

//--------------------------------------------------------------
void testApp::update() {
    CheckOSCMessage();
    
    kinect.update();
    if(kinect.isFrameNew()) {
        
        UpdateImages();
        if (saveBk)
            SaveBackground();
        
        switch(state) {
            case ConfigScreen:
                ConfigureScreen();
                break;
                
            case ConfigBackground:
                SaveBackground();
                SendMessage("/config/done");
                state = Main;
                
                ThresholdImages();
                break;

                
            case Main:
                contourFinder.getTracker().setPersistence(persistence);
                contourFinder.getTracker().setMaximumDistance(maxDistance);
                contourFinder.setFindHoles(true);
                contourFinder.setMinArea(minContArea);
                contourFinder.setMaxArea(maxContArea);
                contourFinder.setUseTargetColor(false);

                if(configured) {
                    ThresholdImages();
                    
                    contourFinder.findContours(grayImage);
                    tracker.track(contourFinder.getBoundingRects());
                    if(contourFinder.size() > 0) {
                        for(int i = 0; i < contourFinder.size(); i++) {
                            int label = contourFinder.getLabel(i);
                            int age = contourFinder.getTracker().getAge(label);
                            const cv::Rect& current = contourFinder.getTracker().getSmoothed(label);
                            cv::Vec2f curPos(current.x + current.width / 2, current.y + current.height / 2);
                            
                            tracker.getFollowers()[i].UpdateVelocity(ofxCv::toOf(contourFinder.getTracker().getVelocity(i))); // Labels and followers *should* match up
                            
                            string notice = "curLabels: " + ofToString(contourFinder.getTracker().getNewLabels().size())
                                + " |newLabels: " + ofToString(contourFinder.getTracker().getNewLabels().size())
                                + " |prevLabels: " + ofToString(contourFinder.getTracker().getPreviousLabels().size())
                                + " |deadLabels: " + ofToString(contourFinder.getTracker().getDeadLabels().size())
                                + " |label: " + ofToString(label)
                                + " |age: " + ofToString(age)
                                + " |pos: " + ofToString(ofxCv::toOf(curPos))
                                + " |vel: " + ofToString(tracker.getFollowers()[i].velocity);
//                                + " |vel: " + ofToString(ofxCv::toOf(contourFinder.getTracker().getVelocity(i)));

                            ofLogNotice(ofToString(notice));
                        }
                    }
                }
                break;
        }
        
    }
    
}

//--------------------------------------------------------------
void testApp::UpdateImages() {
    colImg.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
    colScaleImg.scaleIntoMe(colImg);
    warpedColImg.warpIntoMe(colScaleImg, dest, src);
    
    grayScale.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    grayWarp.scaleIntoMe(grayScale);
    grayImage.warpIntoMe(grayWarp, dest, src);
}

//--------------------------------------------------------------
void testApp::ThresholdImages() {
    grayImage.absDiff(grayImageDiff);
    unsigned char* grayPixels = grayImage.getPixels();
    
    for (int i = 0, n = grayImage.getWidth() * grayImage.getHeight(); i < n; i++) {
        if(ofInRange(grayPixels[i],depthThresh,depthThresh+range)){
            grayPixels[i] = 255;
        }else{
            grayPixels[i] = 0;
        }
    }
    grayImage = grayPixels;
    grayImage.blur(1);
}

//--------------------------------------------------------------
void testApp::ConfigureScreen() {
    timer++;
    contourFinder.setTargetColor(ofColor::white);
    contourFinder.setUseTargetColor(true);
    contourFinder.setMinArea(100); // TODO tweak. Seems good tho.
    contourFinder.setThreshold(100); // TODO tweak. Seems good tho.
    contourFinder.resetMaxArea();
    contourFinder.getTracker().setPersistence(0);
    contourFinder.findContours(warpedColImg);
    
    //TODO cant just give up like this.
    if(contourFinder.size() < 1) {
        ofLogNotice("Less than 1 contour(s) found");
        return;
    }
    if(contourFinder.size() > 1) {
        ofLogNotice("More than 1 contour(s) found");
        return;
    }
    
    if(timer > 20) { //Delay to let front end change scenes
        const std::vector<ofPoint> contPts = contourFinder.getPolyline(0).getVertices();
        get_corners(contPts, &contCorners);
        
        dest[0] = contCorners.tl;//ofPoint(rect.x, rect.y);
        dest[1] = contCorners.tr;//ofPoint(rect.x + rect.width, rect.y);
        dest[2] = contCorners.br;//ofPoint(rect.x + rect.width, rect.y + rect.height);
        dest[3] = contCorners.bl;//ofPoint(rect.x, rect.y + rect.height);
        
        ofxXmlSettings settings;
        settings.addTag("positions");
        settings.pushTag("positions");
        for(int i = 0; i < 4; i++){
            if(dest[i].x != -1 && dest[i].y != -1){
                settings.addTag("position");
                settings.pushTag("position", i);
                settings.setValue("X", dest[i].x);
                settings.setValue("Y", dest[i].y);
                settings.popTag();
            }
        }
        settings.popTag();
        settings.saveFile("points.xml");
        
        SendMessage("/config/cornerParsed");
        state = ConfigBackground;
    }
}

//--------------------------------------------------------------
void testApp::SaveBackground() {
    saveBk = false;
    unsigned char* pixels = grayImage.getPixels();
    grayImageDiff.setFromPixels(pixels, camWidth, camHeight);
}

//--------------------------------------------------------------
void testApp::draw() {
    ofSetHexColor(0xff0000);
    
    ofSetColor(255, 255, 255);
    grayImage.draw(0, 0, camWidth, camHeight);
    warpedColImg.draw(camWidth*2, 0, camWidth, camHeight);
    grayImageDiff.draw(camWidth*2, camHeight, camWidth, camHeight);
    
    if(state == Main) {
        ofSetColor(ofColor::teal);
        for(int i = 0; i < contourFinder.size(); i++) {
            int label = contourFinder.getLabel(i);
//            cv::Vec2f vel = contourFinder.getTracker().getVelocity(i);
            if(tracker.getFollowers().size() > 0) {
                cv::Vec2f vel = ofxCv::toCv(tracker.getFollowers()[i].velocity);
                const cv::Rect& current = contourFinder.getTracker().getCurrent(label);
                cv::Vec2f pos(current.x + current.width / 2, current.y + current.height / 2);
                ofLine(ofxCv::toOf(pos), ofxCv::toOf(pos + (vel * 10)));
            }
        }
    }
    
    if(configured){
        contourFinder.draw();
    } else {
        ofxXmlSettings settings;
        for(int j=0; j<4;j++){
            ofSetColor(255,255,255);
            settings.loadFile("points.xml");
            settings.pushTag("positions");
            settings.pushTag("position", j);
            if(selectedCorner < 0){
                if(settings.getValue("X", -1) != -1){
                    dest[j].x = settings.getValue("X", -1);
                }
                if(settings.getValue("Y", -1) != -1){
                    dest[j].y = settings.getValue("Y", -1);
                }
            }
            
            switch(j){
                case 0:
                    ofSetColor(0,0,255);
                    break;
                case 1:
                    ofSetColor(255,0,0);
                    break;
                case 2:
                    ofSetColor(255,255,0);
                    break;
                case 3:
                    ofSetColor(0,255,0);
                    break;
            }
            ofCircle(dest[j].x, dest[j].y, 3);
            settings.popTag();
            settings.popTag();
        }
        
    }
    gui.draw();
}

//--------------------------------------------------------------
void testApp::CheckOSCMessage() {
    while(receiver.hasWaitingMessages()){
        // get the next message
        ofxOscMessage m;
        if(receiver.getNextMessage(&m)) {
            string addr = m.getAddress();
            if(addr == "/config/corner") {
                state = ConfigScreen;
                timer = 0;
                dest[0] = ofPoint(0,0);
                dest[1] = ofPoint(camWidth,0);
                dest[2] = ofPoint(camWidth,camHeight);
                dest[3] = ofPoint(0,camHeight);
            } else if(addr == "/startGame") {
                state = Main;
            }
        }
    }
}

//--------------------------------------------------------------
void testApp::SendMessage(string message) {
    ofxOscMessage m;
    m.setAddress(message);
    sender.sendMessage(m);
}

//--------------------------------------------------------------
void testApp::exit() {
    kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
    switch(key){
        case OF_KEY_LEFT:
            gui.toggleDraw();
    }
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
//    
//    selectedCorner = -1;
//    if(selectedCorner == -1 && idSet){
//        idSet = false;
//        colorJustAcquired = warpedColImg.getPixelsRef().getColor(x, y);
//        ofLogNotice(ofToString(x) + "  " + ofToString(y));
//        players[id].ballColor = colorJustAcquired;
//        colorContourFinder.setTargetColor(players[id].ballColor,ofxCv::TRACK_COLOR_HSV);
//    }
}
