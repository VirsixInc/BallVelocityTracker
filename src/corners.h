#pragma once

#include <vector>
#include "ofMain.h"
#include "intercept.h"

struct Corners {
    ofPoint tl;
    ofPoint tr;
    ofPoint bl;
    ofPoint br;
};

void calculate_intercepts(const ofPoint &v, Intercept* intercept);
void get_corners(const std::vector<ofPoint> &v, Corners* corners);
