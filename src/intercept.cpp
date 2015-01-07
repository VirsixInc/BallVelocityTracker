#include "intercept.h"


Intercept::Intercept() : has_pos(false), has_neg(false) {}

double Intercept::getPos() const {
    return pos;
}

double Intercept::getNeg() const {
    return neg;
}

void Intercept::setPos(double p) {
    pos = p;
    has_pos = true;
}

void Intercept::setNeg(double n) {
    neg = n;
    has_neg = true;
}