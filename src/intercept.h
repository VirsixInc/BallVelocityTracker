#ifndef _INTERCEPT_H
#define _INTERCEPT_h 1

class Intercept {
    public:
        Intercept();

        bool has_neg;
        bool has_pos;

        double getPos() const;
        double getNeg() const;

        void setPos(double p);
        void setNeg(double n);

    private:
        double pos;
        double neg;
};


#endif