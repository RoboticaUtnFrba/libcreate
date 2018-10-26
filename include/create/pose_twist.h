#ifndef POSE_H
#define POSE_H

class PoseTwist {
private:
public:
    PoseTwist()
    : x(0.0), y(0.0), theta(0.0)
    , xVel(0.0), yVel(0.0), thetaVel(0.0){}
    double x;
    double y;
    double theta;
    double xVel;
    double yVel;
    double thetaVel;
    
};

#endif //POSE_H