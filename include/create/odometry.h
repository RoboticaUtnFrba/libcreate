#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cmath>
#include <math.h>
#include <iostream>

#include "create/encoder.h"
#include "create/pose_twist.h"

namespace create {
class Odometry {
private:
    void setPose(PoseTwist _newPose);
    void useICCMethod(double &_deltaX, double &_deltaY);
    void useStandardMethod(double &_deltaX, double &_deltaY);

    Encoder leftEncoder;
    Encoder rightEncoder;
    PoseTwist pose;
    double lastTime;
    double wheelSeparation;
    double ticksPerMeter;
    double leftTravel;
    double rightTravel;
    double deltaTravel;
    double deltaTheta;
    double wheelDistDiff;
    bool isStdCalculation;
public:
    Odometry(bool _useStdCalculation = true);
    Odometry(uint16_t _leftCount, uint16_t _rightCount, bool _useStdCalculation = true);
    void updatePose(double _newTime);
    void setTime(double _newTime);
    void setWheelSeparation(double _wheelSeparation);
    void setTicksPerMeter(double _ticks);
    void setEncoderRange(uint16_t _low, uint16_t _high);
    void updateLeftWheel(uint16_t _newCount);
    void updateRightWheel(uint16_t _newCount);
    PoseTwist getPose();
    double getLeftWheelDistance(){ return this->leftTravel; }
    double getRightWheelDistance(){ return this->rightTravel; }
    double getDeltaDistance(){ return this->deltaTravel; }
    double getDeltaOrientation(){ return this->deltaTheta; }
};
} // namespace create

#endif //ODOMETRY_H