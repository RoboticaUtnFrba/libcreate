#include "create/odometry.h"

/**
 * Keeps track of the current position
 * and velocity of a robot ustd::sing
 * differential drive.
 */

namespace create {

Odometry::Odometry(bool _useStdCalculation) {
    Odometry(0, 0, _useStdCalculation);
}

Odometry::Odometry(uint16_t _leftCount, uint16_t _rightCount, bool _useStdCalculation) {
    this->leftEncoder = Encoder(_leftCount);
    this->rightEncoder = Encoder(_rightCount);
    this->pose = PoseTwist();
    this->lastTime = 0.0;
    this->isStdCalculation = _useStdCalculation;
}

void Odometry::setWheelSeparation(double _wheelSeparation) {
    this->wheelSeparation = _wheelSeparation;
}

void Odometry::setTicksPerMeter(double _ticks) {
    this->ticksPerMeter = _ticks;
}

void Odometry::setEncoderRange(uint16_t _low, uint16_t _high) {
    this->leftEncoder.setRange(_low, _high);
    this->rightEncoder.setRange(_low, _high);
}

void Odometry::setTime(double _newTime) {
    this->lastTime = _newTime;
}

void Odometry::updateLeftWheel(uint16_t _newCount) {
    this->leftEncoder.update(_newCount);
}

void Odometry::updateRightWheel(uint16_t _newCount) {
    this->rightEncoder.update(_newCount);
}

void Odometry::setPose(PoseTwist _newPose) {
    this->pose = _newPose;
}

PoseTwist Odometry::getPose() {
    return this->pose;
}

void Odometry::updatePose(double _newTime) {
    /**
     * Updates the pose based on the accumulated encoder ticks
     * of the two wheels. See https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
     * for details.
     */
    this->leftTravel = this->leftEncoder.getDelta() / this->ticksPerMeter;
    this->rightTravel = this->rightEncoder.getDelta() / this->ticksPerMeter;
    const double _deltaTime = _newTime - this->lastTime;

    this->wheelDistDiff = this->rightTravel - this->leftTravel;

    this->deltaTravel = (this->rightTravel + this->leftTravel) / 2.0;
    this->deltaTheta = this->wheelDistDiff / this->wheelSeparation;

    double _deltaX;
    double _deltaY;
    if (fabs(this->wheelDistDiff) < 1E-5) {
        _deltaX = this->deltaTravel * std::cos(this->pose.theta);
        _deltaY = this->deltaTravel * std::sin(this->pose.theta);
    }
    else {
        this->isStdCalculation ? useStandardMethod(_deltaX, _deltaY) 
                               : useICCMethod(_deltaX, _deltaY);
    }
    this->pose.x += _deltaX;
    this->pose.y += _deltaY;
    this->pose.theta = std::fmod(this->pose.theta + this->deltaTheta, 2*M_PI);

    this->pose.yVel = 0.0;

    if ((fabs(this->deltaTheta) > 1E-5) || (_deltaTime <= 0.0)) {
        this->pose.xVel = this->deltaTravel / _deltaTime;
        this->pose.thetaVel = this->deltaTheta / _deltaTime;
    }
    else {
        this->pose.xVel = 0.0;
        this->pose.thetaVel = 0.0;
    }

    this->lastTime = _newTime;
}

void Odometry::useICCMethod(double &_deltaX, double &_deltaY) {
    const double _radius = this->deltaTravel / this->deltaTheta;

    // Find the instantaneous center of curvature (ICC)
    const double _iccX = this->pose.x - _radius * std::sin(this->pose.theta);
    const double _iccY = this->pose.y + _radius * std::cos(this->pose.theta);

    _deltaX = std::cos(this->deltaTheta) * (this->pose.x - _iccX)
            - std::sin(this->deltaTheta) * (this->pose.y - _iccY)
            + _iccX - this->pose.x;

    _deltaY = std::sin(this->deltaTheta) * (this->pose.x - _iccX)
            + std::cos(this->deltaTheta) * (this->pose.y - _iccY)
            + _iccY - this->pose.y;
}

void Odometry::useStandardMethod(double &_deltaX, double &_deltaY) {
    const double  _turnRadius = (this->wheelSeparation / 2.0) * (this->leftTravel + this->rightTravel) / this->wheelDistDiff;
    _deltaX = _turnRadius * (std::sin(this->pose.theta + this->deltaTheta) - std::sin(this->pose.theta));
    _deltaY = -_turnRadius * (std::cos(this->pose.theta + this->deltaTheta) - std::cos(this->pose.theta));
}

} // namespace create