#include <cmath>

#include "utils.h"
#include "Robot.h"

trail::Robot::Robot(double mass, double baseWidth, double freeSpeed, double stallTorque, double gearRatio, double wheelRadius,
             int numOfDriveMotors) {
    this->mMass = mass;
    this->mBaseWidth = baseWidth;
    this->mFreeSpeed = freeSpeed;
    this->mStallTorque = stallTorque;
    this->mGearRatio = gearRatio;
    this->mWheelRadius = wheelRadius;
    this->mNumOfDriveMotors = numOfDriveMotors;
}

trail::Robot::Robot() {
    this->mMass = -1;
    this->mBaseWidth = -1;
    this->mFreeSpeed = -1;
    this->mStallTorque = -1;
    this->mGearRatio = -1;
    this->mWheelRadius = -1;
    this->mNumOfDriveMotors = -1;
}

double trail::Robot::timeToMaxVelocity(double i) {
    double m = this->mMass,
           vf = this->mFreeSpeed,
           ts = this->mStallTorque,
           g = this->mGearRatio,
           r = this->mWheelRadius,
           n = this->mNumOfDriveMotors;
    double k1 = (2 * n * ts * g) / (r * m * vf);
    return i / k1;
}

double trail::Robot::maxAcceleration() {
    double m = this->mMass,
           ts = this->mStallTorque,
           g = this->mGearRatio,
           r = this->mWheelRadius,
           n = this->mNumOfDriveMotors;
    return (n * ts * g) / (r * m);
}

double trail::Robot::distanceToMaxVelocity(double startVel, double endVel) {
    double accSigned = this->maxAcceleration() * sgn(endVel - startVel);
    double dvsq = pow(endVel, 2) - pow(startVel, 2);
    return dvsq / (2 * accSigned);
}
