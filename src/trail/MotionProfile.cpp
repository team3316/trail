#include <iostream>
#include <cmath>

#include "utils.h"

#include "MotionProfile.h"

trail::MotionProfile::MotionProfile(double maxVelocity, double maxAcceleration, double maxJerk, double cruiseVelocity) {
    this->mMaxVelocity = maxVelocity;
    this->mMaxAcceleration = maxAcceleration;
    this->mMaxJerk = maxJerk;
    this->mCruiseVelocity = cruiseVelocity;

    double minDistance = 2 * this->distanceToVelocity(0, cruiseVelocity); // Both acceleration and deceleration
    this->mMinDistance = minDistance;
}

trail::MotionProfile::MotionProfile() {
    this->mMaxVelocity = 0;
    this->mMaxAcceleration = 0;
    this->mMaxJerk = 0;
    this->mCruiseVelocity = 0;
    this->mMinDistance = 0;
}

void trail::MotionProfile::setDistance(double distance) {
    this->mFullDistance = distance < this->mMinDistance ? this->mMinDistance : distance;
}

double trail::MotionProfile::distanceToVelocity(double startVel, double endVel) {
    double accSigned = this->mMaxAcceleration * sgn(endVel - startVel);
    double dvsq = pow(endVel, 2) - pow(startVel, 2);
    return dvsq / (2 * accSigned);
}

Eigen::Vector3d trail::MotionProfile::calculate(double t) {
    double timeToCruise = this->mCruiseVelocity / this->mMaxAcceleration; // This is also true for deceleration

    if (this->mMinDistance == this->mFullDistance) { // Shouldn't use the other implementation
        if (0 <= t && t <= timeToCruise) {
            return Eigen::Vector3d(
                this->mMaxAcceleration * t * t / 2, // Position
                this->mMaxAcceleration * t, // Velocity
                this->mMaxAcceleration // Acceleration
            );
        }

        if (timeToCruise <= t && t <= 2 * timeToCruise) {
            return Eigen::Vector3d(
                -this->mMaxAcceleration * t * t / 2, // Position
                -this->mMaxAcceleration * t, // Velocity
                -this->mMaxAcceleration // Acceleration
            );
        }
    } else {
        double distCruise = this->mFullDistance - this->mMinDistance; // The amount of distance we're going to do in cruise
        double timeInCruise = (distCruise / this->mCruiseVelocity); // Acceleration in cruising velocity is zero
        double timeTotal = (2 * timeToCruise) + timeInCruise;

        if (0 <= t && t < timeToCruise) { // Acceleration period
            return Eigen::Vector3d(
                this->mMaxAcceleration * t * t / 2, // Position
                this->mMaxAcceleration * t, // Velocity
                this->mMaxAcceleration // Acceleration
            );
        }


        if (timeToCruise <= t && t < timeToCruise + timeInCruise) { // Cruising period
            double matcher = (this->mMaxAcceleration * timeToCruise * timeToCruise / 2) - this->mCruiseVelocity * timeToCruise;
            return Eigen::Vector3d(
                this->mCruiseVelocity * t + matcher, // Position
                this->mCruiseVelocity, // Velocity
                0 // Acceleration
            );
        }

        if (timeToCruise + timeInCruise <= t && t <= timeTotal) { // Deceleration period
            double dt = t - timeTotal;
            double matcher = this->mCruiseVelocity * (timeTotal - timeToCruise);
            return Eigen::Vector3d(
                -this->mMaxAcceleration * dt * dt / 2 + matcher, // Position
                -this->mMaxAcceleration * dt, // Velocity
                -this->mMaxAcceleration // Acceleration
            );
        }
    }
}

double trail::MotionProfile::getTotalTime() {
    double timeToCruise = this->mCruiseVelocity / this->mMaxAcceleration; // This is also true for deceleration
    double distCruise = this->mFullDistance - this->mMinDistance; // The amount of distance we're going to do in cruise
    double timeInCruise = (distCruise / this->mCruiseVelocity); // Acceleration in cruising velocity is zero

    return 2 * timeToCruise + timeInCruise;
}

