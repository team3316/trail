#include <cmath>

#include <nlohmann/json.hpp>

#include "utils.h"
#include "Robot.h"

using json = nlohmann::json;

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

double trail::Robot::getBaseWidth() {
    return this->mBaseWidth;
}

double trail::Robot::getFreeSpeed() {
    return this->mFreeSpeed;
}

double trail::Robot::timeToMaxVelocity(double i) {
    double af = this->maxAcceleration(),
           vf = this->mFreeSpeed;
    return vf / af;
}

double trail::Robot::maxAcceleration() {
    double m = this->mMass,
           ts = this->mStallTorque,
           g = this->mGearRatio,
           r = this->mWheelRadius,
           n = this->mNumOfDriveMotors;
    return (n * ts * g) / (r * m * 2.5);
}

trail::Robot trail::Robot::fromJSON(const std::string &filename) {
    std::string file;
    if (readFile(filename, &file) < 0) {
        LOGE("Couln't read " << filename << "; Aborting.");
        exit(-1);
    }

    auto json = json::parse(file);

    std::vector<std::string> requiredKeys = {
        "mass",
        "base-width",
        "free-speed",
        "stall-torque",
        "gear-ratio",
        "wheel-radius",
        "motors"
    };
    for (auto key: requiredKeys) {
        if (json.find(key) == json.end()) {
            LOGE(key << " is missing from " << filename << ". Please add it and re-run trail. Aborting.");
            exit(-1);
        }
    }

    return {
        json["mass"],
        json["base-width"],
        json["free-speed"],
        json["stall-torque"],
        json["gear-ratio"],
        json["wheel-radius"],
        json["motors"]
    };
}
