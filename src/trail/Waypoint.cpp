#include <cmath>

#include "utils.h"

#include "Waypoint.h"

trail::Waypoint::Waypoint(double x, double y, double theta) {
    this->mX = x;
    this->mY = y;
    this->mHeading = theta;
    this->mTheta = radians(90.0 - theta);
}

double trail::Waypoint::getX() const {
    return this->mX;
}

double trail::Waypoint::getY() const {
    return this->mY;
}

double trail::Waypoint::getHeading() const {
    return this->mHeading;
}

Eigen::Vector2d trail::Waypoint::firstDerivative(double scale) {
    Eigen::Vector2d vec;
    vec << cos(this->mTheta), sin(this->mTheta);
    return scale * vec;
}

Eigen::Vector2d trail::Waypoint::secondDerivative(double scale) {
    Eigen::Vector2d vec;
    vec << -sin(this->mTheta), cos(this->mTheta);
    return scale * vec;
}

double trail::Waypoint::distanceToPoint(trail::Waypoint otherPoint) {
    double dx = otherPoint.getX() - this->mX;
    double dy = otherPoint.getY() - this->mY;
    return hypot(dx, dy);
}

void trail::from_json(const json &json, trail::Waypoint &wp) {
   double x = json.at("point")[0],
          y = json.at("point")[1],
          heading = json.at("angle");
   wp = Waypoint(x, y, heading);
}

void trail::to_json(json &json, const trail::Waypoint &wp) {
    json = {
        {"point", {wp.getX(), wp.getY()}},
        {"angle", wp.getHeading()}
    };
}
