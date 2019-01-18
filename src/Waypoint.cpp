#include <math.h>

#include "utils.h"

#include "Waypoint.h"

Waypoint::Waypoint(double x, double y, double theta) {
    this->mX = x;
    this->mY = y;
    this->mTheta = radians(90 - theta);
}

double Waypoint::getX() {
    return this->mX;
}

double Waypoint::getY() {
    return this->mY;
}

Eigen::Vector2d Waypoint::firstDerivative(double scale) {
    Eigen::Vector2d vec;
    vec << cos(this->mTheta), sin(this->mTheta);
    return scale * vec;
}

Eigen::Vector2d Waypoint::secondDerivative(double scale) {
    Eigen::Vector2d vec;
    vec << -sin(this->mTheta), cos(this->mTheta);
    return scale * vec;
}

double Waypoint::distanceToPoint(Waypoint otherPoint) {
    double dx = otherPoint.getX() - this->mX;
    double dy = otherPoint.getY() - this->mY;
    return hypot(dx, dy);
}
