#include <cmath>

#include "RobotOrigins.h"

#include "utils.h"

trail::RobotOrigins::RobotOrigins(trail::Waypoint startWaypoint, double scale, double radius) {
  double theta = radians(90 + startWaypoint.getHeading());
  this->mNormal = Eigen::Vector2d(-sin(theta), cos(theta));
  this->mDNormal = Eigen::Vector2d(-cos(theta), sin(theta));
  this->mWaypoint = startWaypoint;
  this->mRadius = radius;
  this->mVelScale = scale;
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d> trail::RobotOrigins::getPositionOrigins() {
  Eigen::Vector2d pos = this->mWaypoint.toPoint();
  return std::make_tuple(pos + this->mRadius * this->mNormal, pos - this->mRadius * this->mNormal);
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d> trail::RobotOrigins::getVelocityOrigins() {
  Eigen::Vector2d vel = this->mWaypoint.firstDerivative(this->mVelScale);
  return std::make_tuple(vel + this->mRadius * this->mDNormal, vel - this->mRadius * this->mDNormal);
}
