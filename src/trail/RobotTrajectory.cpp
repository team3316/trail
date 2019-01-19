//
// Created by Jonathan Ohayon on 2019-01-18.
//

#include <cmath>

#include "utils.h"

#include "RobotTrajectory.h"

trail::RobotTrajectory::RobotTrajectory(trail::Waypoints waypoints, trail::Robot robot) {
    this->mWaypoints = waypoints;
    this->mRobot = robot;
    this->mNumOfSegments = (int) waypoints.size() - 1;
}

trail::RobotTrajectory::RobotTrajectory() {
    this->mWaypoints = {};
}

std::vector<trail::Spline> trail::RobotTrajectory::generateSplines() {
    std::vector<trail::Spline> splines;

    for (int i = 0; i < this->mNumOfSegments; ++i) {
        trail::Waypoint current = this->mWaypoints[i],
                        next = this->mWaypoints[i + 1];
        double dist = current.distanceToPoint(next);

        PointVector points = {
            {current.getX(), current.getY()},
            current.firstDerivative(1.5 * dist),
            current.secondDerivative(),
            {next.getX(), next.getY()},
            next.firstDerivative(1.5 * dist),
            next.secondDerivative()
        };

        splines.emplace_back(Spline(points));
    }

    return splines;
}

trail::Vector12d *trail::RobotTrajectory::calculateTrajectory() {
    auto curve = (trail::Vector12d *) std::malloc(sizeof(trail::Vector12d) * this->mNumOfSegments * SPLINE_SAMPLES);
    std::vector<trail::Spline> splines = this->generateSplines();
    Eigen::ArrayXd interval = Eigen::ArrayXd::LinSpaced(SPLINE_SAMPLES, 0, 1);

    double lastDistance = 0;
    for (int i = 0; i < this->mNumOfSegments; ++i) {
        trail::Spline current = splines[i];
        auto df = [&current] (double t) {
            return current.velocity(t);
        };

        for (int j = 0; j < SPLINE_SAMPLES; ++j) {
            double t = interval[j];
            Eigen::Vector2d pos = current.position(t);
            Eigen::Vector2d vel = current.velocity(t);
            double theta = atan2(vel(1, 0), vel(0, 0));

            double r = this->mRobot.getBaseWidth() / 2.0;
            Eigen::Vector2d normal(-sin(theta), cos(theta));
            Eigen::Vector2d leftPos = pos + r * normal;
            Eigen::Vector2d rightPos = pos - r * normal;

            trail::Vector12d vec;
            vec(0, 0) = i + t; // t
            vec(1, 0) = (double) 1.0 / SPLINE_SAMPLES; // dt
            vec(2, 0) = lastDistance + lengthIntegral(0, t, df); // p(t)
            vec(3, 0) = 0; // v(t), TODO
            vec(4, 0) = 0; // a(t), TODO
            vec(5, 0) = 90 - degrees(theta); // θ(t)
            vec(6, 0) = pos(0, 0); // x(t)
            vec(7, 0) = pos(1, 0); // y(t)
            vec(8, 0) = leftPos(0, 0); // x_l(t)
            vec(9, 0) = leftPos(1, 0); // y_l(t)
            vec(10, 0) = rightPos(0, 0); // x_r(t)
            vec(11, 0) = rightPos(1, 0); // y_r(t)

            curve[j + (SPLINE_SAMPLES - 1) * i] = vec;
        }

        lastDistance += lengthIntegral(0, 1, df);
    }

    return curve;
}

int trail::RobotTrajectory::curveSize() {
    return this->mNumOfSegments * SPLINE_SAMPLES;
}
