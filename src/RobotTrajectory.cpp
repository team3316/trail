//
// Created by Jonathan Ohayon on 2019-01-18.
//

#include <math.h>

#include "utils.h"

#include "RobotTrajectory.h"

trail::RobotTrajectory::RobotTrajectory(trail::Waypoints waypoints, trail::Robot robot) {
    this->mWaypoints = waypoints;
    this->mRobot = robot;
    this->mNumOfSegments = (int) waypoints.size() - 1;
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
    auto curve = (trail::Vector12d *) malloc(sizeof(trail::Vector12d) * this->mNumOfSegments * SPLINE_SAMPLES);
    std::vector<trail::Spline> splines = this->generateSplines();

    Eigen::ArrayXd interval = Eigen::ArrayXd::LinSpaced(SPLINE_SAMPLES, 0, 1);

    for (int i = 0; i < this->mNumOfSegments; ++i) {
        trail::Spline current = splines[i];
        auto df = [&current] (double t) {
            return current.velocity(t);
        };

        for (int j = 0; j < SPLINE_SAMPLES; ++j) {
            double t = interval[j];
            Eigen::Vector2d pos = current.position(t);
            Eigen::Vector2d vel = current.velocity(t);
            double heading = degrees(atan2(vel(1, 0), vel(0, 0)));

            trail::Vector12d vec;

            vec(0, 0) = i + (j / SPLINE_SAMPLES); // t
            vec(1, 0) = 1 / SPLINE_SAMPLES; // dt
            vec(2, 0) = lengthIntegral(0, t, df); // p(t)
            vec(3, 0) = 0; // v(t), TODO
            vec(4, 0) = 0; // a(t), TODO
            vec(5, 0) = heading; // θ(t)
            vec(6, 0) = pos(0, 0); // x(t)
            vec(7, 0) = pos(1, 0); // y(t)
            vec(8, 0) = 0; // x_l(t), TODO
            vec(9, 0) = 0; // y_l(t), TODO
            vec(10, 0) = 0; // x_r(t), TODO
            vec(11, 0) = 0; // y_r(t), TODO

            curve[j + (SPLINE_SAMPLES - 1) * i] = vec;
        }
    }

    return curve;
}

int trail::RobotTrajectory::curveSize() {
    return this->mNumOfSegments * SPLINE_SAMPLES;
}
