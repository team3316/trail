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

Eigen::Vector3d *trail::RobotTrajectory::calculateTrajectory() {
    auto curve = (Eigen::Vector3d *) malloc(sizeof(Eigen::Vector3d) * this->mNumOfSegments * SPLINE_SAMPLES);
    std::vector<trail::Spline> splines = this->generateSplines();

    Eigen::ArrayXd interval = Eigen::ArrayXd::LinSpaced(SPLINE_SAMPLES, 0, 1);

    for (int i = 0; i < this->mNumOfSegments; ++i) {
        trail::Spline current = splines[i];
        for (int j = 0; j < SPLINE_SAMPLES; ++j) {
            double t = interval[j];
            Eigen::Vector2d pos = current.position(t);
            Eigen::Vector2d vel = current.velocity(t);
            double heading = degrees(atan2(vel(1, 0), vel(0, 0)));

            curve[j + (SPLINE_SAMPLES - 1) * i] = Eigen::Vector3d(pos(0, 0), pos(1, 0), heading);
        }
    }

    return curve;
}

int trail::RobotTrajectory::curveSize() {
    return this->mNumOfSegments * SPLINE_SAMPLES;
}
