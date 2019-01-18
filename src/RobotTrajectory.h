//
// Created by Jonathan Ohayon on 2019-01-18.
//

#ifndef TRAIL_ROBOTTRAJECTORY_H
#define TRAIL_ROBOTTRAJECTORY_H

#include <vector>

#include "Robot.h"
#include "Waypoint.h"
#include "Spline.h"

#define SPLINE_SAMPLES 100 // 10ms cycle

namespace trail {
    using Waypoints = std::vector<trail::Waypoint>;

    class RobotTrajectory {
    private:
        trail::Waypoints mWaypoints;
        trail::Robot mRobot;
        int mNumOfSegments;

        std::vector<trail::Spline> generateSplines();

    public:
        RobotTrajectory(trail::Waypoints waypoints, trail::Robot robot);

        Eigen::Vector2d *calculateCurve();
        int curveSize();
    };
}

#endif //TRAIL_ROBOTTRAJECTORY_H
