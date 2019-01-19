#ifndef TRAIL_ROBOTTRAJECTORY_H
#define TRAIL_ROBOTTRAJECTORY_H

#include <vector>

#include "MotionProfile.h"
#include "Robot.h"
#include "Waypoint.h"
#include "Spline.h"

#define SPLINE_SAMPLES 101 // 10ms cycle

namespace trail {
    using Waypoints = std::vector<trail::Waypoint>;
    using Vector12d = Eigen::Matrix<double, 12, 1>;

    class RobotTrajectory {
    private:
        trail::MotionProfile mMotionProfile;
        trail::Waypoints mWaypoints;
        trail::Robot mRobot;
        int mNumOfSegments;

        std::vector<trail::Spline> generateSplines();

    public:
        RobotTrajectory(trail::Waypoints waypoints, trail::Robot robot);

        RobotTrajectory();

        trail::Vector12d *calculateTrajectory();
        int curveSize();
    };
}

#endif //TRAIL_ROBOTTRAJECTORY_H
