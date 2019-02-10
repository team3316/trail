#ifndef TRAIL_ROBOTTRAJECTORY_H
#define TRAIL_ROBOTTRAJECTORY_H

#include <vector>
#include <tuple>
#include <string>

#include "MotionProfile.h"
#include "Robot.h"
#include "Waypoint.h"
#include "Spline.h"

#define DT 0.02 // Cycle time

namespace trail {
    using Waypoints = std::vector<trail::Waypoint>;
    using Vector17d = Eigen::Matrix<double, 17, 1>;

    class RobotTrajectory {
    private:
        trail::MotionProfile mMotionProfile;
        trail::Waypoints mWaypoints;
        trail::Robot mRobot;
        int mNumOfSegments;
        std::string mName = "path";

        std::vector<trail::Spline> generateSplines();

    public:
        RobotTrajectory(trail::Waypoints waypoints, trail::Robot robot);

        RobotTrajectory(trail::Waypoints waypoints, trail::Robot robot, std::string name) : RobotTrajectory(waypoints, robot) {
            this->mName.assign(name);
        };

        RobotTrajectory();

        std::string getName();

        std::tuple<trail::Vector17d *, int> calculateTrajectory();
        int curveSize() { return 0; };

        static trail::RobotTrajectory fromJSON(const std::string &filename, trail::Robot robot);
    };
}

#endif //TRAIL_ROBOTTRAJECTORY_H
