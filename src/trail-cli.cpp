#include "trail/RobotTrajectory.h"
#include "outputs/DesmosOutput.h"
#include "outputs/CSVOutput.h"

#include <iostream>
#include <nlohmann/json.hpp>

using namespace trail;

int main(int argc, char **argv) {
//    Waypoints wps = {
//        ORIGIN,
//        Waypoint(0, 1, 0)
//    };
//
//    RobotTrajectory trajectory(wps, mars);
//
//    DesmosOutput(trajectory).render();
//    CSVOutput(trajectory, "test.csv").render();

    bool hasTwoFiles = argc > 2;
    if (!hasTwoFiles) {
        LOGE("Creating a motion profile requires supplying both a robot spec and a profile spec." << std::endl << "Aborting.");
        return -1;
    }

    Robot robot = Robot::fromJSON("mars.json");
    LOGD("Loaded robot spec");
}
