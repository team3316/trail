#include "trail/RobotTrajectory.h"
#include "outputs/DesmosOutput.h"
#include "outputs/CSVOutput.h"

using namespace trail;

int main(int argc, char **argv) {
    Robot mars(
        45.3592, // Mass - kg
        0.591312, // Robot base width - m
        5.0048, // Free speed - m/s
        2.42, // Motor's stall torque - Nm
        5.67, // Gearbox gear ratio
        0.0508, // Wheel radis - m
        4 // Number of motors on drivetrain
    );

    Waypoints wps = {
        ORIGIN,
        Waypoint(0, 1, 0),
//        Waypoint(1, 2, 45)
    };

    RobotTrajectory trajectory(wps, mars);

    DesmosOutput(trajectory).render();
}
