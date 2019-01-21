#include <string>

#include "trail/utils.h"
#include "trail/RobotTrajectory.h"

#include "outputs/DesmosOutput.h"
#include "outputs/CSVOutput.h"

using namespace trail;

int main(int argc, char **argv) {
    bool hasTwoFiles = argc > 2;
    if (!hasTwoFiles) {
        LOGE("Creating a motion profile requires supplying both a robot spec and a profile spec." << std::endl << "Aborting.");
        return -1;
    }

    Robot robot = Robot::fromJSON(argv[1]);
    LOGD("Loaded robot spec");

    RobotTrajectory trajectory = RobotTrajectory::fromJSON(argv[2], robot);
    LOGD("Loaded trajectory " << trajectory.getName());

    LOGD("Creating CSV file...");
    CSVOutput(trajectory, trajectory.getName() + ".csv").render();
    LOGD("Created CSV file!");

    bool shouldDisplayDesmos = argc > 3 && (std::strcmp(argv[3], "--desmos") != 0 || std::strcmp(argv[3], "-d") != 0);
    if (shouldDisplayDesmos) {
        LOGD("Desmos Output:");
        DesmosOutput(trajectory).render();
    }
}
