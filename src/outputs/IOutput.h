#ifndef TRAIL_IOUTPUT_H
#define TRAIL_IOUTPUT_H

#include <utility>

#include "../trail/RobotTrajectory.h"

namespace trail {
    class IOutput {
    protected:
        RobotTrajectory mTrajectory;

    public:
        explicit IOutput(trail::RobotTrajectory trajectory) {
            this->mTrajectory = std::move(trajectory);
        }

        virtual void render() = 0;
    };
}

#endif //TRAIL_IOUTPUT_H
