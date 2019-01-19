#ifndef TRAIL_DESMOSOUTPUT_H
#define TRAIL_DESMOSOUTPUT_H

#include <iostream>

#include "../trail/RobotTrajectory.h"

#include "IOutput.h"

#define print(x) (std::cout << (x) << std::endl << std::flush)

namespace trail {
    class DesmosOutput: public IOutput {

    using IOutput::IOutput;

    public:
        void render() override {
            auto curve = this->mTrajectory.calculateTrajectory();
            int len = this->mTrajectory.curveSize();

            auto printCurveData = [&curve, &len] (std::string title, int i0, int i1) {
                print(title + ":");
                for (int i = 0; i < len - 1; ++i) {
                    trail::Vector12d current = curve[i];
                    std::cout << "(" << std::to_string(current(i0, 0)) << "," << std::to_string(current(i1, 0)) << ")";

                    if (i != len - 2) std::cout << ",";
                }
                std::cout << std::endl << std::flush;
            };

            printCurveData("Trajectory", 6, 7);
            printCurveData("Heading", 0, 5);
            printCurveData("Distance", 0, 2);
            printCurveData("Velocity", 0, 3);
            printCurveData("Acceleration", 0, 4);
            printCurveData("Left Trajectory", 8, 9);
            printCurveData("Right Trajectory", 10, 11);

            std::free(curve);
        }
    };
}

#endif //TRAIL_DESMOSOUTPUT_H
