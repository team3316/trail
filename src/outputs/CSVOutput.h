#ifndef TRAIL_CSVOUTPUT_H
#define TRAIL_CSVOUTPUT_H

#include <fstream>
#include <string>
#include <utility>

#include "IOutput.h"

namespace trail {
    class CSVOutput: public IOutput {
    private:
        std::ofstream mCSV;

    public:
        CSVOutput(RobotTrajectory trajectory, const std::string &filename): IOutput(std::move(trajectory)) {
            this->mCSV.open(filename);
        }

        void render() override {
            auto curve = this->mTrajectory.calculateTrajectory();
            int len = this->mTrajectory.curveSize();

            this->mCSV << "t,dt,s,v,a,theta,x,y,xl,yl,xr,yr";

            for (int i = 0; i < len - 1; ++i) {
                trail::Vector12d current = curve[i];

                for (int j = 0; j < 12; ++j) {
                    this->mCSV << current(j, 0);
                    if (j != 11) this->mCSV << ",";
                }

                this->mCSV << "\n";
            }

            this->mCSV.close();
        }
    };
}

#endif //TRAIL_CSVOUTPUT_H
