#ifndef TRAIL_CSVOUTPUT_H
#define TRAIL_CSVOUTPUT_H

#include <fstream>
#include <string>
#include <utility>

#include "../trail/utils.h"
#include "IOutput.h"

namespace trail {
    class CSVOutput: public IOutput {
    private:
        std::ofstream mCSV;

    public:
        CSVOutput(RobotTrajectory trajectory, const std::string &filename): IOutput(std::move(trajectory)) {
            this->mCSV.open(cwd() + DELIMETER + filename);
        }

        void render() override {
            trail::Vector17d *curve;
            int len;
            std::tie(curve, len) = this->mTrajectory.calculateTrajectory();

            this->mCSV << "t,dt,s,v,a,theta,omega,x,y,xl,yl,xr,yr,sl,vl,sr,vr\n";

            for (int i = 0; i < len - 1; ++i) {
                trail::Vector17d current = curve[i];

                for (int j = 0; j < 17; ++j) {
                    this->mCSV << std::to_string(current(j, 0));
                    if (j != 16) this->mCSV << ",";
                }

                this->mCSV << "\n";
            }

            this->mCSV.close();
        }
    };
}

#endif //TRAIL_CSVOUTPUT_H
