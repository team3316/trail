#ifndef TRAIL_CSVOUTPUT_H
#define TRAIL_CSVOUTPUT_H

#include <fstream>
#include <string>
#include <utility>

#include "IOutput.h"

#if defined(__APPLE__) || defined(__linux__) || defined(__unix__) // Unix
    #include <unistd.h>
    #define DELIMETER "/"
    #define CWD getcwd
#else // Windows
    #include <direct.h>
    #define DELIMETER "\\"
    #define CWD _getcwd
#endif

namespace trail {
    class CSVOutput: public IOutput {
    private:
        std::ofstream mCSV;

        std::string cwd() {
            char temp[FILENAME_MAX];
            CWD(temp, FILENAME_MAX);
            return std::string(temp);
        }

    public:
        CSVOutput(RobotTrajectory trajectory, const std::string &filename): IOutput(std::move(trajectory)) {
            this->mCSV.open(cwd() + DELIMETER + filename);
        }

        void render() override {
            trail::Vector12d *curve;
            int len;
            std::tie(curve, len) = this->mTrajectory.calculateTrajectory();

            this->mCSV << "t,dt,s,v,a,theta,x,y,xl,yl,xr,yr\n";

            for (int i = 0; i < len - 1; ++i) {
                trail::Vector12d current = curve[i];

                for (int j = 0; j < 12; ++j) {
                    this->mCSV << std::to_string(current(j, 0));
                    if (j != 11) this->mCSV << ",";
                }

                this->mCSV << "\n";
            }

            this->mCSV.close();
        }
    };
}

#endif //TRAIL_CSVOUTPUT_H
