#ifndef TRAIL_MOTIONPROFILE_H
#define TRAIL_MOTIONPROFILE_H

#include <Eigen/Dense>

#include "Robot.h"

namespace trail {
    class MotionProfile {
    private:
        double mMaxVelocity;
        double mMaxAcceleration;
        double mMaxJerk;

        double mCruiseVelocity;

        double mMinDistance;
        double mFullDistance;

        /**
         * Calculates the needed distance required for the robot to reach end_vel in maximum acceleration. Calculated using
         * the kinematics formula:
         *     end_vel^2 = start_vel^2 + 2 * acceleration * delta_x
         * @param startVel The starting velocity.
         * @param endVel The end velocity.
         * @return The required distance.
         */
        double distanceToVelocity(double startVel, double endVel);

    public:
        MotionProfile(double maxVelocity, double maxAcceleration, double maxJerk, double cruiseVelocity);

        MotionProfile();

        void setDistance(double distance);

        Eigen::Vector4d calculate(double t);
        Eigen::Vector4d calculateSCurve(double t);
        double getTotalTime();
    };
}

#endif //TRAIL_MOTIONPROFILE_H
