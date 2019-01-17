#ifndef TRAIL_LIBRARY_H
#define TRAIL_LIBRARY_H

namespace trail {
    class Robot {
    private:
        double mMass;
        double mBaseWidth;
        double mFreeSpeed;
        double mStallTorque;
        double mGearRatio;
        double mWheelRadius;
        int mNumOfDriveMotors;

    public:
        Robot(
                double mass,
                double baseWidth,
                double freeSpeed,
                double stallTorque,
                double gearRatio,
                double wheelRadius,
                int numOfDriveMotors
        );

        double timeToMaxVelocity(double i = 5);
        double distanceToMaxVelocity();
        double maxAcceleration();
    };
}

#endif