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
        /**
         * Creates a new Robot instance. At the moment this is only configured to work with transmissions that have only
         * one type of motors connected to them (we use the VEXpro SSDR with 2 CIMs)
         * @param mass The mass of the robot. Units - kg
         * @param baseWidth The wheelbase width of the robot. Units - m
         * @param freeSpeed The free speed of the robot (of one side). Units - m/s
         * @param stallTorque The stall torque of a single motor connected to the transmission. Units - N*m
         * @param gearRatio The final (input to output, not stages) gear ratio of a transmission.
         * @param wheelRadius The drivetrain's wheel radius (provided that all of the wheels have a single radius). Units - m
         * @param numOfDriveMotors The number of total drive motors (number of motors on the drivetrain)
         */
        Robot(
                double mass,
                double baseWidth,
                double freeSpeed,
                double stallTorque,
                double gearRatio,
                double wheelRadius,
                int numOfDriveMotors
        );

        /**
         * Default constructor. Initializes everything to be -1. Not intended for general usage
         */
        Robot();

        /**
         * Calculates the time required to get the robot to 99.75% of its free speed, based on solving the ODE
         * given here: https://drive.google.com/file/d/0B_lA0xR4_viqbTJYanE4X3VnWE0/view [2] and calculating its
         * time constant.
         * @param i The number to multiply the time constant in. 5 results in 99.32% of free speed, 4 (default) is 98.17%.
         * @return The time to reach 98.17% of the robot's free speed using full power, in seconds.
         */
        double timeToMaxVelocity(double i = 4);

        /**
         * Calculates the maximum acceleration of the robot using Newton's 2nd law.
         * F_max = ma ==> a_max = F_max / m
         * @return The maximum acceleration
        */
        double maxAcceleration();

        /**
         * @return The wheelbase width
         */
        double getBaseWidth();

        /**
         * @return The robot's free speed
         */
        double getFreeSpeed();
    };
}

#endif