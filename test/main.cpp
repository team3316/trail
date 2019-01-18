//
// Created by Jonathan Ohayon on 2019-01-17.
//

#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "../src/Robot.h"
#include "../src/Spline.h"

#define PI 3.14159265359
#define EPSILON 1E-4
#define RAD(x) x * PI / 180
#define ASSERT_PT_EQ(pt1, pt2) ASSERT_NEAR(pt1[0], pt2[0], EPSILON); \
    ASSERT_NEAR(pt1[1], pt2[1], EPSILON);

using namespace trail;
using namespace Eigen;

static Robot mars(
    45.3592, // Mass - kg
    0.591312, // Robot base width - m
    5.0048, // Free speed - m/s
    2.42, // Motor's stall torque - Nm
    5.67, // Gearbox gear ratio
    0.0508, // Wheel radis - m
    4 // Number of motors on drivetrain
);

TEST(RobotTest, TimeToMax) {
    double ttvf = mars.timeToMaxVelocity();

    EXPECT_NEAR(ttvf, 0.5252, EPSILON);
}

TEST(RobotTest, MaxAcceleration) {
    double amax = mars.maxAcceleration();

    EXPECT_NEAR(amax, 23.8193, EPSILON);
}

TEST(SplineTest, ControlPoints) {
    PointVector cps = {
        {0.0, 0.0}, // p0
        {cos(RAD(90)), sin(RAD(90))}, // dp0
        {-sin(RAD(90)), cos(RAD(90))}, // d2p0
        {3.0, 4.0}, // p1
        {cos(0.0), sin(0.0)}, // dp1
        {-sin(0.0), cos(0.0)} // d2p1
    };

    Spline spline(cps);
    Vector2d p0 = spline.position(0),
             p1 = spline.position(1);
    ASSERT_PT_EQ(p0, cps[0]);
    ASSERT_PT_EQ(p1, cps[3]);

    Vector2d dp0 = spline.velocity(0),
             dp1 = spline.velocity(1);
    ASSERT_PT_EQ(dp0, cps[1]);
    ASSERT_PT_EQ(dp1, cps[4]);

    Vector2d d2p0 = spline.acceleration(0),
             d2p1 = spline.acceleration(1);
    ASSERT_PT_EQ(d2p0, cps[2]);
    ASSERT_PT_EQ(d2p1, cps[5]);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
