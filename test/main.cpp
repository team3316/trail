#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "../src/Robot.h"
#include "../src/Spline.h"
#include "../src/utils.h"
#include "../src/Waypoint.h"
#include "../src/RobotTrajectory.h"

#define EPSILON 1E-4 // Accuracy to 4 decimal places is good enough for us
#define EXPECT_PT_EQ(pt1, pt2) EXPECT_NEAR(pt1[0], pt2[0], EPSILON); \
    EXPECT_NEAR(pt1[1], pt2[1], EPSILON);
#define EXPECT_WP_EQ(wp, pt) EXPECT_NEAR(wp.getX(), pt[6], EPSILON); \
    EXPECT_NEAR(wp.getY(), pt[7], EPSILON);

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

    EXPECT_NEAR(ttvf, 0.4202, EPSILON);
}

TEST(RobotTest, MaxAcceleration) {
    double amax = mars.maxAcceleration();

    EXPECT_NEAR(amax, 23.8193, EPSILON);
}

TEST(SplineTest, ControlPoints) {
    PointVector cps = {
        {0.0, 0.0}, // p0
        {cos(radians(90)), sin(radians(90))}, // dp0
        {-sin(radians(90)), cos(radians(90))}, // d2p0
        {3.0, 4.0}, // p1
        {cos(0.0), sin(0.0)}, // dp1
        {-sin(0.0), cos(0.0)} // d2p1
    };

    Spline spline(cps);
    Vector2d p0 = spline.position(0),
             p1 = spline.position(1);
    EXPECT_PT_EQ(p0, cps[0]);
    EXPECT_PT_EQ(p1, cps[3]);

    Vector2d dp0 = spline.velocity(0),
             dp1 = spline.velocity(1);
    EXPECT_PT_EQ(dp0, cps[1]);
    EXPECT_PT_EQ(dp1, cps[4]);

    Vector2d d2p0 = spline.acceleration(0),
             d2p1 = spline.acceleration(1);
    EXPECT_PT_EQ(d2p0, cps[2]);
    EXPECT_PT_EQ(d2p1, cps[5]);
}

TEST(UtilsTest, LengthIntegral) {
    auto traj = [](double t) { // Using a constant term as a derivative for simplicity
        Vector2d d;
        d << 3, 4;
        return d;
    };

    double len = lengthIntegral(0, 1, traj);
    ASSERT_NEAR(len, 5, EPSILON);
}

TEST(UtilsTest, WaypointDistance) {
    Waypoint p1(0, 0, 0);
    Waypoint p2(3, 4, 90);

    ASSERT_NEAR(p1.distanceToPoint(p2), 5, EPSILON);
}

TEST(TrajectoryTest, CreateTrajectory) {
    Waypoints wps = {
        ORIGIN,
        Waypoint(3, 4, 90)
    };

    RobotTrajectory trajectory(wps, mars);

    auto curve = trajectory.calculateTrajectory();

    EXPECT_WP_EQ(ORIGIN, curve[0]);
    EXPECT_WP_EQ(Waypoint(3, 4, 90), curve[100]);

    free(curve);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
