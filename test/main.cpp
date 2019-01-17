//
// Created by Jonathan Ohayon on 2019-01-17.
//

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "../library.h"
#include "../Spline.h"

#define EPSILON 1E-4

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

    EXPECT_NEAR(amax, 47.6386, EPSILON);
}

TEST(SplineTest, NewSpline) {
    Spline spline(QUINTIC_HERMITE, {});

    MatrixXd mat;
    mat << QUINTIC_HERMITE_BASIS_MATRIX;

    ASSERT_EQ(mat, spline.getBasisMatrix());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
