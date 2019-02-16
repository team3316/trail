#include <iostream>
#include <cmath>

#include "Spline.h"

/**
 * Calculates x to the power of p, with differentiation.
 * @param x The number to take the power of
 * @param p The power to use
 * @param diff The differentiation number - meaning, 0th derivative, 1st derivative, 2nd derivative...
 * @return If the result should be constant, then the coefficient that it's supposed to have. If it should be 0 then 0.
 *         Otherwise, the diff-th derivative of x^p.
 */
double pow_diff(double x, int p, int diff) {
    double coeff = 1;
    for (int i = 0; i < diff; i++) coeff *= p - i;

    return p - diff == 0 ? coeff : (p - diff == -1 ? 0 : coeff * pow(x, p - diff));
}

trail::Spline::Spline(PointVector controlPoints) {
    this->mBasisMatrix << QUINTIC_HERMITE_BASIS_MATRIX;

    for (size_t i = 0; i < controlPoints.size(); i++) {
        Eigen::Vector2d point = controlPoints[i];
        this->mControlPoints_x.row(i) = point.row(0);
        this->mControlPoints_y.row(i) = point.row(1);
    }
}

Eigen::Matrix<double, 1, 6> trail::Spline::polynomialBasis(double t, CurveType type) {
    Eigen::Matrix<double, 1, 6> time;
    time << pow_diff(t, 5, type),
            pow_diff(t, 4, type),
            pow_diff(t, 3, type),
            pow_diff(t, 2, type),
            pow_diff(t, 1, type),
            type == POSITION ? 1 : 0;
    return time;
}

Eigen::Vector2d trail::Spline::calculate(double t, trail::CurveType type) {
    Eigen::Matrix<double, 1, 6> tv = this->polynomialBasis(t, type);

    Eigen::Vector2d result;

    double x = tv * this->mBasisMatrix * this->mControlPoints_x;
    double y = tv * this->mBasisMatrix * this->mControlPoints_y;
    result << x, y;

    return result;
}

Eigen::Vector2d trail::Spline::position(double t) {
    return this->calculate(t, POSITION);
}

Eigen::Vector2d trail::Spline::velocity(double t) {
    return this->calculate(t, VELOCITY);
}

Eigen::Vector2d trail::Spline::acceleration(double t) {
    return this->calculate(t, ACCELERATION);
}

double trail::Spline::curvature(double t) {
    Eigen::Vector2d vel = this->velocity(t);

    // Using determinant in order to compute the enumerator of the curvature
    Eigen::Matrix2d accAndVel = Eigen::Matrix2d::Zero();
    accAndVel.col(0) = this->acceleration(t);
    accAndVel.col(1) = vel;

    return -accAndVel.determinant() / pow(vel.norm(), 3);
}
