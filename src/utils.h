#ifndef TRAIL_UTILS_H
#define TRAIL_UTILS_H

#include <Eigen/Dense>
#include <functional>

#define LENGTH_INTEGRAL_SAMPLES 600 // The default number of samples for calculating spline length
#define PI 3.14159265359
#define sgn(x) ((x < 0) ? -1 : ((x > 0) ? 1 : 0))
#define radians(x) x * PI / 180
#define degrees(x) x * 180 / PI

/**
 * Calculates a segment's length using its derivative, from t0 to t1.
 * The intergal is:
 *   I_t0^t1 sqrt((df_x(t))^2 + (df_y(t))^2)dt
 * The implementation of the function is taken from the old dbug-trajectory project and optimized to C++.
 * @param t0 The lower bound of the integral, 0 <= t0 <= 1.
 * @param t1 The upper bound of the integral, 0 <= t0 <= 1.
 * @param df The function's derivative, as a function.
 * @param n The number of samples in the given derivative array.
 * @return An approximated segment length, in the segment's units, using Simpson's rule.
 */
double lengthIntegral(double t0, double t1, std::function<Eigen::Vector2d (double)> df, int n = LENGTH_INTEGRAL_SAMPLES);

#endif //TRAIL_UTILS_H
