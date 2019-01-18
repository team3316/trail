#ifndef TRAIL_SPLINE_H
#define TRAIL_SPLINE_H

#include <Eigen/Dense>
#include <vector>

#define QUINTIC_HERMITE_BASIS_MATRIX -6, -3, -0.5, 6, -3, 0.5, \
                                     15, 8, 1.5, -15, 7, -1, \
                                     -10, -6, -1.5, 10, -4, 0.5, \
                                     0, 0, 0.5, 0, 0, 0, \
                                     0, 1, 0, 0, 0, 0, \
                                     1, 0, 0, 0, 0, 0

using PointVector = std::vector<Eigen::Vector2d>;

namespace trail {
    typedef enum {
        POSITION, VELOCITY, ACCELERATION
    } CurveType;

    class Spline {
    private:
        Eigen::Matrix<double, 6, 6> mBasisMatrix; // The Hermite polynomial basis matrix
        Eigen::Matrix<double, 6, 1> mControlPoints_x; // The x control points
        Eigen::Matrix<double, 6, 1> mControlPoints_y; // The y control points

        Eigen::Matrix<double, 1, 6> polynomialBasis(double t, CurveType type);
        Eigen::Vector2d calculate(double t, CurveType type);

    public:
        /**
         * Constructor
         */
        Spline(PointVector controlPoints);
        ~Spline() = default;

        /**
         * Returns the spline's position curve in spline param t.
         * @param t The spline parameter. 0 <= t <= 1
         * @return A point in R^2, p(t)
         */
        Eigen::Vector2d position(double t);

        /**
         * Returns the spline's velocity (first derivative) curve in spline param t.
         * @param t The spline parameter. 0 <= t <= 1
         * @return A point in R^2, v(t)
         */
        Eigen::Vector2d velocity(double t);

        /**
         * Returns the spline's acceleration (second derivative) curve in spline param t.
         * @param t The spline parameter. 0 <= t <= 1
         * @return A point in R^2, v(t)
         */
        Eigen::Vector2d acceleration(double t);
    };
}

#endif //TRAIL_SPLINE_H
