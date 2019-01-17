//
// Created by Jonathan Ohayon on 2019-01-17.
//

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
        QUINTIC_HERMITE, CUBIC_HERMITE
    } SplineType;

    class Spline {
    private:
        SplineType mType;
        Eigen::MatrixXd mBasisMatrix;
        PointVector mControlPoints;

    public:
        Spline(SplineType type, PointVector controlPoints);
        ~Spline() = default;

        Eigen::MatrixXd getBasisMatrix();
    };
}

#endif //TRAIL_SPLINE_H
