//
// Created by Jonathan Ohayon on 2019-01-17.
//

#include "Spline.h"

trail::Spline::Spline(trail::SplineType type, PointVector controlPoints) {
    this->mType = type;

    switch (type) {
        case QUINTIC_HERMITE:
            this->mBasisMatrix << QUINTIC_HERMITE_BASIS_MATRIX;
            break;
        case CUBIC_HERMITE:
        default:
            this->mBasisMatrix << QUINTIC_HERMITE_BASIS_MATRIX;
            break;
    }

    this->mControlPoints = controlPoints;
}

Eigen::MatrixXd trail::Spline::getBasisMatrix() {
    return this->mBasisMatrix;
}
