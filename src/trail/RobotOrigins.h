#ifndef TRAIL_ROBOTORIGINS_H
#define TRAIL_ROBOTORIGINS_H

#include <Eigen/Dense>
#include <tuple>
#include "Waypoint.h"

namespace trail {
    class RobotOrigins {
    private:
        trail::Waypoint mWaypoint;
        Eigen::Vector2d mNormal, mDNormal;
        double mRadius, mVelScale;

    public:
        RobotOrigins(trail::Waypoint startWaypoint, double scale, double radius);

        ~RobotOrigins() = default;

        std::tuple<Eigen::Vector2d, Eigen::Vector2d> getPositionOrigins();

        std::tuple<Eigen::Vector2d, Eigen::Vector2d> getVelocityOrigins();
    };
}

#endif
