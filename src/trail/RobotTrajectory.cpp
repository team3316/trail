#include <cmath>

#include "utils.h"

#include "RobotTrajectory.h"
#include "RobotOrigins.h"

trail::RobotTrajectory::RobotTrajectory(trail::Waypoints waypoints, trail::Robot robot) {
    this->mWaypoints = waypoints;
    this->mRobot = robot;
    this->mNumOfSegments = (int) waypoints.size() - 1;
    this->mMotionProfile = MotionProfile(
        robot.getFreeSpeed(), // Max velocity; Units - m/s
        robot.maxAcceleration(), // Max acceleration; Units - m/s/s
        60, // Max jerk; Units - m/s/s/s; This is the default value for the KoP chassis
        0.75 * robot.getFreeSpeed() // Cruise velocity; Units - m/s
    );
}

trail::RobotTrajectory::RobotTrajectory() {
    this->mWaypoints = {};
}

std::string trail::RobotTrajectory::getName() {
    return this->mName;
}

std::vector<trail::Spline> trail::RobotTrajectory::generateSplines() {
    std::vector<trail::Spline> splines;

    for (int i = 0; i < this->mNumOfSegments; ++i) {
        trail::Waypoint current = this->mWaypoints[i],
                        next = this->mWaypoints[i + 1];
        double dist = current.distanceToPoint(next);

        PointVector points = {
            current.toPoint(),
            current.firstDerivative(1.5 * dist),
            current.secondDerivative(),
            next.toPoint(),
            next.firstDerivative(1.5 * dist),
            next.secondDerivative()
        };

        splines.emplace_back(Spline(points));
    }

    return splines;
}

std::vector<trail::RobotOrigins> trail::RobotTrajectory::generateOrigins() {
    std::vector<trail::RobotOrigins> origins;
    double r = this->mRobot.getBaseWidth() / 2.0;

    for (int i = 0; i < this->mNumOfSegments; ++i) {
        trail::Waypoint current = this->mWaypoints[i],
                        next = this->mWaypoints[i + 1];
        double dist = current.distanceToPoint(next);

        origins.emplace_back(RobotOrigins(current, 1.5 * dist, r));
    }

    return origins;
}

std::tuple<trail::Vector18d *, int> trail::RobotTrajectory::calculateTrajectory() {
    std::vector<trail::Spline> splines = this->generateSplines();
    std::vector<trail::RobotOrigins> origins = this->generateOrigins();

    double totalMiddleDistance;
    std::vector<double> distances;
    for (auto spline: splines) {
        double currentDist = lengthIntegral(0, 1, [&spline] (double t) {
            return spline.velocity(t);
        });

        totalMiddleDistance += currentDist;
        distances.emplace_back(currentDist);
    }

    this->mMotionProfile.setDistance(totalMiddleDistance);
    double totalTime = this->mMotionProfile.getTotalTime();
    int samplesPerSpline = (int) ceil(totalTime / DT / this->mNumOfSegments);

    auto curve = (trail::Vector18d *) std::malloc(sizeof(trail::Vector18d) * this->mNumOfSegments * samplesPerSpline);
    Eigen::ArrayXd interval = Eigen::ArrayXd::LinSpaced(samplesPerSpline, 0, 1);

    double r = this->mRobot.getBaseWidth() / 2.0;
    double lastTime = 0;
    for (int i = 0; i < this->mNumOfSegments; ++i) {
        trail::Spline current = splines[i];
        trail::RobotOrigins currentOrigins = origins[i];

        Eigen::Vector2d leftPosOrigin, rightPosOrigin, leftVelOrigin, rightVelOrigin;
        std::tie(leftPosOrigin, rightPosOrigin) = currentOrigins.getPositionOrigins();
        std::tie(leftVelOrigin, rightVelOrigin) = currentOrigins.getVelocityOrigins();

        for (int j = 0; j < samplesPerSpline; ++j) {
            double percentage = interval[j];
            Eigen::Vector2d pos = current.position(percentage);
            Eigen::Vector2d vel = current.velocity(percentage);
            Eigen::Vector2d acc = current.acceleration(percentage);
            double curvature = current.curvature(percentage);

            double theta = atan2(vel(1, 0), vel(0, 0));
            double velHypot = hypot(vel(0, 0), vel(1, 0));
            double omega = degrees(velHypot == 0 ? 0 : (acc(1, 0) * vel(0, 0) - acc(0, 0) * vel(1, 0)) / velHypot);
            double heading = 90 - degrees(theta);

            Eigen::Vector2d normal(-sin(theta), cos(theta));
            Eigen::Vector2d leftPos = pos + r * normal;
            Eigen::Vector2d rightPos = pos - r * normal;

            Eigen::Vector2d dnormal(-cos(theta), sin(theta));
            Eigen::Vector2d leftVel = vel + r * dnormal;
            Eigen::Vector2d rightVel = vel - r * dnormal;

            Eigen::Vector4d mpState = this->mMotionProfile.calculate(lastTime);

            trail::Vector18d vec;
            vec(0, 0) = lastTime; // t
            vec(1, 0) = DT; // dt
            vec(2, 0) = mpState(0, 0); // s(t)
            vec(3, 0) = mpState(1, 0); // v(t)
            vec(4, 0) = mpState(2, 0); // a(t)
            vec(5, 0) = heading; // θ(t)
            vec(6, 0) = omega; // ω(t)
            vec(7, 0) = pos(0, 0); // x(t)
            vec(8, 0) = pos(1, 0); // y(t)
            vec(9, 0) = leftPos(0, 0); // x_l(t)
            vec(10, 0) = leftPos(1, 0); // y_l(t)
            vec(11, 0) = rightPos(0, 0); // x_r(t)
            vec(12, 0) = rightPos(1, 0); // y_r(t)
            vec(13, 0) = (leftPos - leftPosOrigin).norm(); // s_l(t)
            vec(14, 0) = (leftVel - leftVelOrigin).norm(); // v_l(t)
            vec(15, 0) = (rightPos - rightPosOrigin).norm(); // s_r(t)
            vec(16, 0) = (rightVel - rightVelOrigin).norm(); // v_r(t)
            vec(17, 0) = curvature; // k(t)

            curve[j + (samplesPerSpline * i)] = vec;

            lastTime += DT;
        }
    }

    return std::make_tuple(curve, this->mNumOfSegments * samplesPerSpline);
}

trail::RobotTrajectory trail::RobotTrajectory::fromJSON(const std::string &filename, trail::Robot robot) {
    std::string file;
    if (readFile(filename, &file) < 0) {
        LOGE("Couln't read " << filename << "; Aborting.");
        exit(-1);
    }

    auto json = json::parse(file);

    std::vector<std::string> requiredKeys = {
        "name",
        "waypoints"
    };
    for (auto key: requiredKeys) {
        if (json.find(key) == json.end()) {
            LOGE(key << " is missing from " << filename << ". Please add it and re-run trail. Aborting.");
            exit(-1);
        }
    }

    Waypoints wps = json.at("waypoints").get<Waypoints>();
    return {wps, robot, json.at("name")};
}
