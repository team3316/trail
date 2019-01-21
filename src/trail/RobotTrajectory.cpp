#include <cmath>

#include "utils.h"

#include "RobotTrajectory.h"

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
            {current.getX(), current.getY()},
            current.firstDerivative(1.5 * dist),
            current.secondDerivative(),
            {next.getX(), next.getY()},
            next.firstDerivative(1.5 * dist),
            next.secondDerivative()
        };

        splines.emplace_back(Spline(points));
    }

    return splines;
}

std::tuple<trail::Vector13d *, int> trail::RobotTrajectory::calculateTrajectory() {
    std::vector<trail::Spline> splines = this->generateSplines();

    double totalMiddleDistance = 0;
    for (auto spline: splines) {
        totalMiddleDistance += lengthIntegral(0, 1, [&spline] (double t) {
            return spline.velocity(t);
        });
    }

    this->mMotionProfile.setDistance(totalMiddleDistance);
    double totalTime = this->mMotionProfile.getTotalTime();
    int samplesPerSpline = (int) ceil(totalTime / DT / this->mNumOfSegments);

    auto curve = (trail::Vector13d *) std::malloc(sizeof(trail::Vector13d) * this->mNumOfSegments * samplesPerSpline);
    Eigen::ArrayXd interval = Eigen::ArrayXd::LinSpaced(samplesPerSpline, 0, 1);

    double lastTime = 0;
    for (int i = 0; i < this->mNumOfSegments; ++i) {
        trail::Spline current = splines[i];

        for (int j = 0; j < samplesPerSpline; ++j) {
            double percentage = interval[j];
            Eigen::Vector2d pos = current.position(percentage);
            Eigen::Vector2d vel = current.velocity(percentage);
            Eigen::Vector2d acc = current.acceleration(percentage);

            double theta = atan2(vel(1, 0), vel(0, 0));
            double velHypot = hypot(vel(0, 0), vel(1, 0));
            double omega = velHypot == 0 ? 0 : (acc(1, 0) * vel(0, 0) - acc(0, 0) * vel(1, 0)) / velHypot;

            double r = this->mRobot.getBaseWidth() / 2.0;
            Eigen::Vector2d normal(-sin(theta), cos(theta));
            Eigen::Vector2d leftPos = pos + r * normal;
            Eigen::Vector2d rightPos = pos - r * normal;

            Eigen::Vector3d mpState = this->mMotionProfile.calculate(lastTime);

            trail::Vector13d vec;
            vec(0, 0) = lastTime; // t
            vec(1, 0) = DT; // dt
            vec(2, 0) = mpState(0, 0); // s(t)
            vec(3, 0) = mpState(1, 0); // v(t)
            vec(4, 0) = mpState(2, 0); // a(t)
            vec(5, 0) = 90 - degrees(theta); // θ(t)
            vec(6, 0) = omega; // ω(t)
            vec(7, 0) = pos(0, 0); // x(t)
            vec(8, 0) = pos(1, 0); // y(t)
            vec(9, 0) = leftPos(0, 0); // x_l(t)
            vec(10, 0) = leftPos(1, 0); // y_l(t)
            vec(11, 0) = rightPos(0, 0); // x_r(t)
            vec(12, 0) = rightPos(1, 0); // y_r(t)

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
