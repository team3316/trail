#ifndef TRAIL_WAYPOINT_H
#define TRAIL_WAYPOINT_H

#include <Eigen/Dense>

class Waypoint {
private:
    double mX;
    double mY;
    double mTheta;

public:
    /**
     * Creates a new Waypoint instance.
     * @param x The x coordinate of the robot's position
     * @param y The y coordinate of the robot's position
     * @param theta The robot's angle, in degrees
     */
    Waypoint(double x, double y, double theta);
    ~Waypoint() = default;

    /**
     * @return The x coordinate of the selected Waypoint
     */
    double getX();

    /**
     * @return The y coordinate of the selected Waypoint
     */
    double getY();

    /**
     * Calculates the required first derivative in order to keep the spline in the desired angle.
     * @param scale A parameter to scale the vector with. Default = 1.75
     * @return A scaled unit vector with the angle given to the constructor
     */
    Eigen::Vector2d firstDerivative(double scale = 1.75);

    /**
     * Calculates the required second derivative in order to keep the spline in the desired angle.
     * @param scale A parameter to scale the vector with. Default = 0.15
     * @return A scaled unit vector with the angle given to the constructor
     */
    Eigen::Vector2d secondDerivative(double scale = 0.15);

    /**
     * Calculates the distance between the current Waypoint and a given Waypoint.
     * @param otherPoint The Waypoint to calculate the distance to
     * @return The distance, calculated using the Pythagorean theorem
     */
    double distanceToPoint(Waypoint otherPoint);
};

#endif //TRAIL_WAYPOINT_H
