#ifndef TRAIL_WAYPOINT_H
#define TRAIL_WAYPOINT_H

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#define ORIGIN Waypoint(0.0, 0.0, 0.0)

using json = nlohmann::json;

namespace trail {
    class Waypoint {
    private:
        double mX;
        double mY;
        double mHeading;
        double mTheta;

    public:
        /**
         * Creates a new Waypoint instance.
         * @param x The x coordinate of the robot's position
         * @param y The y coordinate of the robot's position
         * @param theta The robot's angle, in degrees
         */
        Waypoint(double x, double y, double theta);
        Waypoint() = default;
        ~Waypoint() = default;

        /**
         * @return The x coordinate of the selected Waypoint
         */
        double getX() const;

        /**
         * @return The y coordinate of the selected Waypoint
         */
        double getY() const;

        /**
         * @return The heading angle of the selected Waypoint
         */
        double getHeading() const;

        /**
         * @return The waypoint's coordinates in R^2 as an Eigen vector
         */
        Eigen::Vector2d toPoint() const;

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
        double distanceToPoint(trail::Waypoint otherPoint);
    };

    /**
     * Parses a Waypoint from a JSON object
     * @param json A pointer to the JSON object you want to parse from
     * @param wp The Waypoint to serialize into
     */
    void from_json(const json &json, trail::Waypoint &wp);

    /**
     * Serializes a Waypoint into a JSON object
     * @param json A pointer to the JSON object you want to serialize into
     * @param wp The Waypoint you want to serialize
     */
    void to_json(json &json, const trail::Waypoint &wp);
}

#endif //TRAIL_WAYPOINT_H
