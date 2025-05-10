#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <cmath>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace laserprocessing {
    struct Polar{
        double range;
        double angle;
    };
};

using laserprocessing::Polar;

using sensor_msgs::msg::LaserScan;
using namespace geometry_msgs::msg;
using std::vector;

class LaserProcessing 
{
public:
    /**
     * @brief Creates a laserprocessing object with a laser scan and its Pose
     * @param scan A LaserScan msg
     * @param scanPose The Pose of the laser scan
     * @param coneDiameter The diameter of a cone
     * @param sameConeDist The minimum distance for two ranges to be considered as part of the same object
     * @param minRanges The minimum number of successive ranges needed before the system will analyse the data
    */
    LaserProcessing(LaserScan scan, Pose scanPose, double coneDiameter, double sameConeDist, int minRanges);

    /**
     * @brief Updates the laser scan and the pose with new ones
     * @param scan A LaserScan msg
     * @param scanPose The Pose of the laser scan
    */
    void updateScan(LaserScan scan, Pose scanPose);

    /**
     * @brief Analyses the given scan data and returns all the cones it can find in that data according to the parameters
     * 
     * @returns A vector of the currently visible cones
    */
    vector<Point> findCones();

    /**
     * @brief Analyses the given scan data and returns if there is a Laser scan range within the given box in front of the lsrScan
     * @param width The width of the box to check
     * @param height The height of the box to check
     * 
     * @returns true if there is a range inside the box, false otherwise
    */
    bool isObsInWay(double width, double height);

    /**
     * @brief getter for the current blacklistedd points, (points where we know there are no cones within ~0.5m)
     * @returns the vector of blacklisted points
    */
    vector<Point> getBlacklist();
private:
    /**
     * @brief Analyses a vector of polar ranges and decides whether to blacklist, ignore or add as a new cone
     * @param ranges [in] the vector of ranges
     * @param cone [out] The outputted cone
     * @returns true if it found a cone, false otherwise
    */
    bool analyseRanges(vector<Polar> ranges, Point &cone);

    /**
     * @brief Finds the average of a list of ranges and converts it into a global Point
     * 
     * @param ranges [in] The ranges to find the average of, should make up a cone
     * @param refPose [in] The reference Pose these ranges were calculated from
     * @param offset [in] The offset to place the point at
     * 
     * @returns The calculated global point
    */
    Point avgRangesToPoint(vector<Polar> ranges, Pose refPose, double offset);
    /**
     * @brief Add the unique points from the vector into the internal blacklist
    */
    void blacklistRanges(vector<Point> points);

    /**
     * @brief Converts a local polar coordinate to a global point based on the supplied reference pose
     * @param range The polar coordinates to convert
     * @param refPose the reference Pose
     * @returns The converted global point
    */
    Point rangeToPoint(Polar range, Pose refPose);

private:

    LaserScan scan_;
    Pose scanPose_;

    double coneDiameter_;
    double sameConeDist_;
    int minRanges_;

    vector<Point> blacklistedAreas_; // Points the object is sure are not cones


};
















#endif