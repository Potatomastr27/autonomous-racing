#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <cmath>
#include <mutex>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * Spline calculations are based on 
 * https://people.computing.clemson.edu/~dhouse/courses/405/notes/splines.pdf
 * https://mathworld.wolfram.com/CubicSpline.html
 * https://mathworld.wolfram.com/TridiagonalMatrix.html
*/


using namespace geometry_msgs::msg;
using nav_msgs::msg::Odometry;
using std::vector;
using std::pair;

namespace pathfinder {
    // Namespace for spline related structs
    namespace Spline {

        // A cubic equation coeficents
        struct Coefficents {
            double a;
            double b;
            double c;
            double d;
        };
    }

    struct trackSection {
        std::pair<Point, Point> cones;
        Point Centre;
    };
}

using namespace pathfinder;

class PathFinder {

public:
    /**
     * @brief Creates pathfinder object with given parameters
     * @param conePairDistLow The low threshold for determining cone pairs
     * @param conePairDistHigh The high threshold for determining cone pairs
     * @param lookAhead The number of goals to look ahead
    */
    PathFinder(
        bool automaticGoals, bool repeat, int lookAhead, /*Generic Params*/
        double conePairDistLow, double conePairDistHigh, double maxGoalDist, /*Cone Pair Params*/        
        bool smoothPath, double pursuitDistance, double resolution /*Spline Params*/);

    /**
     * @param cones Either the goals or cone locations
     * @param odo Current vehicle odometry
     */
    void addData(vector<Point> cones, Odometry odo);

    /**
     * @brief Add new goals to drive to, used for basic mode
     * @param goals The goals to add
    */
    void addGoals(vector<Point> goals);

    /**
     * @brief Getter for entire vector of goals to drive to (Will not filter out goals outside the bounds of the track)
     * @returns The entire list of goals
    */
    vector<Point> getGoals() {return goals_;};

    vector<trackSection> getTrackSections() {return trackSections_;};

    /**
     * @brief Calculates new goal(s) and then returns the next goal in the list, will skip goals that are outside the bounds of the track
     * 
     * @param nextGoal [out] The next goal to go to
     * 
     * @returns [true] if there is a next goal, [false] if not
    */
    bool getNextGoal(Point &nextGoal);

    /**
     * @brief Returns the estimated completion percentage, usually very off, only reaches 100% when it arrives at the final goal
     * @returns An int ranging from 0-100
    */
    int getEstimatedCompletion();

private:

    /**
     * @brief Creates a bounding, axis alligned rectangle out of a polygon, with some tollerance
     * @param cones The points of the polygon to create a bounding rectangle out of
     * @returns a pair of points that make up the bounding rectangle, first is the topleft corner, second is the bottom right corner
    */
    std::pair<Point,Point> createBoundingRectangle(vector<Point> cones);

    /**
     * @brief Returns if the given point is within the bounds of the track given by the cones
     * @param point The point to check
     * @returns [true] if within the bounds, [false] otherwise
    */
    bool isPointWithinBounds(Point point);

    /**
     * @brief Finds the midpoint of a pair of cones
     * @param cones A pair of cones
     * @returns the midpoint
    */
    Point calcConesMidpoint(std::pair<Point,Point> cones);

    double calcDistPoints(std::pair<Point,Point> points);

    double calcSectionAngle(trackSection trackSection);

    double calcSectionAngleDiff(pair<trackSection,trackSection> trackSections);

    /**
     * @brief Finds the next roadcentre starting from the given reference point, usually the previous roadcentre or odometry
     * @param refPoint The refrence point
     * @param roadCentre The Roadcentre it found
     * @returns Whether or not the function found a roadcentre
    */
    bool findNextTrackSection(Point refPoint, trackSection &trackSection);

    void computeTrack();

    /**
     * @brief Calculates the cubic spline coefficents for the given axis, for a spline with the form a + bt + ct^2 + dt^3
     * @brief Combine one for each axis to create a space curve that works in 2 dimensions instead of jsut one
     * @param splineAxisPoints The x/y/z coordinates of each control point in the spline
     * @returns The coefficents for each cubic equation between each control point
    */
    vector<Spline::Coefficents> computeSplineCoefficients(vector<double> splineAxisPoints);

    bool computePursuitPoint(Point &nextGoal, vector<Point> controlPoints); 

private:
    vector<Point> cones_;
    vector<Point> goals_;
    vector<trackSection> trackSections_;
    Odometry odo_;
    Odometry initalOdo_;

    int nextGoalIndex_;
    double distanceTravelled_;
    bool hasOdo_;

    // Input Params
    bool automaticGoals_;
    bool repeat_;
    int lookAhead_;
    
    // Cone Pair Params
    double conePairDistLow_, conePairDistHigh_;
    double maxGoalDist_;

    // Spline Params
    bool splineEnabled_;
    double pursuitDistance_;
    double resolution_;
    

    

    
};













#endif