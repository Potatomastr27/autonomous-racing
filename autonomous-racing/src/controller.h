#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "pfms_types.h"
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <cmath>
#include <thread>

using namespace pfms::geometry_msgs;
using namespace std::placeholders;

/**
 * @brief Controls a simulated car in RViz through a pfmsConnector object. Does nothing by itself, this Class is intended to be the parent of multiple different types of cars
*/
class Controller : public rclcpp::Node 
{
  public:
    /**
     * @brief Default Constructor, initilises ros connectors and members
     */
    Controller();

    /**
     * @brief Destroys the pfmsConnector Object to shut down the connection to the RViz simulation
     */
    ~Controller();

    /**
     @brief Returns platform status (indicating if it is executing a series of goals or idle - waiting for goals)
    @return platform status
    */
    pfms::PlatformStatus status(void);

    /**
     Checks whether the platform can travel between origin and destination
    @param[in] origin The origin pose, specified as odometry for the platform
    @param[in] destination The destination point for the platform
    @param[out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
    @param[out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
    @param[out] estimatedGoalPose The estimated goal pose when reaching goal
    @return bool indicating the platform can reach the destination from origin supplied
    */
    virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                            pfms::geometry_msgs::Point goal,
                                            double& distance,
                                            double& time,
                                            pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;

    /**
     @brief Getter for pltform type
    @return PlatformType
    */
    pfms::PlatformType getPlatformType(void);

    /**
     @brief Getter for distance to be travelled to reach goal, updates at the platform moves to current goal
    @return distance to be travlled to goal [m] OR -1 if no goal is set
    */
    double distanceToGoal(void);

    /**
     @brief Getter for time to reach goal, updates at the platform moves to current goal
    @return time to travel to goal [s] OR -1 if no goal is set
    */
    double timeToGoal(void);

    /**
     @brief Set tolerance when reaching goal
    @return tolerance accepted [m]
    */
    bool setTolerance(double tolerance);

    /**
     @brief returns total distance travelled by platform
    @return total distance travelled since started [m]
    */
    double distanceTravelled(void);

    /**
     @brief returns total time in motion by platform, time when stationary not included
    @return total time in motion since started [s]
    */
    double timeTravelled(void);

    /**
     @brief returns current odometry information
    @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
    */
    pfms::nav_msgs::Odometry getOdometry(void);

    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> odo);

protected:
    /**
     * @brief Threaded function, will run continuously until deconstructor is called, runs the control logic for the car and sends commands
     */
    virtual void reachGoals(void) = 0;


    /**
     * @brief sets max speed/throttle
     * 
     * @param speed The speed to set
     */
    void setMaxSpeed(double speed);  

    /**
     * @brief Subscription callback for the next goal to drive to, will overwrite the current goal
     * @param goal The goal to drive to
    */
    void setGoal(const std::shared_ptr<geometry_msgs::msg::Point> goal);

    /**
     * @brief Subscription callback to turn the controller on/off
     * @param onoff Bool, whether to turn the system off or on
    */
    void onoffCallback(const std::shared_ptr<std_msgs::msg::Bool> onoff);

    /**
     * @brief Subscription callback to permenantly stop the car, used in emergencies
     * @param msg An empty message
    */
    void estopCallback(const std::shared_ptr<std_msgs::msg::Empty> msg);
  

private:

protected:
    
    pfms::nav_msgs::Odometry odometer_; // Current odometry reading

    pfms::geometry_msgs::Point goal_; // The current goal we are heading towards
    
    
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_onoffSwitch_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_estop_;
    
    

    // Threading safe control
    
    std::thread* driver_;
    std::mutex odomutex_;
    std::mutex goalMtx_;
    std::atomic<pfms::PlatformStatus> status_;
    unsigned int sequence_; // Keep track of what command sequence we are up to
    std::atomic<pfms::PlatformType> platform_; // Stearing type of car
    std::atomic<double> distanceTraveled_; // Distance the car has traveled (m)
    std::atomic<double> timeTraveled_; // Time the car has spent traveling (s)  
    std::atomic<double> distanceToGoal_; // Estimated distance to travel to reach goal (m)
    std::atomic<double> timeToGoal_; // Estimated time to travel to reach goal (s)
    std::atomic<double> tolerance_; // How close the car has to be to the goal (m)  
    std::atomic<double> lxy_speed_; // The max speed/throttle
    std::atomic<bool> runThread_;
    std::atomic<bool> hasGoal_;
    std::atomic<bool> readyToDrive_;
  

};

#endif // CONTROLLER_H
