#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "std_msgs/msg/float64.hpp"
#include "audi.h"
#include <chrono>
/**
 * @brief Inherits from Controller. Controls an Audi car that uses ackerman steering which is what most cars use
*/
class Ackerman: public Controller
{
public:
    //Default constructor should set all attributes to a default value
    Ackerman();

    

    /**
     Checks whether the platform can travel between origin and destination
    @param[in] origin The origin pose, specified as odometry for the platform
    @param[in] destination The destination point for the platform
    @param[out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
    @param[out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
    @param[out] estimatedGoalPose The estimated goal pose when reaching goal
    @return bool indicating the platform can reach the destination from origin supplied
    */
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                            pfms::geometry_msgs::Point goal,
                                            double& distance,
                                            double& time,
                                            pfms::nav_msgs::Odometry& estimatedGoalPose);

  private:

    /**
     * @brief Threaded reach goal function, will run continuously until deconstructor is called
     */
    void reachGoals(void);

    /**
     * @brief Publishes a command to move the ackerman
     * @param cmd The command to publish
     */
    void sendcmd(pfms::commands::Ackerman cmd);
  

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_brakecmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_steeringcmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_throttlecmd_;

    Audi audi_;

    const int max_braking_torque; // 8000Nm is the max braking torque for the Audi car

    /**
     * @brief Stores a skidsteer's driving state:
     * Driving -> Driving and turning to reach goal,
     * Braking -> Braking to stop on goal,
     * Finished -> We are stopped on goal
     */
    enum ackermanState {Driving, Braking, Finished};
};

#endif // ACKERMAN_H
