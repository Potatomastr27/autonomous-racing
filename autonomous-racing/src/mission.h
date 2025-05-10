#ifndef MISSION_H
#define MISSION_H

// Ros includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pathFinder.h"

#include <vector>
#include <algorithm>

using namespace std::placeholders;
using namespace geometry_msgs::msg;

class Mission : public rclcpp::Node
{
public:
    /**
     * @brief Default Constructor, initilises ros connectors and members
    */
    Mission();

private:

    /**
     * @brief Subscription Callback for given goals, only does anything in basic mode
     *
     * @param goals the msg with goals to drive to
     */
    void addGoals(std::shared_ptr<geometry_msgs::msg::PoseArray> goals);
    /**
     * @brief Subscription Callback for odometry
     * @param odo The odometry
    */
    void odomCallback(std::shared_ptr<nav_msgs::msg::Odometry> odo);

    void allConesCallback(std::shared_ptr<PoseArray> all_cones);

    /**
     * @brief Subscription Callback for currently visible cones
     * @param visible_cones The currently visible cones
    */
    void visibleConesCallback(std::shared_ptr<PoseArray> visible_cones);
    /**
     * @brief Subscription Callback for each unique cone, recieved one at a time as they are detected
     * @param unique_cone The unique cone
    */
    void uniqueConeCallback(std::shared_ptr<geometry_msgs::msg::Pose> unique_cone);

    /**
     * @brief Service Callback for turning the system on/off with a bool, responds with completion percentage and whether a cone is currently visible
     * @param req The request data, a bool
     * @param res The response the callback will respond with
    */
    void toggleRunCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);  
    /**
     * @brief Goal publisher, Ocasionally publishes new goal to controller if it is needed
    */
    void goalPublisher();

private:

    //ROS2 related members
    //Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_goals_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_audiOdom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_visibleCones_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_allCones_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_uniqueCone_; // Subscriber that recieves each cone when they are spotted for the first time
    //Publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_goal_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_roadCentres_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_onoffSwitch_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_splineGoals_;
    //Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_toggleRun_;  

    // Parameters:
    rclcpp::Parameter bool_advanced_;
    rclcpp::Parameter bool_repeat_;
    rclcpp::Parameter int_lookAhead_;

    rclcpp::Parameter dbl_conePairDistLow_, dbl_conePairDistHigh_;
    rclcpp::Parameter dbl_maxGoalDist_;

    rclcpp::Parameter bool_splineEnable_;
    rclcpp::Parameter dbl_pursuitDistance_;    
    rclcpp::Parameter dbl_splineResolution_;
    

    // Atomics for callbacks
    std::atomic<bool> running_;

    // Containers and mutex for callbacks
    std::mutex uniqueConesMtx_;
    std::mutex visibleConesMtx_;
    std::mutex allConesMtx_;
    std::mutex goalsMtx_;
    std::mutex odoMtx_;  

    vector<Pose> allCones_;
    std::vector<geometry_msgs::msg::Point> uniqueCones_; // For storing the cones from sub_uniqueCone_, subscription callback will append new msgs on the end of the vector
    int visibleCones_;
    std::vector<geometry_msgs::msg::Point> goals_; //!< A private copy of all goals the mission must reach
    nav_msgs::msg::Odometry latestOdo_; // The latest odometry info

    //Timers for threading
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals

    // Normal Members
    PathFinder pathfinder_;
    std::vector<geometry_msgs::msg::Pose> splineGoals_;
    bool hasGoal_;
    geometry_msgs::msg::Point goal_; // The current goal the platform is heading towards
  
};

#endif // RANGERFUSION_H
