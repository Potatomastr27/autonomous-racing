#ifndef CONEFINDER_H
#define CONEFINDER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"

#include <mutex>
#include <thread>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "laserProcessing.h"


using namespace std::placeholders;

using sensor_msgs::msg::LaserScan;
using namespace geometry_msgs::msg;
using std::vector;

class ConeFinder : public rclcpp::Node
{
public:
    /**
     * @brief Default Constructor, initilises ros connectors and members
    */
    ConeFinder();

private:
    /**
     * @brief Callback for laserscan subscription
     * @param lsr The msg
    */
    void laserCallback(std::shared_ptr<sensor_msgs::msg::LaserScan> lsr);

    /**
     * @brief Callback for Odometry subscription
     * @param odo The msg
    */
    void odomCallback(std::shared_ptr<nav_msgs::msg::Odometry> odo);

    /**
     * @brief Adds each unique cone provided in the vector to the internal list of cones
     * 
     * @param newCones The vector of new cones to be added if unique
    */
    void addUniqueCones(vector<Point> newCones);

    /**
     * @brief Publisher, given to wall timer to analyse laser scan and publish the collected info whenever necessary
    */
    void publishCones();
    /**
     * @brief Publisher for estop, given to wall timer to check if we are about to crash into an object and if so will publish a estop msg
    */
    void eStopChecker();
    /**
     * @brief Updates LaserProcessing member with latest data
     * @returns Whether or not the member was updated
    */
    bool remakeLsrProcess();


private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserScan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_audiOdom_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_detectedCones_; // Publishes a vector of all the cones the system has ever detected
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_visibleCones_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_uniqueCone_; // Publishes a cone when they are detected for the first time
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_blacklist_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_eStopCar_;

    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals
    rclcpp::TimerBase::SharedPtr timer_estop_;
    // Parameters
    rclcpp::Parameter dbl_coneDiam_; // The diameter of the cones to detect default: 0.4
    rclcpp::Parameter dbl_odoToScanDist_; // The distance between the odometry and scan reading default: 3.7
    rclcpp::Parameter dbl_conePointDistMin_; // The minimum distance for two successive ranges to be considered part of the same cone default: 0.3
    rclcpp::Parameter dbl_dupeConeDist_; // If a new cone is within this distance to another cone they are considered the same cone default: 0.5
    rclcpp::Parameter int_minRanges_; // The minimum number of ranges to determine if a reading is a cone or not default: 2
    rclcpp::Parameter dbl_estopCheckWidth_; // The width of the box in front of the car to check for obstacles in order to estop
    rclcpp::Parameter dbl_estopCheckHeight_; // The height of the box in front of the car to check for obstacles in order to estop
    

    // We publish this every 500ms approx
    geometry_msgs::msg::PoseArray detectedCones_;

    std::mutex scanMtx_;
    sensor_msgs::msg::LaserScan latestScan_;
    nav_msgs::msg::Odometry odoAtScan_;

    std::mutex odoMtx_;
    nav_msgs::msg::Odometry latestOdo_;

    LaserProcessing lsrProcess_;

};


#endif