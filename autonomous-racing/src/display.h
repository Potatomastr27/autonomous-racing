#ifndef DISPLAY_H
#define DISPLAY_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <mutex>
#include <thread>

using namespace std::placeholders;

class Display : public rclcpp::Node
{

public:
    /**
     * @brief Default Constructor, initilises ros connectors and members
    */
    Display();


private:
    /**
     * @brief Subscription callback for the total list of detected cones
     * @param detectedCones The vector of cones
    */
    void detectedConesCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> detectedCones);
    /**
     * @brief Subscription callback for the total list of roadCentres
     * @param roadCentres The vector of roadcentres
    */
    void roadCentresCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> roadCentres);
    /**
     * @brief Subscription callback for the total list of blacklisted areas
     * @param blacklist The vector of blacklisted areas
    */
    void blacklistCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> blacklist);
    /**
     * @brief Subscription callback for the total list of spline goals
     * @param splineGoals The vector of spline goals
    */
    void splineGoalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> splineGoals);

    /**
     * @brief Run occasionally by a wall timer
     * @brief Publishes all the information it has subscribed to to the visualizer_markers for gazebo to visualize
    */
    void visualPublisher();

private:

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_detectedCones_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_roadCentres_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_blacklist_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_splineGoals_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_visualMarker_;

    std::mutex conesMtx_;
    std::mutex roadCentresMtx_;
    std::mutex blacklistMtx_;
    std::mutex splineGoalsMtx_;
    std::vector<geometry_msgs::msg::Pose> detectedCones_;
    std::vector<geometry_msgs::msg::Pose> roadCentres_;
    std::vector<geometry_msgs::msg::Pose> blacklist_;
    std::vector<geometry_msgs::msg::Pose> splineGoals_;

    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals

};











#endif