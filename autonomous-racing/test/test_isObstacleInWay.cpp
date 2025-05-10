#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "../src/laserProcessing.h"

#include <iostream>

TEST(IsObstacleInWay, StartPosition){

    // Find package install share directory
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous-racing");
    // Bag directory (should be saved in /data/)
    std::string bag_filename=package_share_directory + "/data/startPos";

    // Create a reader object to read from the ros bag
    rosbag2_cpp::Reader reader; 

    // Add serialisations for each topic
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> lsrSerial;
    rclcpp::Serialization<nav_msgs::msg::Odometry> odoSerial;

    // Msg Objects
    sensor_msgs::msg::LaserScan::SharedPtr lsr_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    nav_msgs::msg::Odometry::SharedPtr odo_msg = std::make_shared<nav_msgs::msg::Odometry>();

    // Open the bag file
    reader.open(bag_filename);

    bool haslsr = false;
    bool hasodo = false;
    // Read msgs in bag
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        // Get lsr msg
        if (haslsr == false && msg->topic_name == "/orange/laserscan") {
            haslsr = true;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            lsrSerial.deserialize_message(&serialized_msg, lsr_msg.get());
        }

        // Get odo msg
        if (hasodo == false && msg->topic_name == "/orange/odom"){
            hasodo = true;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            odoSerial.deserialize_message(&serialized_msg, odo_msg.get());
        }

        // Break out if we have all msgs
        if (hasodo == true && haslsr == true)
            break;
        
        
    }
    
    Pose lsrPose = odo_msg->pose.pose;
    // Calculate position of laser scanner from odo and known dist 3.7m
    double angle = tf2::getYaw(lsrPose.orientation);

    lsrPose.position.x += 3.7 * cos (angle);
    lsrPose.position.y += 3.7 * sin (angle);
    
    LaserProcessing lsr(*lsr_msg, lsrPose, 0.4, 0.3, 2);
    
    ASSERT_FALSE(lsr.isObsInWay(2.4,5));

}

TEST(IsObstacleInWay, BehindFireTruck){

    // Find package install share directory
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous-racing");
    // Bag directory (should be saved in /data/)
    std::string bag_filename=package_share_directory + "/data/behindFireTruck";

    // Create a reader object to read from the ros bag
    rosbag2_cpp::Reader reader; 

    // Add serialisations for each topic
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> lsrSerial;
    rclcpp::Serialization<nav_msgs::msg::Odometry> odoSerial;

    // Msg Objects
    sensor_msgs::msg::LaserScan::SharedPtr lsr_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    nav_msgs::msg::Odometry::SharedPtr odo_msg = std::make_shared<nav_msgs::msg::Odometry>();

    // Open the bag file
    reader.open(bag_filename);

    bool haslsr = false;
    bool hasodo = false;
    // Read msgs in bag
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        // Get lsr msg
        if (haslsr == false && msg->topic_name == "/orange/laserscan") {
            haslsr = true;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            lsrSerial.deserialize_message(&serialized_msg, lsr_msg.get());
        }

        // Get odo msg
        if (hasodo == false && msg->topic_name == "/orange/odom"){
            hasodo = true;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            odoSerial.deserialize_message(&serialized_msg, odo_msg.get());
        }

        // Break out if we have all msgs
        if (hasodo == true && haslsr == true)
            break;
        
        
    }
    
    Pose lsrPose = odo_msg->pose.pose;
    // Calculate position of laser scanner from odo and known dist 3.7m
    double angle = tf2::getYaw(lsrPose.orientation);

    lsrPose.position.x += 3.7 * cos (angle);
    lsrPose.position.y += 3.7 * sin (angle);
    
    LaserProcessing lsr(*lsr_msg, lsrPose, 0.4, 0.3, 2);
    
    ASSERT_TRUE(lsr.isObsInWay(2.4,5));

}