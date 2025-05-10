#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "../src/pathFinder.h"
#include "../src/laserProcessing.h"

#include <iostream>

using namespace geometry_msgs::msg;
using std::vector;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::LaserScan;

TEST(AdditionalGoalGeneration, StartPosition){

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
    
    // Use laser processor to find cones
    vector<Point> cones = lsr.findCones(); // Find cones with default parameters
    // create pathfinder with lookahead of 2 so it finds all possible goals
    PathFinder pathfinder = PathFinder(true, 2, 6.5, 12.0, 15.0, true, 3.5, 0.05); 

    // Setup the pathfinder with the required info
    pathfinder.addData(cones, *odo_msg);

    // Get the goals
    vector<Point> goals = pathfinder.getGoals();

    // Expected goals, manually calculated from sim
    vector<Point> correctGoals;
    correctGoals.push_back(geometry_msgs::build<Point>().x(38.37).y(13.33).z(0));
    correctGoals.push_back(geometry_msgs::build<Point>().x(49.77).y(12.06).z(0));

    // Run tests

    ASSERT_EQ(goals.size(), correctGoals.size());

    ASSERT_NEAR(goals[0].x, correctGoals[0].x, 0.1);
    ASSERT_NEAR(goals[0].y, correctGoals[0].y, 0.1);
    ASSERT_NEAR(goals[1].x, correctGoals[1].x, 0.1);
    ASSERT_NEAR(goals[1].y, correctGoals[1].y, 0.1);


}

TEST(AdditionalGoalGeneration, TightTurn){

    // Find package install share directory
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous-racing");
    // Bag directory (should be saved in /data/)
    std::string bag_filename=package_share_directory + "/data/tightTurn";

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
    
    // Use laser processor to find cones
    vector<Point> cones = lsr.findCones(); // Find cones with default parameters
    // create pathfinder with a look ahead of 4 so it finds all possible goals
    PathFinder pathfinder = PathFinder(true, 4, 6.5, 12.0, 15.0, true, 3.5, 0.05); 

    // Setup the pathfinder with the required info
    pathfinder.addData(cones, *odo_msg);

    // Get the goals
    vector<Point> goals = pathfinder.getGoals();

    // Expected goals, manually calculated from sim
    vector<Point> correctGoals;
    correctGoals.push_back(geometry_msgs::build<Point>().x(74.18).y(-56.06).z(0));
    correctGoals.push_back(geometry_msgs::build<Point>().x(70.475).y(-61.625).z(0));
    correctGoals.push_back(geometry_msgs::build<Point>().x(60.19).y(-61.925).z(0));
    correctGoals.push_back(geometry_msgs::build<Point>().x(51.72).y(-54.345).z(0));

    // Run tests

    ASSERT_EQ(goals.size(), correctGoals.size());

    ASSERT_NEAR(goals[0].x, correctGoals[0].x, 0.1);
    ASSERT_NEAR(goals[0].y, correctGoals[0].y, 0.1);
    ASSERT_NEAR(goals[1].x, correctGoals[1].x, 0.1);
    ASSERT_NEAR(goals[1].y, correctGoals[1].y, 0.1);
    ASSERT_NEAR(goals[2].x, correctGoals[2].x, 0.1);
    ASSERT_NEAR(goals[2].y, correctGoals[2].y, 0.1);
    ASSERT_NEAR(goals[3].x, correctGoals[3].x, 0.1);
    ASSERT_NEAR(goals[3].y, correctGoals[3].y, 0.1);


}