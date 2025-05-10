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

TEST(IsGoalWithinCones, StartPosition){

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
    
    PathFinder pathfinder = PathFinder(false, 2, 6.5, 12.0, 15.0, false, 3.5, 0.05); // create pathfinder with default parameters

    // Setup the pathfinder with the required info
    pathfinder.addData(cones, *odo_msg);

    vector<Point> givenGoals;
    givenGoals.push_back(geometry_msgs::build<Point>().x(38.37).y(13.33).z(0)); // Within cones
    givenGoals.push_back(geometry_msgs::build<Point>().x(0).y(0).z(0)); // Outside of cones
    givenGoals.push_back(geometry_msgs::build<Point>().x(49.77).y(12.06).z(0)); // Within cones

    // Pathfinder will ignore goals that are not within the cones
    pathfinder.addGoals(givenGoals);

    vector<Point> goals = pathfinder.getGoals();

    Point nextGoal;    

    // Run tests

    // Ensure pathfinder recieved all given goals without modifications
    ASSERT_EQ(goals.size(), givenGoals.size());

    ASSERT_DOUBLE_EQ(goals[0].x, givenGoals[0].x); 
    ASSERT_DOUBLE_EQ(goals[0].y, givenGoals[0].y);
    ASSERT_DOUBLE_EQ(goals[1].x, givenGoals[1].x); 
    ASSERT_DOUBLE_EQ(goals[1].y, givenGoals[1].y);
    ASSERT_DOUBLE_EQ(goals[2].x, givenGoals[2].x); 
    ASSERT_DOUBLE_EQ(goals[2].y, givenGoals[2].y);

    // Ensure pathfinder will skip the second given goal as it is outside the given cones
    ASSERT_TRUE(pathfinder.getNextGoal(nextGoal)); // Should give 1st given goal

    ASSERT_DOUBLE_EQ(nextGoal.x, givenGoals[0].x);
    ASSERT_DOUBLE_EQ(nextGoal.y, givenGoals[0].y);

    ASSERT_TRUE(pathfinder.getNextGoal(nextGoal)); // Should give 3rd given goal (skips second)

    ASSERT_DOUBLE_EQ(nextGoal.x, givenGoals[2].x);
    ASSERT_DOUBLE_EQ(nextGoal.y, givenGoals[2].y);

    ASSERT_FALSE(pathfinder.getNextGoal(nextGoal)); // Should return false

}