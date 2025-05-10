#include "display.h"

Display::Display():
    Node("display")
{
    sub_detectedCones_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "orange/cones", 10, std::bind(&Display::detectedConesCallback, this, _1)
    );
    sub_roadCentres_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "orange/road_centres", 10, std::bind(&Display::roadCentresCallback, this, _1)
    );
    
    sub_blacklist_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "orange/blacklisted_areas", 10, std::bind(&Display::blacklistCallback, this, _1)
    );

    sub_splineGoals_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "orange/splineGoals", 10, std::bind(&Display::splineGoalsCallback, this, _1)
    );

    pub_visualMarker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);

    // Publish visualisation message at the specified rate
    timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&Display::visualPublisher, this));
}

void Display::detectedConesCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> detectedCones){
    {
        std::unique_lock<std::mutex> lck(conesMtx_);
        detectedCones_ = detectedCones->poses;
    }
}

void Display::roadCentresCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> roadCentres){
    {
        std::unique_lock<std::mutex> lck(roadCentresMtx_);
        roadCentres_ = roadCentres->poses;
    }
}

void Display::blacklistCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> blacklist){
    {
        std::unique_lock<std::mutex> lck(blacklistMtx_);
        blacklist_ = blacklist->poses;
    }
}

void Display::splineGoalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> splineGoals){
    {
        std::unique_lock<std::mutex> lck(splineGoalsMtx_);
        splineGoals_ = splineGoals->poses;
    }
}


void Display::visualPublisher(){
    // Move Member variables into a new vector so we dont have to hog the mutex's
    std::vector<geometry_msgs::msg::Pose> cones;
    std::vector<geometry_msgs::msg::Pose> centres;
    std::vector<geometry_msgs::msg::Pose> blacklists;
    {
        std::unique_lock<std::mutex> lck1(roadCentresMtx_);
        std::unique_lock<std::mutex> lck2(conesMtx_);
        std::unique_lock<std::mutex> lck3(blacklistMtx_);
        cones = detectedCones_;
        centres = roadCentres_;   
        blacklists = blacklist_;     
    }

    visualization_msgs::msg::MarkerArray msg;
    int id = 0;
    for (auto cone : cones){
        
        msg.markers.push_back(visualization_msgs::msg::Marker {});
        msg.markers.back().header.frame_id = "world";
        msg.markers.back().header.stamp = this->get_clock()->now();
        msg.markers.back().ns = "cones";
        // Unique id
        msg.markers.back().id = id;
        // Set type to cylinder
        msg.markers.back().type = msg.markers.back().CYLINDER;
        // Set action to add
        msg.markers.back().action = msg.markers.back().ADD;
        // The pose is the pose of the cone
        msg.markers.back().pose = cone;
        msg.markers.back().pose.position.z += 0.25; // Otherwise the cyllinder will clip into the ground
        // Set Radius to 0.2m and height to 0.5m
        msg.markers.back().scale.x = 0.4; 
        msg.markers.back().scale.y = 0.4;
        msg.markers.back().scale.z = 0.5;
        // Set Colour to orange
        msg.markers.back().color.a = 0.5; // Transperancy
        msg.markers.back().color.g = 0.5;
        msg.markers.back().color.r = 1.0;
        msg.markers.back().color.b = 0.0;
        msg.markers.back().frame_locked = true;

        id++;
    }

    for (auto centre : centres){
        msg.markers.push_back(visualization_msgs::msg::Marker {});
        msg.markers.back().header.frame_id = "world";
        msg.markers.back().header.stamp = this->get_clock()->now();
        msg.markers.back().ns = "road";
        // Unique id
        msg.markers.back().id = id;
        // Set type to cylinder
        msg.markers.back().type = msg.markers.back().CUBE;
        // Set action to add
        msg.markers.back().action = msg.markers.back().ADD;
        // The pose is the pose of the road centre
        msg.markers.back().pose = centre;
        // Set side lengths to 0.5m
        msg.markers.back().scale.x = 0.5; 
        msg.markers.back().scale.y = 0.5;
        msg.markers.back().scale.z = 0.5;
        // Set Colour to red
        msg.markers.back().color.a = 1.0;
        msg.markers.back().color.g = 0.0;
        msg.markers.back().color.r = 1.0;
        msg.markers.back().color.b = 0.0;

        id++;
    }

    for (auto blklistPoint : blacklists){
        
        msg.markers.push_back(visualization_msgs::msg::Marker {});
        msg.markers.back().header.frame_id = "world";
        msg.markers.back().header.stamp = this->get_clock()->now();
        msg.markers.back().ns = "blacklists";
        // Unique id
        msg.markers.back().id = id;
        // Set type to cylinder
        msg.markers.back().type = msg.markers.back().CYLINDER;
        // Set action to add
        msg.markers.back().action = msg.markers.back().ADD;
        // The pose is the pose of the cone
        msg.markers.back().pose = blklistPoint;
        msg.markers.back().pose.position.z += 0.25; // Otherwise the cyllinder will clip into the ground
        // Set Radius to 0.3m and height to 0.5m
        msg.markers.back().scale.x = 0.6; 
        msg.markers.back().scale.y = 0.6;
        msg.markers.back().scale.z = 0.5;
        // Set Colour to black
        msg.markers.back().color.a = 1.0; // Transperancy
        msg.markers.back().color.g = 0;
        msg.markers.back().color.r = 0;
        msg.markers.back().color.b = 1.0;
        msg.markers.back().frame_locked = true;

        id++;
    }

    /* for (auto splineGoal : splineGoals_){
        msg.markers.push_back(visualization_msgs::msg::Marker {});
        msg.markers.back().header.frame_id = "world";
        msg.markers.back().header.stamp = this->get_clock()->now();
        msg.markers.back().ns = "splineGoals";
        // Unique id
        msg.markers.back().id = id;
        // Set type to cylinder
        msg.markers.back().type = msg.markers.back().CUBE;
        // Set action to add
        msg.markers.back().action = msg.markers.back().ADD;
        // The pose is the pose of the road centre
        msg.markers.back().pose = splineGoal;
        // Set side lengths to 0.5m
        msg.markers.back().scale.x = 0.5; 
        msg.markers.back().scale.y = 0.5;
        msg.markers.back().scale.z = 0.5;
        // Set Colour to red
        msg.markers.back().color.a = 1.0;
        msg.markers.back().color.g = 1.0;
        msg.markers.back().color.r = 0.0;
        msg.markers.back().color.b = 0.0;

        id++;
    } */

    pub_visualMarker_->publish(msg);  

}