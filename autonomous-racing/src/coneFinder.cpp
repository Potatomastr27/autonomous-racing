#include "coneFinder.h"

ConeFinder::ConeFinder():
    Node("cone_finder"),
    detectedCones_({}),
    latestScan_({}),
    odoAtScan_({}),
    latestOdo_({}),
    lsrProcess_(LaserProcessing(LaserScan {}, Pose {}, 0, 0, 0))
    

{
    sub_laserScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "orange/laserscan", 10, std::bind(&ConeFinder::laserCallback,this,_1));
    sub_audiOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "orange/odom", 10, std::bind(&ConeFinder::odomCallback, this, _1));
    
    pub_detectedCones_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/cones", 10);
    pub_visibleCones_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/visible_cones", 10);
    pub_uniqueCone_ = this->create_publisher<geometry_msgs::msg::Pose>("orange/unique_cone", 10);
    pub_blacklist_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/blacklisted_areas", 10);
    pub_eStopCar_ = this->create_publisher<std_msgs::msg::Empty>("orange/e_stop",10);

    // Occasionally check if we need to publish a new goal to the controller
    timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&ConeFinder::publishCones, this));

    timer_estop_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ConeFinder::eStopChecker, this));

    this->declare_parameter("_coneDiam", 0.4);
    this->declare_parameter("_odoScanDist", 3.7);
    this->declare_parameter("_conePointDistMin", 0.3);
    this->declare_parameter("_dupeConeDist", 0.5);
    this->declare_parameter("_minConePoints", 2);
    this->declare_parameter("_estopCheckHeight", 2.5);
    this->declare_parameter("_estopCheckWidth", 2.2);
    
    dbl_coneDiam_ = this->get_parameter("_coneDiam");
    dbl_odoToScanDist_ = this->get_parameter("_odoScanDist");
    dbl_conePointDistMin_ = this->get_parameter("_conePointDistMin");
    dbl_dupeConeDist_ = this->get_parameter("_dupeConeDist");
    int_minRanges_ = this->get_parameter("_minConePoints");
    dbl_estopCheckWidth_ = this->get_parameter("_estopCheckHeight");
    dbl_estopCheckHeight_ = this->get_parameter("_estopCheckWidth");

    // Create LaserProcessing member with given parameters
    lsrProcess_ = LaserProcessing(LaserScan {}, Pose {}, dbl_coneDiam_.as_double(), dbl_conePointDistMin_.as_double(), int_minRanges_.as_int());

    RCLCPP_INFO_STREAM(this->get_logger(), "Params: ");
    RCLCPP_INFO_STREAM(this->get_logger(), dbl_coneDiam_);
    RCLCPP_INFO_STREAM(this->get_logger(), dbl_odoToScanDist_);
    RCLCPP_INFO_STREAM(this->get_logger(), dbl_conePointDistMin_);
    RCLCPP_INFO_STREAM(this->get_logger(), dbl_dupeConeDist_);
    RCLCPP_INFO_STREAM(this->get_logger(), int_minRanges_);
    RCLCPP_INFO_STREAM(this->get_logger(), dbl_estopCheckWidth_);
    RCLCPP_INFO_STREAM(this->get_logger(), dbl_estopCheckHeight_);

    // Sleep for 500ms to give other nodes time to start fully
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void ConeFinder::laserCallback(std::shared_ptr<sensor_msgs::msg::LaserScan> lsr){
    // Lock Odo and Scan mutex's as we access both
    {
        std::unique_lock<std::mutex> lck1(scanMtx_);
        std::unique_lock<std::mutex> lck2(odoMtx_);

        // The scan results mean nothing without the odometry of the car from the same timestamp
        odoAtScan_ = latestOdo_;
        latestScan_ = *lsr;
    }
    
}


void ConeFinder::odomCallback(std::shared_ptr<nav_msgs::msg::Odometry> odo){
    // Lock odo mutex
    {
        std::unique_lock<std::mutex> lck(odoMtx_);
        latestOdo_ = *odo;
    }    

}

void ConeFinder::addUniqueCones(vector<Point> newCones){
    for (auto cone : newCones){
        bool coneIsDupe;
        // Check if the cone is unique
        for (auto detectedCone : detectedCones_.poses){
            // Expression will return whether or not the two cones 0.3m apart on either axis's
            coneIsDupe = (fabs(cone.x - detectedCone.position.x) < dbl_dupeConeDist_.as_double() && 
                fabs(cone.y - detectedCone.position.y) < dbl_dupeConeDist_.as_double());
            // Break out if cone is a dupe to save processing time
            if (coneIsDupe)
                break;
        }
        // If the cone is unique add it to the list of unique cones
        if (coneIsDupe == false){
            Pose uniqueCone;
            uniqueCone.position = cone;
            detectedCones_.poses.push_back(uniqueCone);
            pub_uniqueCone_->publish(uniqueCone);
        }
    }
}

void ConeFinder::publishCones(){
    if(remakeLsrProcess() == false)
        return;
    // Use Library to find the cones with the provided laser scan and odometry
    vector<Point> foundCones = lsrProcess_.findCones();

    // Add cones to detectedCones_ and publish each unique cone
    addUniqueCones(foundCones);

    // Publish the total list of cones detected
    pub_detectedCones_->publish(detectedCones_);

    // Publish all currently visible cones
    PoseArray msg;
    for (auto point : foundCones){
        // Convert each point into a pose and add it to the message
        msg.poses.push_back(geometry_msgs::build<Pose>().position(point).orientation(Quaternion {}));
    }
    pub_visibleCones_->publish(msg);

    // Publish current blacklist
    PoseArray blacklistmsg;
    vector<Point> blacklist = lsrProcess_.getBlacklist();
    for (auto point : blacklist){
        blacklistmsg.poses.push_back(geometry_msgs::build<Pose>().position(point).orientation(Quaternion {}));
    }
    pub_blacklist_->publish(blacklistmsg);
}

void ConeFinder::eStopChecker(){
    if(remakeLsrProcess() == false)
        return;

    // Check to make sure there are no obstacles we are about to crash into
    if (lsrProcess_.isObsInWay(dbl_estopCheckWidth_.as_double(), dbl_estopCheckHeight_.as_double()) == false)
        return;

    // Send message to stop car
    pub_eStopCar_->publish(std_msgs::msg::Empty {});
}



bool ConeFinder::remakeLsrProcess(){
    sensor_msgs::msg::LaserScan lsrScan;
    nav_msgs::msg::Odometry odo;
    {
        std::unique_lock<std::mutex> lck(scanMtx_);
        lsrScan = latestScan_;
        odo = odoAtScan_;
    }
    
    // Make sure the reading came from the world frame, if its not, then it did not come from the sim so we exit the function
    // If I dont do this the system could run with an odometry of 0 meaning it gets the car position wrong
    if (odo.header.frame_id.size() != 5)
        return false;
    
    // Make sure the stamps are within ~5ms of eachother, if not just return
    if (fabs((int64_t)odo.header.stamp.nanosec - (int64_t)odo.header.stamp.nanosec) > 5e6) 
        return false;
    
    // Laserscaner is ~3.7m in front of odo reading so change the odo accordingly

    double angle = tf2::getYaw(odo.pose.pose.orientation);

    odo.pose.pose.position.x += dbl_odoToScanDist_.as_double() * cos (angle);
    odo.pose.pose.position.y += dbl_odoToScanDist_.as_double() * sin (angle);

    // return library object
    lsrProcess_.updateScan(lsrScan, odo.pose.pose);
    return true;
}