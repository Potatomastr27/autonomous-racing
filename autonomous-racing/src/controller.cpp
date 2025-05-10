#include "controller.h"

Controller::Controller():
    Node("controller"),
    odometer_(pfms::nav_msgs::Odometry {0,0,0,0,0,0,0,0}), //0 time, 0,0,0 position, 0 yaw, 0,0,0 velocity
    status_(pfms::PlatformStatus::IDLE),
    sequence_(0),
    distanceTraveled_(0),
    timeTraveled_(0),
    distanceToGoal_(0),
    timeToGoal_(0),
    tolerance_(0.5),    
    runThread_(true),
    hasGoal_(false),
    readyToDrive_(false)
{   
    goal_.x = 0;
    goal_.y = 0;

    sub_goal_ = this->create_subscription<geometry_msgs::msg::Point>("orange/goal",
        10, std::bind(&Controller::setGoal, this, _1));

    sub_onoffSwitch_ = this->create_subscription<std_msgs::msg::Bool>("orange/drive", 
        10, std::bind(&Controller::onoffCallback, this, _1));
    
    sub_estop_ = this->create_subscription<std_msgs::msg::Empty>("orange/e_stop", 
        10, std::bind(&Controller::estopCallback, this, _1));
    this->declare_parameter("_goalTolerance", 0.5);
    tolerance_ = this->get_parameter("_goalTolerance").as_double();
}

Controller::~Controller(){
    // Tell thread to stop, waits until thread is finished, then deconstruct thread
    runThread_ = false;
    driver_->join();
    driver_->~thread();
}

pfms::PlatformStatus Controller::status(void){
    return status_.load();
}

void Controller::setGoal(const std::shared_ptr<geometry_msgs::msg::Point> goal){

    double endDist, endTime;
    pfms::nav_msgs::Odometry endOdo;

    // Check if the car can reach the goal and get the estimated distance, time and end odometry
    if(checkOriginToDestination(getOdometry(), {goal->x, goal->y, 0}, endDist, endTime, endOdo) == false)
        return;
    // Sum total distance over all goals
    distanceToGoal_ = endDist;
    timeToGoal_  = endTime;             

    // Update goal
    {
        std::unique_lock<std::mutex> lck(goalMtx_);
        goal_.x = goal->x;
        goal_.y = goal->y;
    }
    

    hasGoal_ = true;
    RCLCPP_INFO_STREAM(this->get_logger(),"Recieved Goal: " << goal_.x << ", " << goal_.y);
    return;
}

void Controller::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> odo){
    std::unique_lock<std::mutex> lck(odomutex_);
    odometer_.position.x = odo->pose.pose.position.x;
    odometer_.position.y = odo->pose.pose.position.y;
    odometer_.position.z = odo->pose.pose.position.z;

    odometer_.yaw = tf2::getYaw(odo->pose.pose.orientation);

    odometer_.linear.x = odo->twist.twist.linear.x;
    odometer_.linear.y = odo->twist.twist.linear.y;
    odometer_.linear.z = odo->twist.twist.linear.z;
    lck.unlock();
}

void Controller::onoffCallback(const std::shared_ptr<std_msgs::msg::Bool> onoff){
    readyToDrive_ = onoff->data;

    RCLCPP_INFO_STREAM(this->get_logger(),"Currently Driving: " << (readyToDrive_ & hasGoal_));
}

void Controller::estopCallback(const std::shared_ptr<std_msgs::msg::Empty> msg){
    readyToDrive_ = false;

    // Unsubscribe as the system will now brake the car and then ignore all requests to start driving
    sub_onoffSwitch_.reset();
    sub_goal_.reset();
    sub_estop_.reset();
    sub_odom_.reset();

    RCLCPP_INFO_STREAM(this->get_logger(), "E-STOP Called, Platform is will now brake and not respond to any topics/services");
}

void Controller::setMaxSpeed(double speed){
    lxy_speed_ = speed;
}


pfms::PlatformType Controller::getPlatformType(void){
    return platform_.load();
}

double Controller::distanceToGoal(void){
    return distanceToGoal_.load();
}

double Controller::timeToGoal(void){
    return timeToGoal_.load();
}

bool Controller::setTolerance(double tolerance){
    if (tolerance < 0)
        return false;

    tolerance_.store(tolerance);
    return true;
}

double Controller::distanceTravelled(void){
    return distanceTraveled_.load();
}

double Controller::timeTravelled(void){
    return timeTraveled_.load();
}


pfms::nav_msgs::Odometry Controller::getOdometry(void){

    // Create a unique lock using the mutex
    // Unlock by leaving scope
    pfms::nav_msgs::Odometry odo;
    {
        std::unique_lock<std::mutex> lck(odomutex_);
        odo = odometer_;
    }
    
    // Return new odometry
    return odo;
}
