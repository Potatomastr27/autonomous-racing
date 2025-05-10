#include "mission.h"

Mission::Mission():
    Node("mission"),
    visibleCones_(0),
    goals_(std::vector<geometry_msgs::msg::Point> {}),
    goal_({}),
    hasGoal_(false),
    running_(false),
    pathfinder_(PathFinder(true, true, 2, 6.5, 12.0, 15.0, true, 3.5, 0.05)),
    splineGoals_(vector<Pose> {})
    
{

    sub_audiOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "orange/odom", 10, std::bind(&Mission::odomCallback, this, _1));
    sub_goals_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "orange/goals", 10, std::bind(&Mission::addGoals, this, _1));

    sub_visibleCones_ = this->create_subscription<PoseArray>(
        "orange/cones", 10, std::bind(&Mission::allConesCallback, this, _1));
    sub_visibleCones_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "orange/visible_cones", 10, std::bind(&Mission::visibleConesCallback, this, _1));
    sub_uniqueCone_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "orange/unique_cone", 10, std::bind(&Mission::uniqueConeCallback, this, _1));

    // Setting up Pulishers
    pub_goal_ = this->create_publisher<geometry_msgs::msg::Point>("orange/goal", 10);
    pub_onoffSwitch_ = this->create_publisher<std_msgs::msg::Bool>("orange/drive", 10);
    pub_roadCentres_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/road_centres", 10);
    pub_splineGoals_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/splineGoals", 10);

    srv_toggleRun_ = this->create_service<std_srvs::srv::SetBool>("orange/mission", std::bind(&Mission::toggleRunCallback, this, _1, _2));

    // Declare parameters and set their default values
    this->declare_parameter("_advanced", true);
    bool_advanced_ = this->get_parameter("_advanced");
    this->declare_parameter("_repeat", true);
    bool_repeat_ = this->get_parameter("_repeat");
    this->declare_parameter("_lookAhead", 2);
    int_lookAhead_ = this->get_parameter("_lookAhead");

    this->declare_parameter("_coneDistLow", 6.5);
    dbl_conePairDistLow_ = this->get_parameter("_coneDistLow");
    this->declare_parameter("_coneDistHigh", 12.0);
    dbl_conePairDistHigh_ = this->get_parameter("_coneDistHigh");
    this->declare_parameter("_maxGoalDist", 15.0);
    dbl_maxGoalDist_ = this->get_parameter("_maxGoalDist");

    this->declare_parameter("_smoothControl", true);
    bool_splineEnable_ = this->get_parameter("_smoothControl");
    this->declare_parameter("_pursuitDistance", 3.5);
    dbl_pursuitDistance_ = this->get_parameter("_pursuitDistance");
    this->declare_parameter("_splineResolution", 0.05);
    dbl_splineResolution_ = this->get_parameter("_splineResolution");

    pathfinder_ = PathFinder(
        bool_advanced_.as_bool(), bool_repeat_.as_bool(), int_lookAhead_.as_int(),
        dbl_conePairDistLow_.as_double(), dbl_conePairDistHigh_.as_double(), dbl_maxGoalDist_.as_double(),
        bool_splineEnable_.as_bool(), dbl_pursuitDistance_.as_double(), dbl_splineResolution_.as_double()        
        );

    // Occasionally check if we need to publish a new goal to the controller
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Mission::goalPublisher, this));
}

void Mission::addGoals(std::shared_ptr<geometry_msgs::msg::PoseArray> goals){
    // Lock for goals vector
    std::unique_lock<std::mutex> lck(goalsMtx_);


    // Add goals into end of private vector converting from a Pose to a Point (We dont need the orientation)
    for (auto goal : goals->poses){
        goals_.push_back(geometry_msgs::msg::Point {});
        goals_.back().x = goal.position.x;
        goals_.back().y = goal.position.y;
        goals_.back().z = goal.position.z;
    }

    lck.unlock();
}

void Mission::odomCallback(std::shared_ptr<nav_msgs::msg::Odometry> odo){
    {
        std::unique_lock<std::mutex> lck(odoMtx_);
        latestOdo_ = *odo;
    }
    
}

void Mission::allConesCallback(std::shared_ptr<PoseArray> all_cones){
    {
        std::unique_lock<std::mutex> lck(allConesMtx_);
        allCones_ = all_cones->poses;
    }
}

void Mission::visibleConesCallback(std::shared_ptr<PoseArray> visible_cones){
    {
        std::unique_lock<std::mutex> lck(visibleConesMtx_);
        visibleCones_ = visible_cones->poses.size();     
    }
}

void Mission::uniqueConeCallback(std::shared_ptr<geometry_msgs::msg::Pose> unique_cone){
    {
        std::unique_lock<std::mutex> lck(uniqueConesMtx_);
        uniqueCones_.push_back(unique_cone->position);
    }
}

void Mission::toggleRunCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res){
    running_ = req->data;
    std_msgs::msg::Bool msg;
    msg.data = running_;
    pub_onoffSwitch_->publish(msg);

    RCLCPP_INFO_STREAM(this->get_logger(),"Recieved bool, running = " << running_ );
    {
        std::unique_lock<std::mutex> lck(visibleConesMtx_);
        res->success = visibleCones_ > 0;
    }

    // Get the completion percentage, convert it into a string with format "<completion>%" and then set it as the response
    res->message = std::to_string(pathfinder_.getEstimatedCompletion()) + "%";
}

void Mission::goalPublisher(){

    if (running_ == false)
        return;

    // Get odometry safely
    nav_msgs::msg::Odometry odo;
    {
        std::unique_lock<std::mutex> lck(odoMtx_);
        odo = latestOdo_;
    }
    
    // Add new cones to pathfinder
    {
        std::unique_lock<std::mutex> lck(uniqueConesMtx_);
        // Add new cones only if we have new cones to add

        pathfinder_.addData(uniqueCones_, odo);
        uniqueCones_.clear();
    }

    // If we are in basic mode try to add goals
    if (bool_advanced_.as_bool() == false){
        std::unique_lock<std::mutex> lck(goalsMtx_);
        // Add new goals only if we have new goals to add
        if (goals_.size() > 0){
            pathfinder_.addGoals(goals_);
            goals_.clear();
        }

        lck.unlock();            
    }

    // Get the next goal, if there is none return early and idle until started back up
    if(pathfinder_.getNextGoal(goal_) == false){
        RCLCPP_INFO_STREAM(this->get_logger(), "Returning early");
        /* vector<Point> allConesTmp;
        for (auto cone : allCones_){
            allConesTmp.push_back(cone.position);
        }
        pathfinder_.addData(allConesTmp, odo); */
        hasGoal_ = false;
        running_ = false;
        return;
    }

    // We now have a goal so set hasGoal to true
    hasGoal_ = true;
    // Publish the new goal to the controller
    pub_goal_->publish(goal_);

    splineGoals_.push_back(Pose{});
    splineGoals_.back().position = goal_;

    // Publish all road centres from pathfinder
    geometry_msgs::msg::PoseArray roadCentres;
    vector<Point> allGoals = pathfinder_.getGoals();
    for (auto goal : allGoals){
        roadCentres.poses.push_back(Pose {});
        roadCentres.poses.back().position = goal;
    }
    pub_roadCentres_->publish(roadCentres);

    // Publish the spline goals
    geometry_msgs::msg::PoseArray splineGoals;
    splineGoals.poses = splineGoals_;
    pub_splineGoals_->publish(splineGoals);
}