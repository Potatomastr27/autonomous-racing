#include "ackerman.h"



Ackerman::Ackerman():
    max_braking_torque(8000)
{
    
    platform_ = pfms::PlatformType::ACKERMAN;
    driver_ = new std::thread(&Ackerman::reachGoals, this);

    pub_brakecmd_ = this->create_publisher<std_msgs::msg::Float64>("orange/brake_cmd", 10);
    pub_steeringcmd_ = this->create_publisher<std_msgs::msg::Float64>("orange/steering_cmd", 10);
    pub_throttlecmd_ = this->create_publisher<std_msgs::msg::Float64>("orange/throttle_cmd", 10);
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "orange/odom", 10, std::bind(&Controller::odomCallback, this, _1));
    this->declare_parameter("_speed", 0.07);
    lxy_speed_ = this->get_parameter("_speed").as_double();
}

void Ackerman::reachGoals(void){
    pfms::nav_msgs::Odometry prevOdometry = getOdometry();  
    pfms::nav_msgs::Odometry curOdo;
    odometer_ = getOdometry();
    double tempSteering, tempDist, steering, breakingDist, linearDistToGoal;
    
    // Save Initial odometer reading for distance travelled calculation
    pfms::commands::Ackerman command;
    // Thread will loop this function endlessly
    while (true){
        // Thread will only run if it has a goal and it is ready to drive
        while ((hasGoal_ & readyToDrive_) == false)
        {
            // Sleep to reduce cpu usage
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // if deconstructor is called, it will set this variable to false to stop stop the thread
            if (runThread_ == false)
                return;
        }
        status_ = pfms::PlatformStatus::RUNNING;
        
        hasGoal_ = false;
        // Save start time to measure the time spent travelling
        auto prevTime = std::chrono::high_resolution_clock::now();

        // Timeout
        auto startTime = std::chrono::high_resolution_clock::now();

        curOdo = getOdometry();

        {
            std::unique_lock<std::mutex> lck(goalMtx_);
            if (audi_.computeSteering(curOdo, goal_, tempSteering, tempDist) == false){
                continue;
            }   
        }   
        
        steering = tempSteering;

        ackermanState state = Driving; // Start in Driving state

        command.seq = sequence_;

        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count()/1000 < timeToGoal_ * 3){
            command.seq++;
            sequence_ = command.seq;

            // Reset command to 0 values
            command.steering = 0;
            command.throttle = 0;
            command.brake = 0;

            switch (state){
                case Driving:
                    // Switch to braking if within the goal area
                    // Calculate breaking distance as a linear function of speed, constant chosen experimentally
                    breakingDist = tolerance_ + (0.1 * std::hypot(curOdo.linear.x, curOdo.linear.y)) -0.2;
                    linearDistToGoal = std::hypot(goal_.x - curOdo.position.x, goal_.y - curOdo.position.y);

                    if (linearDistToGoal < breakingDist){
                        state = Braking;
                        break;
                    }

                    command.steering = steering; // Use the steering provided by audi library
                    command.throttle = lxy_speed_; // set to 0.1
                    break;
                case Braking: 
                    // If car is stopped switch to finished state
                    if (fabs(curOdo.linear.x) < 0.05 && fabs(curOdo.linear.y) < 0.05){
                        state = Finished;
                        break;
                    }

                    command.brake = max_braking_torque; // brake really hard to stop car on goal
                    break;
                default:
                    break;

            }

            if (state == Finished)
                break;

            if (readyToDrive_ == false){
                state = Braking;
            }

            sendcmd(command);

            // Sum distance travlled (assume small distances are straight)
            distanceTraveled_ = distanceTraveled_ + std::hypot(prevOdometry.position.x - curOdo.position.x, prevOdometry.position.y - curOdo.position.y);
            // Save current odometry
            prevOdometry = curOdo;
            // Get new odometer reading
            curOdo = getOdometry();
            // Recalculate steering angle
            if (audi_.computeSteering(curOdo, goal_, tempSteering, tempDist) == true){
                steering = tempSteering;
            }
            

            // Hold percentage completion at ~95% until fully complete by modifying the estimated distance as we approach the goal
            if (distanceToGoal_*0.95 < distanceTraveled_){
                distanceToGoal_ = distanceTraveled_ * 1.053;
            }

            // Calculate time between last call and this call in microseconds (for precision)
            double deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - prevTime).count();
            prevTime = std::chrono::high_resolution_clock::now();
            // Add delta time to the total time travelled, convert to seconds
            timeTraveled_ = timeTraveled_ + deltaTime/1000000; 

            // Sleep so we dont overload the cpu and pfmsConnector, give other threads a chance to send commands
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            
        }   

        // Finished/Failed Ending code
        // We have finished so set estimated distance equal to distance travelled
        distanceToGoal_.store(distanceTraveled_.load());
        
        RCLCPP_INFO_STREAM(this->get_logger(),"Stopping at: " << curOdo.position.x << ", " << curOdo.position.y);
        status_ = pfms::PlatformStatus::IDLE;
    }
}

void Ackerman::sendcmd(pfms::commands::Ackerman cmd){
    // Create float64 message, store a double in it and then publish it for each entry of cmd
    std_msgs::msg::Float64 brake;
    brake.data = cmd.brake;
    pub_brakecmd_->publish(brake);
    std_msgs::msg::Float64 throttle;
    throttle.data = cmd.throttle;
    pub_throttlecmd_->publish(throttle);
    std_msgs::msg::Float64 steering;
    steering.data = cmd.steering;
    pub_steeringcmd_->publish(steering);
}



bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){

    bool OK = audi_.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose);

    return OK;
}