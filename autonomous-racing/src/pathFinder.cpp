#include "pathFinder.h"

PathFinder::PathFinder(
    bool automaticGoals, bool repeat, int lookAhead, /*Generic Params*/
    double conePairDistLow, double conePairDistHigh, double maxGoalDist, /*Cone Pair Params*/        
    bool smoothPath, double pursuitDistance, double resolution /*Spline Params*/
    ):
    cones_({}),
    goals_({}),
    trackSections_({}),
    odo_({}),
    initalOdo_({}),

    nextGoalIndex_(0),
    distanceTravelled_(0),
    hasOdo_(false),

    automaticGoals_(automaticGoals),
    repeat_(repeat),
    lookAhead_(lookAhead),

    conePairDistLow_(conePairDistLow),
    conePairDistHigh_(conePairDistHigh),
    maxGoalDist_(maxGoalDist),
    
    splineEnabled_(smoothPath),
    pursuitDistance_(pursuitDistance),
    resolution_(resolution)
{

}


void PathFinder::addData(vector<Point> cones, Odometry odo){
    // Save the inital odometry
    if (hasOdo_ == false)
        initalOdo_ = odo;

    odo_ = odo;
    hasOdo_ = true;

    // Add Cones
    for (auto cone : cones){
        cones_.push_back(cone);
    }

    computeTrack();
}

void PathFinder::addGoals(vector<Point> goals){    
    for (auto goal : goals){
        goals_.push_back(goal);
    }
}

bool PathFinder::getNextGoal(Point &nextGoal){

    // If we are on the current target goal, iterate to the next one
    if (hypot(goals_[nextGoalIndex_].x - odo_.pose.pose.position.x, goals_[nextGoalIndex_].y - odo_.pose.pose.position.y) < 0.5){
        nextGoalIndex_++;
    }

    // If the next index is out of bounds, we have run out of goals so return false
    if (nextGoalIndex_ >= goals_.size())
        if (repeat_ == true)
            nextGoalIndex_ = 0;
        else
            return false;

    // If we don't have enough points to make a spline or spline is not enabled, simply return the next goal
    if (((goals_.size() - nextGoalIndex_) < 2) || (splineEnabled_ == false)){
        nextGoal = goals_[nextGoalIndex_];
        return true;
    }

    // Spline Calculations for pursuit control

    // Create Control Points
    vector<Point> controlPoints;
    controlPoints.push_back(odo_.pose.pose.position);
    controlPoints.push_back(goals_[nextGoalIndex_]);
    controlPoints.push_back(goals_[nextGoalIndex_+1]);

    // Compute Pursuit Point
    if (computePursuitPoint(nextGoal, controlPoints) == false)
        nextGoal = goals_[nextGoalIndex_];
    

    return true;
}

int PathFinder::getEstimatedCompletion(){
    if (goals_.size() < 2)
        return 0;
    
    // Calculate straight line distance to final goal
    double distanceToGo = hypot(odo_.pose.pose.position.x - goals_.back().x, odo_.pose.pose.position.y - goals_.back().y);

    // If we are on the final goal with some tollerance, return 100% completion
    if (distanceToGo < 0.5){
        return 100;
    }
    // Return 0% completion if we haven't reached the first goal yet
    if (nextGoalIndex_ == 0)
        return 0;
    
    // calculate the distance travelled
    double distanceTravelled = hypot(initalOdo_.pose.pose.position.x - goals_.front().x, initalOdo_.pose.pose.position.y - goals_.front().y);
    for (int i = 0; i < nextGoalIndex_-1; i++){
        distanceTravelled += hypot(goals_[i].x - goals_[i+1].x, goals_[i].y - goals_[i+1].y);
    }
    distanceToGo = 0;
    for (int i = nextGoalIndex_-1; i < goals_.size()-1; i++){
        distanceToGo += hypot(goals_[i].x - goals_[i+1].x, goals_[i].y - goals_[i+1].y);
    }
    

    // Return the percentage of the distance travelled over the total distance the system must travel to reach the current final goal
    double percent = (distanceTravelled / (distanceToGo + distanceTravelled) * 100);
    return (int)percent;
}

/* Private Functions */

std::pair<Point,Point> PathFinder::createBoundingRectangle(vector<Point> cones){
    // .first is top left (max x and max y), .second is bottom right (min x and min y)
    std::pair<Point,Point> boundingRec;
    // These infinities should be immediately overwritten as any number is higher/lower than them
    boundingRec.first.x = -INFINITY;
    boundingRec.first.y = -INFINITY;
    boundingRec.second.x = INFINITY;
    boundingRec.second.y = INFINITY;
    // Find the min and max values of x and y in the vector
    for (auto cone : cones){
        if (cone.x > boundingRec.first.x)
            boundingRec.first.x = cone.x;
        if (cone.y > boundingRec.first.y)   
            boundingRec.first.y = cone.y;
        if (cone.x < boundingRec.second.x)
            boundingRec.second.x = cone.x;
        if (cone.y < boundingRec.second.y)
            boundingRec.second.y = cone.y;
    }

    // Add a tollerance to the bounding rectangle
    boundingRec.first.x += 0.5;
    boundingRec.first.y += 0.5;
    boundingRec.second.x -= 0.5;
    boundingRec.second.y -= 0.5;
    return boundingRec;
}

bool PathFinder::isPointWithinBounds(Point point){
    // If we don't have any cone paris just return false
    if (trackSections_.size() == 0)
        return false;

    std::pair<Point,Point> boundingRec;

    // If we only have 1 cone pair make a bounding rec with only that
    if (trackSections_.size() < 2){
        boundingRec = createBoundingRectangle({trackSections_[0].cones.first, trackSections_[0].cones.second});

        // Check if point is within the bounding rectangle (.first is always top left corner, .second is bottom right)
        if (point.x < boundingRec.first.x && point.x > boundingRec.second.x && point.y > boundingRec.second.y && point.y < boundingRec.first.y)
            return true;
        return false;
    }
    
    // Rather than running an expensive computation to determine if the goal is exactly within the bounds
    // We find an axis alligned rectangle that achieves the same thing with much less accuracy
    // (Can be false positives if close enough but not quite on between cones)    
    for (int i = 0; i < trackSections_.size()-1;i++){
        // Create bounding rectangle out of cone pairs
        boundingRec = createBoundingRectangle({trackSections_[i].cones.first, trackSections_[i].cones.second, trackSections_[i+1].cones.first, trackSections_[i+1].cones.second});

        // Check if point is within the bounding rectangle (.first is always top left corner, .second is bottom right)
        if (point.x < boundingRec.first.x && point.x > boundingRec.second.x && point.y > boundingRec.second.y && point.y < boundingRec.first.y)
            return true;

    }

    return false;
}

Point PathFinder::calcConesMidpoint(std::pair<Point,Point> cones){
    geometry_msgs::msg::Point roadCentre;
    // Calculate the midpoint of the two points, that is the road centre
    roadCentre.x = (cones.second.x + cones.first.x)/2;
    roadCentre.y = (cones.second.y + cones.first.y)/2;
    roadCentre.z = 0;  

    return roadCentre;
}

double PathFinder::calcDistPoints(std::pair<Point,Point> points){
    return std::hypot(points.first.x - points.second.x, points.first.y - points.second.y);
}

double PathFinder::calcSectionAngle(trackSection trackSection){
    return atan2(
        trackSection.cones.first.y - trackSection.cones.second.y, 
        trackSection.cones.first.x - trackSection.cones.second.x
    );
}

double PathFinder::calcSectionAngleDiff(pair<trackSection,trackSection> trackSections){

    double angle1 = calcSectionAngle(trackSections.first);
    double angle2 = calcSectionAngle(trackSections.second);
    double diff = angle1 - angle2;
    // Normalising angle difference to an absolute angle ranging from 0-90
    if (diff > M_PI){
        diff -= 2 * M_PI;
    }
    else if (diff < -M_PI){
        diff += 2 * M_PI;
    }
    diff = fabs(diff);
    if (diff > M_PI_2)
        diff = M_PI - diff;
    
    return diff;
}

bool PathFinder::findNextTrackSection(Point refPoint, trackSection &trackSection){

    vector<pair<double, uint32_t>> distancesToCones; // Distance to the cone and the index of the cone

    geometry_msgs::msg::Point conePos;
    // Create the distancesToCones vector based on the internal cone vector,
    for (int i = 0; i < cones_.size(); i++){
        conePos = cones_[i];
        // Calculate distance to cone and add it and its index to the vector
        distancesToCones.push_back({calcDistPoints({conePos, refPoint}), i});
    }
    
    // Sort them according to distance, lowest first
    std::stable_sort(distancesToCones.begin(), distancesToCones.end(), 
        [](pair<double, uint32_t> pair1, pair<double, uint32_t> pair2){
            return pair1.first < pair2.first;
        }
    );

    // The closest cone is always the first one, the closest cone should always be used to find a pair
    
    trackSection.cones.first = cones_[distancesToCones[0].second];
    double pairDist = 0, driveDist = 0, angleDiff = 0;
    for (int i = 1; i < distancesToCones.size(); i++){

        trackSection.cones.second = cones_[distancesToCones[i].second];
        // Calc distance between cones
        pairDist = calcDistPoints(trackSection.cones);
        // Calc distance from last track section
        trackSection.Centre = calcConesMidpoint(trackSection.cones);
        if (trackSections_.empty() == false){
            driveDist = calcDistPoints({trackSection.Centre, trackSections_.back().Centre});
            angleDiff = calcSectionAngleDiff({trackSection, trackSections_.back()});
        }    
        
        // If valid we found next track section
        if (pairDist > conePairDistLow_ && pairDist < conePairDistHigh_ && 
            driveDist < maxGoalDist_ /* && angleDiff < (75 * (M_PI/180.0)) */){
            
            // Remove the two cones from the internal cone vector, remove the highest index first so we dont loose track of the index's
            if (distancesToCones[0].second < distancesToCones[i].second){
                cones_.erase(cones_.begin() + distancesToCones[i].second);
                cones_.erase(cones_.begin() + distancesToCones[0].second);
            }
            else {
                cones_.erase(cones_.begin() + distancesToCones[0].second);
                cones_.erase(cones_.begin() + distancesToCones[i].second);
            }
            // Return true as we have found the next track section
            return true;
        }
    }

    return false;

}

void PathFinder::computeTrack(){
    // If there are not enough cones, just return
    if (cones_.size() < 2)
        return;

    trackSection tmpSection;
    // If this is the first goal, find based on car position instead of prev goal
    if (goals_.size() == 0){
        // Find next goal based on the previous goal
        if (findNextTrackSection(initalOdo_.pose.pose.position, tmpSection) == false)
            return;
        
        // Add track section to appropriate members
        trackSections_.push_back(tmpSection);
        if (automaticGoals_ == true)
            goals_.push_back(tmpSection.Centre);
    }

        

    // Figure out how many goals we need to find to reach the requested look ahead amount
    int numGoalsCalculate = lookAhead_ - (goals_.size() - (nextGoalIndex_));
    
    for (int i = 0; i < numGoalsCalculate; i++){
        // If there are not enough cones, just return
        if (cones_.size() < 2)
            return;
        
        // Find next track section based on the previous track section
        if (findNextTrackSection(trackSections_.back().Centre, tmpSection) == false)
            return;

        // Add track section to appropriate members
        trackSections_.push_back(tmpSection);
        if (automaticGoals_ == true)
            goals_.push_back(tmpSection.Centre);
    }
}

vector<Spline::Coefficents> PathFinder::computeSplineCoefficients(vector<double> splineAxisPoints) {
    int numOfCubics = splineAxisPoints.size() - 1;

    vector<Spline::Coefficents> coefs;

    // Fill out coefiecents, 0s will be replaced later
    for (auto axisPoint : splineAxisPoints){
        coefs.push_back(Spline::Coefficents {axisPoint, 0,0,0});
    }

    // Intermediate calculation vector
    std::vector<double> intermediateValues(numOfCubics);
    // Diagonal matrix vector elements
    std::vector<double> diagonalElements(numOfCubics), subDiagonalElements(numOfCubics), solutionVector(numOfCubics);

    // Calculate the intermediate values
    for (int i = 1; i < numOfCubics; i++) {
        intermediateValues[i] = (3.0 * (coefs[i + 1].a - coefs[i].a)) - (3.0 * (coefs[i].a - coefs[i - 1].a));
    }
    
    // The start and end of the tridiagonal matrix are different because the spline is not closed

    // Inital values of the Tridiagonal matrix
    diagonalElements[0] = 2.0;
    subDiagonalElements[0] = 0.0;
    solutionVector[0] = 0.0;

    // Computing the Tridiagonal Matrix
    for (int i = 1; i < numOfCubics; i++) {
        diagonalElements[i] = 2.0 * 2.0 - 1.0 * subDiagonalElements[i - 1];
        subDiagonalElements[i] = 1.0 / diagonalElements[i];
        solutionVector[i] = (intermediateValues[i] - 1.0 * solutionVector[i - 1]) / diagonalElements[i];
    }
    
    // Setting end of spline 
    diagonalElements[numOfCubics] = 2.0;
    solutionVector[numOfCubics] = 0.0;
    coefs[numOfCubics].c = 0.0;

    // Calculating coeficents
    for (int i = numOfCubics-1; i >= 0; i--) {
        coefs[i].c = solutionVector[i] - subDiagonalElements[i] * coefs[i + 1].c;
        coefs[i].b = (coefs[i + 1].a - coefs[i].a) - (coefs[i + 1].c + 2.0 * coefs[i].c) / 3.0;
        coefs[i].d = (coefs[i + 1].c - coefs[i].c) / 3.0;
    }

    return coefs;
}

bool PathFinder::computePursuitPoint(Point &nextGoal, vector<Point> controlPoints){

    // If we don't have enough points to make a spline, exit
    if (controlPoints.size() < 3){
        return false;
    }

    // Separate the control points into x and y components
    std::vector<double> controlPoints_x;
    std::vector<double> controlPoints_y;

    for (auto point : controlPoints){
        controlPoints_x.push_back(point.x);
        controlPoints_y.push_back(point.y);
    }

    // Computing cubic spline coefficents for x and y
    vector<Spline::Coefficents> x_coefs = computeSplineCoefficients(controlPoints_x);
    vector<Spline::Coefficents> y_coefs = computeSplineCoefficients(controlPoints_y);

    Point interpolatedPoint;
    for (int i = 0; i < controlPoints_x.size()-1; i++){
        // Calculate interpolated points with specified resolution
        for (double t = 0.0; t < 1.0; t += resolution_) {
            interpolatedPoint.x = x_coefs[i].a + x_coefs[i].b * pow(t,1) + x_coefs[i].c * pow(t,2) + x_coefs[i].d * pow(t,3);
            interpolatedPoint.y = y_coefs[i].a + y_coefs[i].b * pow(t,1) + y_coefs[i].c * pow(t,2) + y_coefs[i].d * pow(t,3);

            // Once we find a point along the spline that is far enough away, we return it
            if (hypot(interpolatedPoint.x - odo_.pose.pose.position.x, interpolatedPoint.y - odo_.pose.pose.position.y) > pursuitDistance_){
                nextGoal = interpolatedPoint;
                return true;
            }
        }
    }

    return false;
}