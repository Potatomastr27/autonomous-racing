#include "laserProcessing.h"


LaserProcessing::LaserProcessing(LaserScan scan, Pose scanPose, double coneDiameter, double sameConeDist, int minRanges):
    scan_(scan),
    scanPose_(scanPose),
    coneDiameter_(coneDiameter),
    sameConeDist_(sameConeDist),
    minRanges_(minRanges),
    blacklistedAreas_({})
{

}

void LaserProcessing::updateScan(LaserScan scan, Pose scanPose){
    scan_ = scan;
    scanPose_ = scanPose;
}

vector<Point> LaserProcessing::getBlacklist(){
    return blacklistedAreas_;
}

vector<Point> LaserProcessing::findCones(){
    double distBetweenRanges;
    vector<Polar> coneRanges; // A list of all ranges that make up the current cone
    vector<Point> cones; // The x,y pos of each found cone
    // Start from 1 so we dont get a seg fault
    for (unsigned int i = 1; i < scan_.ranges.size(); i++){

        bool isCurRangeInvalid = scan_.ranges[i] == INFINITY || scan_.ranges[i] == NAN || scan_.ranges[i] >= scan_.range_max || scan_.ranges[i] < scan_.range_min;
        bool isPrevRangeInvalid = scan_.ranges[i-1] == INFINITY || scan_.ranges[i-1] == NAN || scan_.ranges[i-1] >= scan_.range_max || scan_.ranges[i-1] < scan_.range_min;

        // If both readings are invalid immediately skip as there is no useful data here
        if (isCurRangeInvalid && isPrevRangeInvalid)
            continue;
        
        // If the previous reading is invalid but current is valid, we have found a new cone so start adding up the average
        if (isPrevRangeInvalid){
            // If the cur reading is not invalid but prev is, add it to the avg and then skip
            coneRanges.push_back({}); 
            coneRanges.back().range = scan_.ranges[i];
            coneRanges.back().angle = scan_.angle_increment * (i) + scan_.angle_min;
            continue;
        }
            
        // If this reading is invalid, but the previous isnt, we have found the end of a cone so calc the average and add the cone
        if (isCurRangeInvalid){

            Point cone;
            if (analyseRanges(coneRanges, cone) == true)
                cones.push_back(cone);
            // Its the end of a cone so clear the ranges
            coneRanges.clear();   
            
            continue;
        }

        // If none of the previous if statements triggered we know both ranges are valid so calculate the distance between them:
        
        // Use cosine rule to get distance between the two readings
        // We know the two readings are 1 angle increment apart which gives angle C and we have the two side lengths a and b so we can find side length c
        distBetweenRanges = sqrt(pow(scan_.ranges[i-1], 2) + pow(scan_.ranges[i], 2) - 2 * scan_.ranges[i-1] * scan_.ranges[i] * cos(scan_.angle_increment));

        // If the two ranges are more than 0.3m apart, we have found the end of the current cone and the start of a new one
        if (distBetweenRanges >= sameConeDist_){

            // If this is a valid cone add it
            Point cone;
            if (analyseRanges(coneRanges, cone) == true)
                cones.push_back(cone);

            // Set the new cones first point as initial values rather than 0
            coneRanges.clear();
            // Start of new cone
            coneRanges.push_back({}); 
            coneRanges.back().range = scan_.ranges[i];
            coneRanges.back().angle = scan_.angle_increment * (i) + scan_.angle_min;
            continue;
        }

        // If none of the previous if statements triggered we know the two points are part of the same cone so continue adding points to vec
        coneRanges.push_back({}); 
        coneRanges.back().range = scan_.ranges[i];
        coneRanges.back().angle = scan_.angle_increment * (i) + scan_.angle_min;       
    }



    // Return all cones found
    return cones;
}

bool LaserProcessing::isObsInWay(double width, double height){
    // Convert all valid ranges into a cartesian coordinate
    vector<Point> points;
    // Start the angle at one before the minimum cause we will add one back in the for loop
    double rangeAngle = scan_.angle_min - scan_.angle_increment;
    for (auto range : scan_.ranges){
        rangeAngle += scan_.angle_increment;
        // Ensure range is valid
        if (range == INFINITY || range == NAN || range >= scan_.range_max || range < scan_.range_min)
            continue;
        
        // Convert to x,y and add it to vec
        Point point;
        double globalAngle = rangeAngle + tf2::getYaw(scanPose_.orientation);
        point.x = range * cos (globalAngle) + scanPose_.position.x;
        point.y = range * sin (globalAngle) + scanPose_.position.y;  
        points.push_back(point);
    }

    // If there are no points, there are no obstacles
    if (points.size() == 0)
        return false;

    // Define the box where we look for obstacles
    Point topleftCorner, bottomRightCorner;
    Polar tmpPolar;

    //Define top left corner of box
    tmpPolar.range = hypot(width/2, height);
    tmpPolar.angle = atan(width/height);
    topleftCorner = rangeToPoint(tmpPolar, scanPose_);

    // Define Bottom right corner
    tmpPolar.range = width/2;
    tmpPolar.angle = -M_PI_2;
    bottomRightCorner = rangeToPoint(tmpPolar, scanPose_);

    // Check if any detected scan points are within the defined box
    for (auto point : points){
        // Check if point is within the defined box
        if (point.x < topleftCorner.x && point.x > bottomRightCorner.x && point.y > bottomRightCorner.y && point.y < topleftCorner.y){
            // If it is we might run into it so return true
            return true;
        }
    }

    return false;
}

bool LaserProcessing::analyseRanges(vector<Polar> ranges, Point &cone){
    // If there aren't enough ranges, we don't have enough data to analyse
    if (ranges.size() < minRanges_)
        return false;

    // Calculate span using cosine rule
    double cosineC = ranges.front().angle - ranges.back().angle;
    double span = sqrt(pow(ranges.back().range, 2) + pow(ranges.front().range, 2) - 2 * ranges.front().range * ranges.back().range * cos (cosineC));
    
    // Convert to global points
    vector<Point> rangePoints;
    for (auto range : ranges){
        rangePoints.push_back(rangeToPoint(range, scanPose_));
    }

    // If the span is greater than the maximum cone diameter we blacklist these ranges as they are not a cone
    if (span > coneDiameter_ + 0.1){
        blacklistRanges(rangePoints);
        return false;
    }

    

    bool shouldBeBlacklisted = false;
    // Iterate through each combination of point and blacklisted point
    for (auto point : rangePoints){
        for (auto blacklistedPoint : blacklistedAreas_){
            // These points are part of the same object so we blacklist them all
            if (hypot(point.x - blacklistedPoint.x, point.y - blacklistedPoint.y) < 0.5){
                shouldBeBlacklisted = true;
                goto exitLoops;
            }
        }
    }
exitLoops:
    
    if (shouldBeBlacklisted == true){
        // Blacklist the ranges
        blacklistRanges(rangePoints);
        return false;
    }

    // Return the cone as this is a valid cone
    cone = avgRangesToPoint(ranges, scanPose_, coneDiameter_/2);    
    return true;
}
    
Point LaserProcessing::avgRangesToPoint(vector<Polar> ranges, Pose refPose, double offset){
    
    // Calculate the average of all the ranges
    Polar avgRange = {0,0};
    for (int i = 0; i < ranges.size(); i++){
        avgRange.range += ranges[i].range;
        avgRange.angle += ranges[i].angle;
    }
    avgRange.range = avgRange.range / ranges.size();
    avgRange.angle = avgRange.angle / ranges.size();

    // Modify range by the specified offset (usually the conediam/2)
    avgRange.range += offset;

    return rangeToPoint(avgRange, refPose);
}

void LaserProcessing::blacklistRanges(vector<Point> points){
    bool shouldBeBlacklisted = true;
    // Iterate through each point
    for (auto point : points){
        shouldBeBlacklisted = true;
        for (auto blklistPoint : blacklistedAreas_){
            // If the point is close enough to an already existing blacklisted point we don't add it to the blacklist to avoid endless data accumulation
            if (hypot (blklistPoint.x - point.x, blklistPoint.y - point.y) < 0.5){
                shouldBeBlacklisted = false;
                break;
            }
                
            
        }
        // If bool is still true after iterating through all blacklisted points, we blacklist it
        if (shouldBeBlacklisted)
            blacklistedAreas_.push_back(point);        
    }
}

Point LaserProcessing::rangeToPoint(Polar range, Pose refPose){
    Point point;
    double globalAngle = range.angle + tf2::getYaw(refPose.orientation);
    point.x = range.range * cos (globalAngle) + refPose.position.x;
    point.y = range.range * sin (globalAngle) + refPose.position.y;  
    return point;
}