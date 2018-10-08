#include <planning/exploration.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <maebot/maebot_channels.h>
#include <slam/slam_channels.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <cassert>

// assumptions for these constants:
// paths points lay on the center of grids cells, robot moves its physical center thought the center of these cells in the grid 
// there is error in the position estimate of up to  ?
// there is error in the map/wall location estimate of up to 0.05 (1 cell), specially in corners 
const float kMinDistanceToObstacle = 2*0.05f;    // robot diameter is 7.5cm, stepping on cell #2 could easily cause a collision
const float kMaxDistanceWithCost = 3*0.05f;      // preferably we keep 3 cells away from walls but if needed we can step on cell #3
const float kDistanceCostExponent = 1.0f ;       // 
const float kNonNavigable = 0;                  // inflatedMap cells with values larger than this are non-navigable
const float kReachedPositionThreshold = 0.05f;  // must get within this distance of a position for it to be explored
const float kMinFrontierLength = 0.1;

// Define an equality operator for poses to allow direct comparison of two paths
bool operator==(const pose_xyt_t& lhs, const pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}


Exploration::Exploration(int32_t teamNumber, 
                         bool shouldAttemptEscape,
                         const std::string& targetFile,
                         lcm::LCM* lcmInstance)
: teamNumber_(teamNumber)
, state_(exploration_status_t::STATE_INITIALIZING)
, shouldAttemptEscape_(shouldAttemptEscape)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
{
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Exploration::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Exploration::handlePose, this);
    
    currentPath_.path_length = 0;
    
    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    // If a target file was specified, then attempt to load the targets
    if((targetFile.length() > 0) && shouldAttemptEscape_)
    {
        std::ifstream targetIn(targetFile);
        if(targetIn.is_open())
        {
            // Ignoring I/O error processing here
            targetIn >> keyPose_.x >> keyPose_.y >> keyPose_.theta
                >> treasurePose_.x >> treasurePose_.y >> treasurePose_.theta;
            
            std::cout << "INFO: Exploration: Running in attempt escape mode.\n"
                << "Key pose: " << keyPose_.x << ',' << keyPose_.y << ',' << keyPose_.theta << '\n'
                << "Treasure pose: " << treasurePose_.x << ',' << treasurePose_.y << ',' << treasurePose_.theta << '\n';
        }
        else
        {
            std::cerr << "ERROR: Exploration: Failed to open the target file: " << targetFile << '\n';
        }
    }
    else
    {
        shouldAttemptEscape_ = false;
        std::cout << "INFO: Exploration: Running in explore and return home mode.\n";
    }
    
    // Initialize the escape pose to something far off the map, so it can't accidentally be correct
    escapePose_.x = escapePose_.y = escapePose_.theta = -1000000.0f;

    params_.minDistanceToObstacle = kMinDistanceToObstacle;
    params_.maxDistanceWithCost = kMaxDistanceWithCost;
    params_.distanceCostExponent = kDistanceCostExponent;
}


bool Exploration::exploreEnvironment()
{
    while((state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runExploration();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }
    }
    
    // If the state is completed, then we didn't fail
    return state_ == exploration_status_t::STATE_COMPLETED_EXPLORATION;
}


void Exploration::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Exploration::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}


bool Exploration::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Exploration::runExploration(void)
{
    assert(isReadyToUpdate());
    
    copyDataForUpdate();
    executeStateMachine();
}


void Exploration::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    
    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
    }
    
    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;
    
    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        haveHomePose_ = true;
        std::cout << "INFO: Exploration: Set home pose:" << homePose_.x << ',' << homePose_.y << ',' 
            << homePose_.theta << '\n';
    }
}


void Exploration::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;
    
    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;
    
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing();
                break;
            case exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeExploringMap(stateChanged);
                break;
                
            case exploration_status_t::STATE_RETURNING_HOME:
                nextState = executeReturningHome(stateChanged);
                break;
                
            case exploration_status_t::STATE_FINDING_KEY:
                nextState = executeFindingKey(stateChanged);
                break;
                
            case exploration_status_t::STATE_GRABBING_TREASURE:
                nextState = executeGrabbingTreasure(stateChanged);
                break;
                
            case exploration_status_t::STATE_ESCAPING_MAP:
                nextState = executeEscapingMap(stateChanged);
                break;
                
            case exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;
                
            case exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;
        }
        
        stateChanged = nextState != state_;
        state_ = nextState;
        
    } while(stateChanged);
    
    if(previousPath.path != currentPath_.path)
    {
        std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Maebot...\n";
        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }
}


int8_t Exploration::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to exploring once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    return exploration_status_t::STATE_EXPLORING_MAP;
}


bool Exploration::nearestReachableFrontier(Point<int> &nf, const pose_xyt_t& rPose)  //returns in nf the grid coordinates to the neares forntier point if it exist
{
    frontier_t f;
    Point<int> fp;
    fp.x = fp.y = nf.x = nf.y = -1; // initial value outside the grid range
    int d =0;
    int minDist = 500; //set to 500 cells larger than max posible distance between 2 points of the grid (200 +200) cells

    if(frontiers_.empty())
        {return false;}             //no frontiers exist

    for(auto f_it : frontiers_)     //for all frontiers
    {
        f = f_it;
        for (auto p_it : f.cells)         //for all points within each frontier
        {    
            fp = p_it;
            // frontiers are by definition reachable (connected to a previous robot position by a beam) and straingt lines
            // however the robot cannot drive to the points of the frontier that are close to the wall (in the padding zone)
            if (inflatedMap_(fp.x,fp.y) >= kNonNavigable) //skip if the point is in padding zone (notReachable)
                d = abs(rPose.x-fp.x)+abs(rPose.y-fp.y);        // we could use a more precise mesure of distance if this behaves poorly
                if ( d < minDist )                              // frontier point is closer than current closest point
                {
                    nf = fp;
                    minDist = d;
                }
        }
    }
    if ( minDist < 500 ) {return true;} //found nearest frontier reurene dby reference in &nf
    return false;                       //did not find a nearest frontier, only if no frontier points are reachable
}

int8_t Exploration::executeExploringMap(bool initialize)
{
    //////////////////////// TODO: Implement your method for exploring the map ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) frontiers_.empty() == true      : all frontiers have been explored as determined by find_map_frontiers()
    *       (2) currentPath_.path_length > 1 : currently following a path to the next frontier
    * 
    *   - Use the provided function find_map_frontiers() to find all frontiers in the map.
    *   - You will need to implement logic to select which frontier to explore.
    *   - You will need to implement logic to decide when to select a new frontier to explore. Take into consideration:
    *       -- The map is evolving as you drive, so what previously looked like a safe path might not be once you have
    *           explored more of the map.
    *       -- You will likely be able to see the frontier before actually reaching the end of the path leading to it.
    */
    
    //frontier_t f;       // used to call frontier functions, data not use in f
    Point<int> nf;      // used to store nearest frontier grid coordinates
    nf.x = nf.y = -1;   // initialize to values outside the grid
    pose_xyt_t nf_pose;

    nf_pose.x = nf.x;
    nf_pose.y = nf.y;
    nf_pose.theta = 0;  // temporarily at 0, check with Ted

    inflatedMap_.setDistances(currentMap_);
    frontiers_ = find_map_frontiers(currentMap_, currentPose_, kMinFrontierLength); //minFrontierLength default is 0.1m 

    while (nearestReachableFrontier(nf,currentPose_)) 
    {
        /**
        arrivedFlag = 0;
        arrivedFlag = moveTo(nf);
        if(!arrivedFlag) 
            { printf("Frontier not reachable, skipping to next forntier"); }
        */
        currentPath_ = search_for_path(currentPose_, nf_pose, inflatedMap_, params_);
        // nf is by definition Reachable so no need to check that here, A* call should always find a reachable path here
        arrivedFlag_ = false;
        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_); 
        // Check with TED : PID should update arrivedFlag when it reaches goal, nf has been checked before to be reachable
        do { usleep(100000);} while(!arrivedFlag_);  
        if(arrivedFlag_) 
            { printf("Arrived to a Frontier, will check if there exist any other reachable forntier"); }
        // need to  remove these 2 lines and only run one iteration here, zoom out to run expl, out to explore environment and back in for next frontier
        inflatedMap_.setDistances(currentMap_);
        frontiers_ = find_map_frontiers(currentMap_, currentPose_, kMinFrontierLength); 
    }  
    // at the end of this loop one of two conditions are true
    // (1) frontiers_.empty() is false however none is reachable - > STATUS_FAILED to fully explore the map
    // (2) frontiers_.empty() is true in which case we completed the exploration  - STATUS_COMPLETE

    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    
    // If no frontiers remain, then exploration is complete
    if(frontiers_.empty())
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    
    /** GC : deleted segment of original code 
    // for each reachable frontier we wait until we arrive to it before we come here, so we will never be "in progress" here
    // Else if there's a path to follow, then we're still in the process of exploring
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    */
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        /** We should never enter here
        case exploration_status_t::STATUS_IN_PROGRESS:
            return exploration_status_t::STATE_EXPLORING_MAP;
        */    
        // If exploration is completed, then head home
        case exploration_status_t::STATUS_COMPLETE:
            return exploration_status_t::STATE_RETURNING_HOME;
            
        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case exploration_status_t::STATUS_FAILED:
            return exploration_status_t::STATE_FAILED_EXPLORATION;
            
        default:
            std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
            return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeReturningHome(bool initialize)
{
    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */
    
    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_RETURNING_HOME;
    
    double distToHome = distance_between_points(Point<float>(homePose_.x, homePose_.y), 
                                                Point<float>(currentPose_.x, currentPose_.y));
    // If we're within the threshold of home, then we're done.
    if(distToHome <= kReachedPositionThreshold)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_RETURNING_HOME;
    }
    else if(status.status == exploration_status_t::STATUS_COMPLETE)
    {
        return shouldAttemptEscape_ ? exploration_status_t::STATE_FINDING_KEY
            : exploration_status_t::STATE_COMPLETED_EXPLORATION;
    }
    else // if(status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeFindingKey(bool initialize)
{
    //////////////////////// TODO: Implement your method for finding the key (drive to keyPose_)  ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, keyPose_) < kReachedPositionThreshold  :  reached the key pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the key pose
    */
    
    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_FINDING_KEY;
    
    double distToKey = distance_between_points(Point<float>(keyPose_.x, keyPose_.y),
                                               Point<float>(currentPose_.x, currentPose_.y));
    // If we're within the threshold of the key, then we're done.
    if(distToKey <= kReachedPositionThreshold)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_FINDING_KEY;
    }
    else if(status.status == exploration_status_t::STATUS_COMPLETE)
    {
        return exploration_status_t::STATE_GRABBING_TREASURE;
    }
    else // if(status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeGrabbingTreasure(bool initialize)
{
    //////////////////////// TODO: Implement your method for grabbing the treasure (drive to treasurePose_) ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, treasurePose_) < kReachedPositionThreshold  :  reached the treasure pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the treasure pose
    */
    
    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_GRABBING_TREASURE;
    
    double distToTreasure = distance_between_points(Point<float>(treasurePose_.x, treasurePose_.y),
                                                    Point<float>(currentPose_.x, currentPose_.y));
    // If we're within the threshold of the treasure, then we're done.
    if(distToTreasure <= kReachedPositionThreshold)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_GRABBING_TREASURE;
    }
    else if(status.status == exploration_status_t::STATUS_COMPLETE)
    {
        return exploration_status_t::STATE_ESCAPING_MAP;
    }
    else // if(status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeEscapingMap(bool initialize)
{
    //////////////////////// TODO: Implement your method for finding the exit from the map and escaping  ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, escapePose_) < kReachedPositionThreshold  :  reached the escape pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the escape pose
    */
    
    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_ESCAPING_MAP;
    
    double distToEscape = distance_between_points(Point<float>(escapePose_.x, escapePose_.y),
                                                  Point<float>(currentPose_.x, currentPose_.y));
    // If we're within the threshold of the escape, then we're done.
    if(distToEscape <= kReachedPositionThreshold)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_ESCAPING_MAP;
    }
    else if(status.status == exploration_status_t::STATUS_COMPLETE)
    {
        return exploration_status_t::STATE_COMPLETED_EXPLORATION;
    }
    else // if(status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
    
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}


int8_t Exploration::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
}


int8_t Exploration::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}
