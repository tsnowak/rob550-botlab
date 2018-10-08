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
#include <planning/astar.hpp>

// assumptions for these constants:
// paths points lay on the center of grids cells, robot moves its physical center thought the center of these cells in the grid 
// there is error in the position estimate of up to  ?
// there is error in the map/wall location estimate of up to 0.05 (1 cell), specially in corners 
const float kMinDistanceToObstacle = 2*0.05f;    // robot diameter is 7.5cm, stepping on cell #2 could easily cause a collision
const float kMaxDistanceWithCost = 3*0.05f;      // preferably we keep 3 cells away from walls but if needed we can step on cell #3
const float kDistanceCostExponent = 1.0f ;       // 
const float kReachedPositionThreshold = 0.05f;  // must get within this distance of a position for it to be explored
const float kMinFrontierLength = 0.1;
const float kNonNavigable = 3000;                  // inflatedMap cells with values larger than this are non-navigable

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
            //printf("EXPLORATION :: EXPLORE ENVIRONMENT :: running exploration\n");
            runExploration();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            //printf("EXPLORATION :: EXPLORE ENVIRONMENT :: usleep\n");
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
    //printf("exploration:: isReady to Update :: haveNewMap: %d\t haveNewPose: %d\n", haveNewMap_, haveNewPose_); 
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
        inflatedMap_.setDistances(currentMap_);
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
    printf("\n\nSTATE MACHINE : state_ = %i \n", state_);
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
    
    //printf("State Machine  :: previous path length = %i and last element x,y = %f , %f\n", previousPath.path_length, previousPath.path.end()->x, previousPath.path.end()->y);
    //printf("State Machine  :: current path length = %i and last element x,y = %f , %f\n", currentPath_.path_length, currentPath_.path.end()->x, currentPath_.path.end()->y);

    if(previousPath.path != currentPath_.path)
    {
        //std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Maebot...\n";
        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
        //printf("PUBLISHED PATH: \n");
        /*
        for (uint i = 0; i < currentPath_.path.size(); i++)
        {
            printf("%u: (%f, %f)\n", i, currentPath_.path[i].x, currentPath_.path[i].y);
        }
        printf("\n");
        */
        printf("State Machine  :: current path published to CONTROLLER_PATH_CHANNEL\n");
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


// returns true if it finds at least one reachable frontier, nf is a point with the coordinates to the nearest frontier
// returns false if there are no more frontiers or if the remaining frontiers are not reachable (too small or no A*path)
bool Exploration::nearestReachableFrontier(Point<float> &gfm, const pose_xyt_t& rPose)
{
	Point<float> fp,lfm,lgfm;
	frontier_t f;
	pose_xyt_t nf_pose;

    fp.x = fp.y = lfm.x = lfm.y = lgfm.x = lgfm.y = -7; // initial value outside the grid range
    nf_pose.x = lfm.x;
    nf_pose.y = lfm.y;
	nf_pose.theta = 0;  			// temporarily at 0, check with Ted
	float d =1000;
	float localMin = 500; 			//set to 500 cells larger than max posible distance between 2 points of the grid (200 +200) cells
	float globalMin = 500;
    //frontier_t f_it;

    float csize = 0.05; // this is the cell size 5cm
    float Xg, Yg;
    Xg = Yg = 5;  // this is the offset of the corner of the map to the center of the map. x,y are generally 0,0 in the center of the map and can take neg values
    int fp_row, fp_col;

    if(frontiers_.empty())
        {
            printf("EXP:: nearestFront = frontiers-.size() = %lu \n", frontiers_.size());
            return false;
        }             //no frontiers exist

    printf("NEAR FRONT:: frontiers_.size () = %lu \n\n", frontiers_.size());
    for(frontier_t f_it : frontiers_)     //for all frontiers
    {
    	//printf("NEAR FRONT:: initial valu eof fp Point coordinate before changin =  %f %f \n", fp.x, fp.y);// inflatedMap_(fp.x,fp.y));
        localMin = 500;
    	f = f_it;
        printf("NEAR FRONT:: number of points in frontier = %lu \n", f.cells.size());
        for (Point<float> p_it : f.cells)  	//for all points within each frontier, select closest point to current pose
        {    
            fp = p_it;
            //fp.x = p_it.x;
            //fp.y = p_it.y;
            fp_row = floor((fp.x+Xg)/csize);
            fp_col = floor((fp.y+Yg)/csize);

            //printf("NEAR FRONT::frontier point GRID coordinates %i %i  and inflated map value = %f \n", fp_row, fp_col, inflatedMap_(fp_row,fp_col));// inflatedMap_(fp.x,fp.y));
           
            // frontiers are by definition reachable (connected to a previous robot position by a beam) and straingt lines
            // however the robot cannot drive to the points of the frontier that are close to the wall (in the padding zone)
            if (inflatedMap_(fp_row,fp_col) < kNonNavigable) 		//skip if the point is in padding zone (notReachable)
            {	
                d = abs(rPose.x-fp.x)+abs(rPose.y-fp.y);        // we could use a more precise mesure of distance if this behaves poorly
                if ( d < localMin )                             // frontier point is closer than current closest point
                {
                    lfm = fp;
                    localMin = d;
                }
            }
        }
        printf("FRONTIER : CLOSEST POINT (may not be reachable) at row/col = %i,%i and x/y = %0.2f,%0.2f  with  wieght = %0.2f\n", fp_row, fp_col, lfm.x, lfm.y, inflatedMap_(fp_row,fp_col));
        if (localMin<500) 			// there is at least one point reachable in the frontier being checked
        {
        	nf_pose.x = lfm.x;
            nf_pose.y = lfm.y;
            fp_row = floor((lfm.x+Xg)/csize);
            fp_col = floor((lfm.y+Yg)/csize);           
            printf("FRONTIER : reachable point at row/col = %i,%i and x/y = %0.2f,%0.2f  with  wieght = %0.2f\n", fp_row, fp_col, lfm.x, lfm.y, inflatedMap_(fp_row,fp_col));
            //printf("NEAREST FRONTIER : global minimum currently at x = %f   y = %f  \n", gfm.x, gfm.y);
        	d = search_for_path(rPose, nf_pose, inflatedMap_, params_).path_length;        // length of A* path
            if ( d < globalMin && d>1 )                             	// this frontier point is closer than current closest point
            {
                lgfm = lfm;
                globalMin = d;
                //printf("\nUPDATING GLOBAL MINIMUM\n");
                //printf("NEAREST FRONTIER : local minimum updated to x = %f   y = %f  \n", lfm.x, lfm.y);
                //printf("NEAREST FRONTIER : global minimum updated to x = %f   y = %f  \n\n", lgfm.x, lgfm.y);
            }
        }
        else 
        { printf("FRONTIER : non reachable point in this frontier\n\n"); }
    }
    //printf("\nNEAREST FRONTIER : local frontier currently at x = %i   y = %i  \n", fp_row, fp_col);
    //printf("NEAREST FRONTIER : global minimum currently at x = %f   y = %f  \n", lgfm.x, lgfm.y);
    if ( globalMin < 500 )  //found nearest frontier returned by reference in & gfm
    {
        gfm = lgfm;
        fp_row = floor((lgfm.x+Xg)/csize);
        fp_col = floor((lgfm.y+Yg)/csize);  
        printf("NEAREST FRONTIER : lgfm ,  nearest frontier x = %i   nearest frontier y = %i  \n", fp_row, fp_col);
        printf("NEAREST FRONTIER : GlobalMin = %f   nearest frontier x = %f   nearest frontier y = %f  \n", globalMin, lgfm.x, lgfm.y);
        fp_row = floor((gfm.x+Xg)/csize);
        fp_col = floor((gfm.y+Yg)/csize);  
        printf("NEAREST FRONTIER : lgfm ,  nearest frontier x = %i   nearest frontier y = %i  \n", fp_row, fp_col);
        printf("NEAREST FRONTIER : GlobalMin = %f   nearest frontier x = %f   nearest frontier y = %f  \n", globalMin, gfm.x, gfm.y);
        return true;
    }
    printf("NEAREST FRONTIER : no frontier found ! \n");
    return false;                       	//did not find a nearest frontier, only if no frontier points are reachable
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
    
    Point<float> nf;      // used to store nearest frontier grid coordinates
    nf.x = nf.y = -1;   // initialize to values outside the grid
    pose_xyt_t nf_pose;
    bool reachableF = false;

    nf_pose.x = nf.x;
    nf_pose.y = nf.y;
    nf_pose.theta = 0;  // temporarily at 0, check with Ted

    frontiers_ = find_map_frontiers(currentMap_, currentPose_, kMinFrontierLength); //minFrontierLength default is 0.1m 
    reachableF = nearestReachableFrontier(nf,currentPose_);
    //printf("\nEXECUTE EXP MAP :: NEAREST FRONTIER RETURN VALUES : global minimum currently at x = %f   y = %f  \n",  nf.x, nf.y);
    if (reachableF) 	// a frontier exist, is reachable and path length >2
    {
    	nf_pose.x = nf.x;
        nf_pose.y = nf.y;
        currentPath_ = search_for_path(currentPose_, nf_pose, inflatedMap_, params_);
        printf("EXECUTE EXP MAP :: reachableF found, current path updated to x,y = %f , %f\n", nf_pose.x, nf_pose.y );
    }  

    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    
    // If no frontiers remain, then exploration is complete
    if(frontiers_.empty())
    {
    	printf("EXPLORE MAP : no more frontiers  COMPLETE\n");
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else if there's a path to follow, then we're still in the process of exploring
    else if(reachableF)
    {
    	printf("EXPLORE MAP : there is at least one reachable frontier IN PROGRESS\n");
    	status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    else
    {
    	printf("EXPLORE MAP : no more frontiers AND not in progress FAILED\n");
        status.status = exploration_status_t::STATUS_COMPLETE;
        //status.status = exploration_status_t::STATUS_IN_PROGRESS;
    	//status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case exploration_status_t::STATUS_IN_PROGRESS:
            return exploration_status_t::STATE_EXPLORING_MAP;

        // If exploration is completed, then head home
        case exploration_status_t::STATUS_COMPLETE:
            // TED
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
    
    currentPath_ = search_for_path(currentPose_, homePose_, inflatedMap_, params_);

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
    
	currentPath_ = search_for_path(currentPose_, keyPose_, inflatedMap_, params_);

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
    
    currentPath_ = search_for_path(currentPose_, treasurePose_, inflatedMap_, params_);

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
    	escapePose_.x = escapePose_.y = -1;  // initialize escape pose to -1, meaning escape pose not found yet
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

    Point<float> nf;      // used to store nearest frontier grid coordinates
    pose_xyt_t nf_pose;
    nf.x = nf.y = -1;   // initialize to values outside the grid
    bool reachableF = false;
    robot_path_t tmp_path;

	if (escapePose_.x!=-1) 		// we know where the exit is
	{
        currentPath_ = escapeHack;
		//currentPath_ = search_for_path(currentPose_, escapePose_, inflatedMap_, params_);
	}
	else 						// we try to find the exit
	{
		// frontiers is working on the latest map, it will find the exit if it exist or as soon as it opens up
		// it assumes we are driving pass the exit in the route home->key->treasure->home
		// this will switch path to exit as soon as found, and never again
		frontiers_ = find_map_frontiers(currentMap_, currentPose_, kMinFrontierLength); //minFrontierLength default is 0.1m
		reachableF = nearestReachableFrontier(nf,currentPose_);
		if (reachableF) 		// we found the exit, nf contains the exit coordinates
		{
            printf("ENTERED reachable escape frontier!!!\n");
			// frontier will always be outside of the arena,
			// more over it will be outside padding zone so at least 10cm outside which should ensure we crossed out
			//escapePose_.y = 0;  // temporarily, need to figure out a better setting
			nf_pose.x = nf.x;
            nf_pose.y = nf.y;

            escapeHack = search_for_path(currentPose_, nf_pose, inflatedMap_, params_);
		      
            if (escapeHack.path.size() > 1)
            {
                currentPath_ = escapeHack;
                escapePose_.x = nf.x;
                escapePose_.y = nf.y;
            } 
        }
		else  					// we have not found exit and keep heading home
		{
			currentPath_ = search_for_path(currentPose_, homePose_, inflatedMap_, params_);
		}
	}

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
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
        //status.status = exploration_status_t::STATUS_FAILED;
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
