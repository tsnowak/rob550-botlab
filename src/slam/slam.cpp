#include <slam/slam.hpp>
#include <slam/slam_channels.h>
#include <maebot/maebot_channels.h>
#include <optitrack/optitrack_channels.h>
#include <unistd.h>
#include <cassert>
#include <fstream>

#include <planning/obstacle_distance_grid.hpp>

OccupancyGridSLAM::OccupancyGridSLAM(int         numParticles,
                                     int8_t      hitOddsIncrease,
                                     int8_t      missOddsDecrease,
                                     lcm::LCM&   lcmComm,
                                     bool waitForOptitrack,
                                     bool mappingOnlyMode,
                                     const std::string localizationOnlyMap)
: mode_(full_slam)  // default is running full SLAM, unless user specifies otherwise on the command line
, haveInitializedPoses_(false)
, waitingForOptitrack_(waitForOptitrack)
, haveMap_(false)
, numIgnoredScans_(0)
, filter_(numParticles)
, map_(10.0f, 10.0f, 0.05f)  // create a 10m x 10m grid with 0.05m cells
, mapper_(5.0f, hitOddsIncrease, missOddsDecrease)
, lcm_(lcmComm)
, mapUpdateCount_(0)
{
    // Confirm that the mode is valid -- mapping-only and localization-only are not specified
    assert(!(mappingOnlyMode && localizationOnlyMap.length() > 0));
    
    // Determine which mode to run based on the inputs
    if(mappingOnlyMode)
    {
        mode_ = mapping_only;
    }
    else if(localizationOnlyMap.length() > 0)
    {
        haveMap_ = map_.loadFromFile(localizationOnlyMap);
        assert(haveMap_);   // if there's no map, then the localization can't run!
        //std::cerr << "Load Map Successful!" << haveMap_ << "\n";
        
        mode_ = localization_only;
    }
    
    currentOdometry_.utime = 0;
    currentScan_.utime = 0;
    
    // Laser and odometry data are always required
    lcm_.subscribe(RPLIDAR_LASER_CHANNEL, &OccupancyGridSLAM::handleLaser, this);
    lcm_.subscribe(ODOMETRY_CHANNEL, &OccupancyGridSLAM::handleOdometry, this);
    lcm_.subscribe(TRUE_POSE_CHANNEL, &OccupancyGridSLAM::handleOptitrack, this);
    
    // If we are only building the occupancy grid using ground-truth poses, then subscribe to the ground-truth poses.
    if(mode_ == mapping_only)
    {
        lcm_.subscribe(SLAM_POSE_CHANNEL, &OccupancyGridSLAM::handlePose, this);
    }
    
    // Zero-out all the poses to start. Either the robot will start at (0,0,0) or at the first pose received from the
    // Optitrack system.
    initialPose_.x = initialPose_.y = initialPose_.theta = 0.0f;
    previousPose_.x = previousPose_.y = previousPose_.theta = 0.0f;
    currentPose_.x  = currentPose_.y  = currentPose_.theta  = 0.0f;
}


void OccupancyGridSLAM::runSLAM(void)
{
    while(true)
    {
        // If new data has arrived
        if(isReadyToUpdate())
        {
            // Then run an iteration of our SLAM algorithm
            //std::cerr << "Ready to Update" << "\n";
            runSLAMIteration();
            //std::cerr << "Ran SLAM iteration" << "\n";
        }
        // Otherwise, do a q
        //need to update left and right wheel speed according to definition/use uick spin while waiting for data rather than using more complicated condition variable.
        else
        {
            //printf("sun SLAM :: usleep\n");
            usleep(1000);
        }
    }
}


// Handlers for LCM messages
void OccupancyGridSLAM::handleLaser(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const rplidar_laser_t* scan)
{
    const int kNumIgnoredForMessage = 10;   // number of scans to ignore before printing a message about odometry
    
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    // Ignore scans until odometry data arrives -- need odometry before a scan to safely built the map
    bool haveOdom = (mode_ != mapping_only) // For full SLAM, odometry data is needed.
        && !odometryPoses_.empty() 
        && (odometryPoses_.front().utime <= scan->times.front());
    bool havePose = (mode_ == mapping_only) // For mapping-only, ground-truth poses are needed
        && !groundTruthPoses_.empty() 
        && (groundTruthPoses_.front().utime <= scan->times.front());
    
    //printf("Mode = %i \n", mode_);

    // If there's appropriate odometry or pose data for this scan, then add it to the queue.
    if(haveOdom || havePose)
    {
        incomingScans_.push_back(*scan);
        
        // If we showed the laser error message, then provide another message indicating that laser scans are now
        // being saved
        if(numIgnoredScans_ >= kNumIgnoredForMessage)
        {
            std::cout << "INFO: OccupancyGridSLAM: Received odometry or pose data, so laser scans are now being \
                saved.\n";
            numIgnoredScans_ = 0;
        }
    }
    // Otherwise ignore it
    else
    {
        ++numIgnoredScans_;
    }
    
    if(numIgnoredScans_ == kNumIgnoredForMessage)
    {
        std::cout << "INFO: OccupancyGridSLAM: Ignoring scan because no odometry data is available. \
            Please start the odometry module or use a log with ground-truth poses.\n";
    }
}


void OccupancyGridSLAM::handleOdometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* odometry)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    
    pose_xyt_t odomPose;
    odomPose.utime = odometry->utime;
    odomPose.x = odometry->x;
    odomPose.y = odometry->y;
    odomPose.theta = odometry->theta;
    odometryPoses_.addPose(odomPose);
    //printf("UPDATE HANDLE Odometry values,%f,%f,%f\n",odomPose.x,odomPose.y,odomPose.theta);
}


void OccupancyGridSLAM::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    //printf("SLAM: currentPose_: x: %f\ty: %f\tth: %f\n", (*pose).x, (*pose).y, (*pose).theta);
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    groundTruthPoses_.addPose(*pose);
}


void OccupancyGridSLAM::handleOptitrack(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);

    //printf("Optitrack start :: waiting for Optitrack flag = %i \n\n", waitingForOptitrack_);
    if(waitingForOptitrack_)
    {
        initialPose_ = *pose;
        waitingForOptitrack_ = false;
        //printf("Optitrack start :: Initial pose x,y,theta = %1.2f,%1.2f,%1.2f \n", initialPose_.x, initialPose_.y, initialPose_.theta);
    }
}


bool OccupancyGridSLAM::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    
    bool haveData = false;
    
    //printf("GridSLAM :: incomingScans? = %i \n", incomingScans_.empty());

    // If there's at least one scan to process, then check if odometry/pose information is available
    if(!incomingScans_.empty())
    {
        // Find if there's a scan that there is odometry data for
        const rplidar_laser_t& nextScan = incomingScans_.front();
        
        // Ensure that there's a pose that exists at or after the final laser measurement to be sure that valid
        // interpolation of robot motion during the scan can be performed.
        
        // Only care if there's odometry data if we aren't in mapping-only mode
        bool haveNewOdom = (mode_ != mapping_only) && (odometryPoses_.containsPoseAtTime(nextScan.times.back()));
        // Otherwise, only see if a new pose has arrived
        bool haveNewPose = (mode_ == mapping_only) && (groundTruthPoses_.containsPoseAtTime(nextScan.times.back()));
        
        haveData = haveNewOdom || haveNewPose;
    }
    
    // If all SLAM data and optitrack data has arrived, then we're ready to go.
    //printf("GridSLAM :: is ready to update haveData?  = %i \n", haveData);
    return haveData && !waitingForOptitrack_;
}


void OccupancyGridSLAM::runSLAMIteration(void)
{
    copyDataForSLAMUpdate();
    //std::cerr << "Copied Data for SLAM update" << "\n";
    initializePosesIfNeeded();
    //std::cerr << "Initialized poses" << "\n";
    
    // Sanity check the laser data to see if rplidar_driver has lost sync
    if(currentScan_.num_ranges > 250)
    {
        //std::cerr << "Rplidar in sync" << "\n";
        updateLocalization();
        //std::cerr << "Updated Localization" << "\n";
        updateMap();
        //std::cerr << "Updated Map" << "\n";
    }
    else 
    {
        std::cerr << "ERROR: OccupancyGridSLAM: Detected invalid laser scan with " << currentScan_.num_ranges 
            << " ranges.\n";
    }
}


void OccupancyGridSLAM::copyDataForSLAMUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    
    // Copy the data needed for the new SLAM update
    currentScan_ = incomingScans_.front();
    incomingScans_.pop_front();
    
    if(mode_ == mapping_only)
    {
        // No localization is performed during mapping-only mode, so the previous pose needs to be correctly adjusted
        // here.
        previousPose_ = currentPose_;
        currentPose_  = groundTruthPoses_.poseAt(currentScan_.times.back());
    }
    else
    {
        currentOdometry_ = odometryPoses_.poseAt(currentScan_.times.back());
        //printf("UPDATE COPY DATA Odometry values,%f,%f,%f\n",currentOdometry_.x,currentOdometry_.y,currentOdometry_.theta);

    }
}


void OccupancyGridSLAM::initializePosesIfNeeded(void)
{
    // The initial poses need to be set with the timestamps associated with the first last scan to ensure that proper
    // interpolation of the laser scan happen in MovingLaserScan. This initialization requires the timestamp of the
    // first laser scan, so it can't be performed in the constructor.
    if(!haveInitializedPoses_)
    {
        // In mapping-only mode, the reference frame is defined by the SLAM pose. The robot doesn't necessarily
        // start at (0, 0, 0) though.
        if(mode_ == mapping_only)
        {
            initialPose_ = groundTruthPoses_.poseAt(currentScan_.times.back());
        }
        
        previousPose_ = initialPose_;
        previousPose_.utime = currentScan_.times.front();
        
        currentPose_ = previousPose_;
        currentPose_.utime  = currentScan_.times.back();
        haveInitializedPoses_ = true;
        
        filter_.initializeFilterAtPose(previousPose_);
    }
    
    assert(haveInitializedPoses_);
}


void OccupancyGridSLAM::updateLocalization(void)
{
    if(haveMap_ && (mode_ != mapping_only))
    {
        //std::cerr << "Have map and mode not mapping only" << "\n";
        previousPose_ = currentPose_;
        //std::cerr << "Update previous pose" << "\n";
        //printf("UPDATE LOCALIZATION Odometry values,%f,%f,%f\n",currentOdometry_.x,currentOdometry_.y,currentOdometry_.theta);
        currentPose_  = filter_.updateFilter(currentOdometry_, currentScan_, map_);
        //std::cerr << "Got current pose" << "\n";
        
        auto particles = filter_.particles();

        lcm_.publish(SLAM_POSE_CHANNEL, &currentPose_);
        lcm_.publish(SLAM_PARTICLES_CHANNEL, &particles);
    }
}


void OccupancyGridSLAM::updateMap(void)
{
    if(mode_ != localization_only)
    {
        // Process the map
        //GC mapper_.updateMap(currentScan_, currentPose_, map_);

        mapper_.updateMap(currentScan_, previousPose_, currentPose_, map_);  // passing previous pose to interpolate lidar scans when updating map
        haveMap_ = true;
    }

    // Publish the map even in localization-only mode to ensure the visualization is meaningful
    // Send every 5th map -- about 1Hz update rate for map output -- can change if want more or less during operation
    if(mapUpdateCount_ % 4 == 0)
    {
        auto mapMessage = map_.toLCM();
        lcm_.publish(SLAM_MAP_CHANNEL, &mapMessage);
        map_.saveToFile("current.map");
        printf("PUBLISHED updated map 5 times, mapUpdateCount = %i \n\n", mapUpdateCount_);
        map_.saveToFile("grid_map_.txt");
    }

    /**

    switch(mapUpdateCount_) 
    {
      case 0 :
         map_.saveToFile("mmap0.txt");
         break;
      case 1 :
         map_.saveToFile("mmap1.txt");
         break;
      case 5 :
         map_.saveToFile("mmap5.txt");
         break;
      case 10 :
         map_.saveToFile("mmap10.txt");
         break;
      case 50 :
         map_.saveToFile("mmap50.txt");
         break;
      case 100 :
         map_.saveToFile("mmap100.txt");
         break;
      case 400 :
         map_.saveToFile("mmap400.txt");
         break;
      case 800 :
         map_.saveToFile("mmap800.txt");
         break;
      case 2000 :
         map_.saveToFile("mmap2000.txt");
         break;
      default :
         break;
    }
    */

    map_.saveToFile("current.map");
    ++mapUpdateCount_;
}
