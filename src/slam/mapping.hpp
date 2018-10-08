#ifndef SLAM_MAPPING_HPP
#define SLAM_MAPPING_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <cstdint>
//GC added
#include <lcmtypes/rplidar_laser_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/moving_laser_scan.hpp>


class OccupancyGrid;
class rplidar_laser_t;


/**
* Mapping implements the occupancy grid mapping algorithm.  On each map update, the updateMap method is called. The
* provided laser scan should be used to update the provided OccupancyGrid.
*/
class Mapping
{
public:
    
    /**
    * Constructor for Mapping.
    * 
    * \param    maxLaserDistance    Maximum distance for the rays to be traced
    * \param    hitOdds             Increase in occupied odds for cells hit by a laser ray
    * \param    missOdds            Decrease in occupied odds for cells passed through by a laser ray
    */
    Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds);
   
    /**
    * updateMap incorporates information from a new laser scan into an existing OccupancyGrid.
    * 
    * \param    scan            Laser scan to use for updating the occupancy grid
    * \param    pose            Pose of the robot at the time when the last ray was measured
    * \param    map             OccupancyGrid instance to be updated
    */
    //void updateMap(const rplidar_laser_t& scan, const pose_xyt_t& pose, OccupancyGrid& map);
    void updateMap(const rplidar_laser_t& scan, const pose_xyt_t& begSPose,  const pose_xyt_t& endSPose, OccupancyGrid& map);

    
    /**
    returns 1 if there is a point in vector Phits that has coordinates row, col; else returns 0
    this is used within a single lidar scan update to check if a point was marked as hit in that scan and avoid marking it as free within that same scan
    it improves the mapping results 
    */
    bool isHitPoint(int row, int col, std::vector<Point<int>> Phits);

private:
    
    const float  kMaxLaserDistance_;
    const int8_t kHitOdds_;
    const int8_t kMissOdds_;
    
    //////////////////// TODO: Add any private members needed for your occupancy grid mapping algorithm ///////////////
};

#endif // SLAM_MAPPING_HPP
