#ifndef APPS_SLAM_MAP_STATISTICS_HPP
#define APPS_SLAM_MAP_STATISTICS_HPP

#include <lcmtypes/pose_xyt_t.hpp>

class OccupancyGrid;

/**
* map_correlation calculates the correlation between two occupancy grids. Only the intersection of the grids is
* considered.
* 
* The correlation considers three types of cells:
*   
*   - occupied : log_odds > 0
*   - unknown : log_odds == 0
*   - free : log_odds < 0
* 
* \param    lhs         One of the grids being considered
* \param    rhs         The other grid being considered
* \return   The correlation value between the maps in the range [-1, 1], where a value of 1 is identical and a value of
*           -1 means the inverse.
*/
double map_correlation(const OccupancyGrid& lhs, const OccupancyGrid& rhs);

/**
* amount_explored finds the amount of the map that has been explored. The amount explored how much of the boundary of
* free space is a frontier.
* 
* The boundary is determined by searching from the robot's current pose until a boundary with an unknown or occupied
* cell is encountered.
* 
* \param    grid            Grid in which to find the exploration amount
* \param    robotPose       Current pose of the robot in the grid
* \return   Amount of the boundary that has been explored [0, 1].
*/
double amount_explored(const OccupancyGrid& grid, const pose_xyt_t& robotPose);

#endif // APPS_SLAM_MAP_STATISTICS_HPP
