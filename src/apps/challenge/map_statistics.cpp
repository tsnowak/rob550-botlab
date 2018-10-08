#include <apps/challenge/map_statistics.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <queue>
#include <set>
    
typedef Point<int> cell_t;
    
struct map_intersection_t
{
    Point<int> lhsStartCell;
    Point<int> rhsStartCell;
    int width;
    int height;
    
    // Assumes that there is some overlap between lhs and rhs
    map_intersection_t(const OccupancyGrid& lhs, const OccupancyGrid& rhs);
};

inline bool is_free_cell(int x, int y, const OccupancyGrid& grid) { return grid.logOdds(x, y) < 0; }
inline bool is_occupied_cell(int x, int y, const OccupancyGrid& grid) { return grid.logOdds(x, y) > 0; }

double cell_value(int x, int y, const OccupancyGrid& grid);
bool is_boundary_cell(int x, int y, const OccupancyGrid& grid);


double map_correlation(const OccupancyGrid& lhs, const OccupancyGrid& rhs)
{
    map_intersection_t intersection(lhs, rhs);
    
    // Calculate the mean cell value
    double meanCellValue = 0.0;
    
    for(int y = 0; y < intersection.height; ++y)
    {
        for(int x = 0; x < intersection.width; ++x)
        {
            meanCellValue += cell_value(intersection.lhsStartCell.x + x, intersection.lhsStartCell.y + y, lhs);
            meanCellValue += cell_value(intersection.rhsStartCell.x + x, intersection.rhsStartCell.y + y, rhs);
        }
    }
    
    meanCellValue /= 2.0 * intersection.height * intersection.width;
    
    // Calculate the correlation value
    double numerator = 0.0;
    double sumSqLhs = 0.0;
    double sumSqRhs = 0.0;
    
    for(int y = 0; y < intersection.height; ++y)
    {
        for(int x = 0; x < intersection.width; ++x)
        {
            double lhsValue = cell_value(intersection.lhsStartCell.x + x, intersection.lhsStartCell.y + y, lhs);
            double rhsValue = cell_value(intersection.rhsStartCell.x + x, intersection.rhsStartCell.y + y, rhs);
            
            numerator += (lhsValue - meanCellValue) * (rhsValue - meanCellValue);
            sumSqLhs += std::pow(lhsValue - meanCellValue, 2.0);
            sumSqRhs += std::pow(rhsValue - meanCellValue, 2.0);
        }
    }
    
    double denominator = std::sqrt(sumSqLhs * sumSqRhs);
    
    // If both are exactly the mean cell, then exactly correlation -- shouldn't happen
    if(denominator == 0.0)
    {
        return 1.0;
    }
    else
    {
        return numerator / denominator;
    }
}


double amount_explored(const OccupancyGrid& grid, const pose_xyt_t& robotPose)
{
    // Run a simple breadth-first search from the robot pose to see how much of the map has been explored
    std::set<cell_t> visited;
    std::queue<cell_t> searchQueue;
    
    searchQueue.push(global_position_to_grid_cell(Point<double>(robotPose.x, robotPose.y), grid));
    visited.insert(global_position_to_grid_cell(Point<double>(robotPose.x, robotPose.y), grid));
    
    double occupiedBoundaryLength = 0.0;
    double frontierBoundaryLength = 0.0;
    
    while(!searchQueue.empty())
    {
        cell_t next = searchQueue.front();
        searchQueue.pop();
        
        // If a boundary, it must either be free or occupied
        if(is_boundary_cell(next.x, next.y, grid))
        {
            if(is_occupied_cell(next.x, next.y, grid))
            {
                occupiedBoundaryLength += 1.0;
            }
            else
            {
                frontierBoundaryLength += 1.0;
            }
        }
        // Otherwise expand all free cells in four-way fashion
        else if(is_free_cell(next.x, next.y, grid))
        {
            cell_t adjacent(next.x + 1, next.y);
            if(visited.find(adjacent) == visited.end())
            {
                searchQueue.push(adjacent);
                visited.insert(adjacent);
            }
            
            adjacent = cell_t(next.x - 1, next.y);
            if(visited.find(adjacent) == visited.end())
            {
                searchQueue.push(adjacent);
                visited.insert(adjacent);
            }
            
            adjacent = cell_t(next.x, next.y + 1);
            if(visited.find(adjacent) == visited.end())
            {
                searchQueue.push(adjacent);
                visited.insert(adjacent);
            }
            
            adjacent = cell_t(next.x, next.y - 1);
            if(visited.find(adjacent) == visited.end())
            {
                searchQueue.push(adjacent);
                visited.insert(adjacent);
            }
        }
    }
    
    // If there is currently no boundary ?!? then nothing is explored
    if(occupiedBoundaryLength + frontierBoundaryLength == 0.0)
    {
        return 0.0;
    }
    
    return occupiedBoundaryLength / (occupiedBoundaryLength + frontierBoundaryLength);
}


map_intersection_t::map_intersection_t(const OccupancyGrid& lhs, const OccupancyGrid& rhs)
{
    Point<double> lhsTopRight(lhs.originInGlobalFrame().x + lhs.widthInMeters(),
                              lhs.originInGlobalFrame().y + lhs.heightInMeters());
    Point<double> rhsTopRight(rhs.originInGlobalFrame().x + rhs.widthInMeters(),
                              rhs.originInGlobalFrame().y + rhs.heightInMeters());
    
    Point<double> origin(std::max(lhs.originInGlobalFrame().x, rhs.originInGlobalFrame().x),
                         std::max(lhs.originInGlobalFrame().y, rhs.originInGlobalFrame().y));
    Point<double> topRight(std::max(lhsTopRight.x, rhsTopRight.x),
                           std::max(lhsTopRight.y, rhsTopRight.y));
    
    lhsStartCell = global_position_to_grid_cell(origin, lhs);
    rhsStartCell = global_position_to_grid_cell(origin, rhs);
    
    width = std::min(lhs.widthInCells() - lhsStartCell.x, rhs.widthInCells() - rhsStartCell.x);
    height = std::min(lhs.heightInCells() - lhsStartCell.y, rhs.heightInCells() - rhsStartCell.y);
}


double cell_value(int x, int y, const OccupancyGrid& grid)
{
    if(grid(x, y) > 0)
    {
        return 1.0;
    }
    else if(grid(x, y) < 0)
    {
        return -1.0;
    }
    else // unknown cell
    {
        return 0.0;
    }
}


bool is_boundary_cell(int x, int y, const OccupancyGrid& grid)
{
    // A free cell can't be a boundary
    if(is_free_cell(x, y, grid))
    {
        return false;
    }
    
    // Check if any four-way connected neighbors are free. If so, then this cell is occupied/unknown and neighbor is
    // free which is definition of a boundary
    return is_free_cell(x + 1, y, grid)
        || is_free_cell(x - 1, y, grid)
        || is_free_cell(x, y + 1, grid)
        || is_free_cell(x, y - 1, grid);
}
