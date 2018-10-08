#ifndef APPS_SLAM_GRID_VISUALIZATION_HPP
#define APPS_SLAM_GRID_VISUALIZATION_HPP

#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/point.hpp>
#include <common/pose_trace.hpp>
#include <vx/vx_display.h>
#include <vx/vxo_image.h>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <vector>

/**
* GridVisualization
*/
class GridVisualization
{
public:

    /**
    * Constructor for GridVisualization.
    * 
    * \param    title           Tile described the state being drawn
    * \param    poseColor       Color to draw the poses
    */
    GridVisualization(std::string title, const float poseColor[4]);

    /**
    * Destructor for GridVisualization.
    */
    ~GridVisualization(void);

    // No moving or copying allowed
    GridVisualization(const GridVisualization& rhs) = delete;
    GridVisualization(GridVisualization&& rhs) = delete;

    GridVisualization& operator=(const GridVisualization& rhs) = delete;
    GridVisualization& operator=(GridVisualization&& rhs) = delete;

    /**
    * initializeDisplay sets the display to be used for on which to draw the visualization.
    */
    void initializeDisplay(vx_display_t* display, bool isLeft);

    /**
    * render draws the current state to the screen.
    */
    void render(void);

    /**
    * reset resets all internal state.
    */
    void reset(void);

    // LCM handlers
    void handleOccupancyGrid(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map);
    void handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose);

private:

    std::string title_;
    vx_world_t* world_;
    vx_layer_t* layer_;
    float color_[4];
    OccupancyGrid grid_;
    PoseTrace poses_;

    std::mutex dataLock_;
};

#endif // APPS_SLAM_GRID_VISUALIZATION_HPP
