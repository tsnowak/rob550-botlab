#include <apps/challenge/grid_visualization.hpp>
#include <apps/utils/drawing_functions.hpp>
#include <common/grid_utils.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <vx/vx_layer.h>
#include <vx/vx_world.h>
#include <vx/vx_colors.h>
#include <vx/vxo_chain.h>
#include <vx/vxo_lines.h>
#include <vx/vxo_mat.h>
#include <vx/vxo_mesh.h>
#include <vx/vxo_robot.h>
#include <vx/vxo_pix_coords.h>
#include <vx/vxo_text.h>

void fit_camera_to_occupied_space(const OccupancyGrid& grid, vx_layer_t* layer);
    

GridVisualization::GridVisualization(std::string title, const float poseColor[4])
: title_(title)
, world_(0)
, layer_(0)
{
    std::copy(poseColor, poseColor + 4, color_);
}


GridVisualization::~GridVisualization(void)
{
    if(layer_)
    {
        vx_layer_destroy(layer_);
    }
    
    if(world_)
    {
        vx_world_destroy(world_);
    }
}


void GridVisualization::initializeDisplay(vx_display_t* display, bool isLeft)
{
    world_ = vx_world_create();
    layer_ = vx_layer_create(world_);

    vx_layer_set_background_color(layer_, vx_white);
    vx_layer_set_display(layer_, display);

    float viewport[4] = { (isLeft ? 0.0f : 0.5f), 0.0f, 0.5f, 1.0f };
    vx_layer_set_viewport_rel(layer_, viewport);

    vx_layer_camera_op (layer_, OP_PROJ_PERSPECTIVE);
    float eye[3]    = {  0,  0,  1 };
    float lookat[3] = {  0,  0,  0 };
    float up[3]     = {  0,  1,  0 };
    vx_layer_camera_lookat (layer_, eye, lookat, up, 1);

    vx_code_output_stream_t *couts = vx_code_output_stream_create (128);
    couts->write_uint32 (couts, OP_LAYER_CAMERA);
    couts->write_uint32 (couts, vx_layer_id (layer_));
    couts->write_uint32 (couts, OP_INTERFACE_MODE);
    couts->write_float  (couts, 2.5f);
    display->send_codes (display, couts->data, couts->pos);
    vx_code_output_stream_destroy (couts);
    
    // Title only needs to be drawn once
    vx_buffer_t* titleBuf = vx_world_get_buffer(world_, "title");
    char titleString[1001];
    snprintf(titleString, 1000, "<<left,#0000CD>>%s", title_.c_str());
    
    vx_object_t* titleText = vxo_text_create(VXO_TEXT_ANCHOR_TOP, titleString);
    vx_buffer_add_back(titleBuf, vxo_pix_coords(VX_ORIGIN_TOP, titleText));
    vx_buffer_swap(titleBuf);
}


void GridVisualization::render(void)
{
    std::lock_guard<std::mutex> lock(dataLock_);

    assert(world_ && layer_);
    
    vx_buffer_t* gridBuffer = vx_world_get_buffer(world_, "grid");
    draw_occupancy_grid(grid_, gridBuffer);
    
    pose_xyt_t currentPose;
    currentPose.x = 0.0f;
    currentPose.y = 0.0f;
    currentPose.theta = 0.0f;
    
    vx_buffer_t* poseBuffer = vx_world_get_buffer(world_, "poses");
    
    if(!poses_.empty())
    {
        draw_pose_trace(poses_, color_, poseBuffer);
        
        currentPose = poses_.back();
        draw_robot(poses_.back(), color_, poseBuffer);
    }

    char poseString[1001];
    snprintf(poseString, 1000, "<<left,#0000CD>>Pose:(%0 .2f,%0 .2f,%0 .2f)", currentPose.x, currentPose.y, currentPose.theta);

    vx_object_t* poseText = vxo_text_create(VXO_TEXT_ANCHOR_BOTTOM_LEFT, poseString);
    vx_buffer_add_back(poseBuffer, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, poseText));

    vx_buffer_swap(gridBuffer);
    vx_buffer_swap(poseBuffer);
}


void GridVisualization::reset(void)
{
    std::lock_guard<std::mutex> lock(dataLock_);
    poses_.clear();
}


void GridVisualization::handleOccupancyGrid(const lcm::ReceiveBuffer* rbuf,
                                            const std::string& channel,
                                            const occupancy_grid_t* map)
{
    std::lock_guard<std::mutex> lock(dataLock_);
    grid_.fromLCM(*map);
    
    // Auto-zoom the camera to show all occupied cells in the map, plus a little
    if(layer_)
    {
        fit_camera_to_occupied_space(grid_, layer_);
    }
}


void GridVisualization::handlePose(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& channel,
                                   const pose_xyt_t* pose)
{
    std::lock_guard<std::mutex> lock(dataLock_);
    poses_.addPose(*pose);
}


void fit_camera_to_occupied_space(const OccupancyGrid& grid, vx_layer_t* layer)
{
    // Search through the grid and create an axis-aligned bounding rectangle around the occupied cells in the grid
    Point<int> bottomLeft(10000, 10000);
    Point<int> topRight(-10000, -10000);
    
    for(int y = 0; y < grid.heightInCells(); ++y)
    {
        for(int x = 0; x < grid.widthInCells(); ++x)
        {
            if(grid(x, y) > 0)
            {
                bottomLeft.x = std::min(bottomLeft.x, x);
                bottomLeft.y = std::min(bottomLeft.y, y);
                
                topRight.x = std::max(topRight.x, x);
                topRight.y = std::max(topRight.y, y);
            }
        }
    }
    
    Point<float> globalBottomLeft = grid_position_to_global_position(bottomLeft, grid);
    Point<float> globalTopRight = grid_position_to_global_position(topRight, grid);
    
    // Expand the boundary a little outside the desired region so the walls don't slam against the edge of the grid
    float bl[2] = { globalBottomLeft.x - 0.5f, globalBottomLeft.y - 0.5f };
    float tr[2] = { globalTopRight.x + 0.5f, globalTopRight.y + 0.5f };
    
    vx_layer_camera_fit2D(layer, bl, tr, 1);
}
