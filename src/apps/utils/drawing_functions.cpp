#include <gtk/gtk.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <typeinfo>
#include <iostream>
#include <vector>
using namespace std;
#include <time.h>
#include "vx/vxo_drawables.h"
#include "vx/gtk/vx_gtk_display_source.h"
// core api
#include "vx/vx_global.h"
#include "vx/vx_layer.h"
#include "vx/vx_world.h"
#include "vx/vx_colors.h"


#define PROJ_HEIGHT 200
#define PROJ_LENGTH 200
#define PROJ_SCALE .05
#define MAX_NUM_PTS 3
#define PATH_SIZE 10

#include <apps/utils/drawing_functions.hpp>
#include <common/pose_trace.hpp>
#include <lcmtypes/particles_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/rplidar_laser_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <planning/frontiers.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <imagesource/image_u8.h>
#include <common/angle_functions.hpp>


void draw_robot(const pose_xyt_t& pose, const float* color, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw robot at the pose using vxo_robot ////////////////////////////
    
    vx_object_t* robot = vxo_chain(vxo_mat_translate2(pose.x,pose.y),
                                vxo_mat_rotate_z(pose.theta),
                                vxo_mat_scale(PROJ_SCALE),
                                vxo_robot(vxo_mesh_style(color)));
    
    vx_buffer_add_back(buffer, robot);

}

void draw_pose_trace(const PoseTrace& poses, const float* color, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw PoseTrace as line segments connecting consecutive poses ////////////////////////////

    //poses.at(i).x is value of the x term of the ith member of the vector poses
    //printf("Nyaa \n");
    
    float points1[2*poses.size()];
    int npoints1 = poses.size();
    std::vector< float > arr1;
    for(unsigned int i = 0; i < poses.size(); i++)
    {
        arr1.push_back(poses.at(i).x);
        arr1.push_back(poses.at(i).y);
    }
     std::copy(arr1.begin(), arr1.end(), points1);
    vx_resc_t *verts1 = vx_resc_copyf(points1, npoints1*2);
    vx_buffer_add_back(buffer, vxo_lines(verts1, npoints1, GL_LINES, vxo_points_style(color, 2.0f)));

    float points2[2*(poses.size()-1)];
    int npoints2 = poses.size()-1;
    std::vector< float > arr2;
    for(unsigned int i = 1; i < poses.size(); i++)
    {
        arr2.push_back(poses.at(i).x);
        arr2.push_back(poses.at(i).y);
    }
    std::copy(arr2.begin(), arr2.end(), points2);
    vx_resc_t *verts2 = vx_resc_copyf(points2, npoints2*2);
    vx_buffer_add_back(buffer, vxo_lines(verts2, npoints2, GL_LINES, vxo_points_style(color, 2.0f)));
}


void draw_laser_scan(const rplidar_laser_t& laser, const pose_xyt_t& pose, const float* color, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw rplidar_laser_t as specified in assignment ////////////////////////////
    //printf("NYAA\n");
    for(int i = 0; i < laser.num_ranges; i=i+4)
    {
        //printf("Range: %f, Theta: %f\n",laser.ranges[i], laser.thetas[i]);
        float points[4] = {pose.x, pose.y, (pose.x + (laser.ranges[i])*cos(wrap_to_pi(pose.theta
         - laser.thetas[i]))), (pose.y + (laser.ranges[i])*sin(wrap_to_pi(pose.theta - laser.thetas[i])))};
        int npoints = 2;
        vx_resc_t *verts = vx_resc_copyf(points, npoints*2);
        vx_buffer_add_back(buffer, vxo_lines(verts, npoints, GL_LINES, vxo_points_style(color, 1.0f)));
    }
        
}


void draw_occupancy_grid(const OccupancyGrid& grid, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw OccupancyGrid as specified in assignment ////////////////////////////
    //printf("%d\n",grid.heightInCells());
    
    
    image_u8_t *img = image_u8_create (grid.widthInCells(),grid.heightInCells()); 
    //image_u8 im = *img;

    for(int i = 0; i < grid.heightInCells(); ++i)
    {
        for(int j = 0; j < grid.widthInCells(); ++j)
        {
            if (grid.logOdds(j,i) < 0)
                img->buf[(i*img->stride)+j] = 254;
            else if (grid.logOdds(j,i) == 0)
                img->buf[(i*img->stride)+j] = 127;
            else
                img->buf[(i*img->stride)+j] = 0;
            //printf("%d    ",grid.logOdds(j,i));    
        }
    }
    //printf("\n Nyaa \n"); 
    vx_object_t* image = vxo_chain(vxo_mat_translate2(-100*PROJ_SCALE,-100*PROJ_SCALE),
                         vxo_mat_scale(PROJ_SCALE),
                         vxo_image_from_u8(img,VXO_IMAGE_NOFLAGS,VXO_IMAGE_NOFLAGS));
    vx_buffer_add_back(buffer, image);

}


void draw_particles(const particles_t& particles, vx_buffer_t* buffer)
{
    ////////////////// OPTIONAL: Draw particles using any of the mentioned approaches ////////////////////////////
    //printf("%d\n",particles.num_particles);
    
    for(int i = 0; i < particles.num_particles; i=i+1)
    {
        //printf("%d: %f\n",i,particles.particles[i].weight);

        if (particles.particles[i].weight<0.25)
        {
            vx_object_t* particle = vxo_chain(vxo_mat_translate2(particles.particles[i].pose.x,particles.particles[i].pose.y),
                                vxo_mat_rotate_z(particles.particles[i].pose.theta),
                                vxo_mat_scale(PROJ_SCALE/5),
                                vxo_arrow(vxo_mesh_style(vx_red)));
            vx_buffer_add_back(buffer, particle);
        }
        else if ((particles.particles[i].weight>0.25)&&(particles.particles[i].weight<0.5))
        {
            vx_object_t* particle = vxo_chain(vxo_mat_translate2(particles.particles[i].pose.x,particles.particles[i].pose.y),
                                vxo_mat_rotate_z(particles.particles[i].pose.theta),
                                vxo_mat_scale(PROJ_SCALE/3),
                                vxo_arrow(vxo_mesh_style(vx_yellow)));
            vx_buffer_add_back(buffer, particle);
        }
        else
        {
            vx_object_t* particle = vxo_chain(vxo_mat_translate2(particles.particles[i].pose.x,particles.particles[i].pose.y),
                                vxo_mat_rotate_z(particles.particles[i].pose.theta),
                                vxo_mat_scale(PROJ_SCALE),
                                vxo_arrow(vxo_mesh_style(vx_blue)));
            vx_buffer_add_back(buffer, particle);
        }
        
    }
}


void draw_path(const robot_path_t& path, const float* color, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw robot_path_t as specified in assignment ////////////////////////////
    float points1[2*path.path_length];
    int npoints1 = path.path_length;
    std::vector< float > arr1;
    for(int i = 0; i < path.path_length; i++)
    {
        arr1.push_back(path.path[i].x);
        arr1.push_back(path.path[i].y);
    }
    std::copy(arr1.begin(), arr1.end(), points1);
    vx_resc_t *verts1 = vx_resc_copyf(points1, npoints1*2);
    vx_buffer_add_back(buffer, vxo_lines(verts1, npoints1, GL_LINES, vxo_points_style(color, 2.0f)));

    float points2[2*(path.path_length)];
    int npoints2 = path.path_length;
    std::vector< float > arr2;
    for(int i = 1; i < path.path_length; i++)
    {
        arr2.push_back(path.path[i].x);
        arr2.push_back(path.path[i].y);
    }
    std::copy(arr2.begin(), arr2.end(), points2);
    vx_resc_t *verts2 = vx_resc_copyf(points2, npoints2*2);
    vx_buffer_add_back(buffer, vxo_lines(verts2, npoints2, GL_LINES, vxo_points_style(color, 2.0f)));

    for(int i = 0; i < path.path_length; i++)
    {
        vx_object_t* path1 = vxo_chain(vxo_mat_translate2(path.path[i].x,path.path[i].y),
                            vxo_mat_rotate_z(path.path[i].theta),
                            vxo_mat_scale(PROJ_SCALE/5),
                            vxo_box(vxo_mesh_style(color)));
        vx_buffer_add_back(buffer, path1);
    }
}


void draw_distance_grid(const ObstacleDistanceGrid& grid, float cspaceDistance, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw ObstacleDistanceGrid as specified in assignment ////////////////////////////
    //////// We recommend using image_u32 to represent the different colors /////////////////////////////////
}


void draw_frontiers(const std::vector<frontier_t>& frontiers, 
                    double metersPerCell, 
                    const float* color, 
                    vx_buffer_t* buffer)
{
    //////////////////// TODO: Draw the frontiers using one box for each cell located along a frontier ////////////////
    Point<float> box_position;
    
    for(unsigned int g = 0; g < frontiers.size(); g++)
    {
        int size_cells = frontiers[g].cells.size();  //Size of cells in each frontier
        for(int h = 0; h < size_cells; h++)
        {
            box_position = frontiers[g].cells[h];
            vx_object_t *box = vxo_chain(vxo_mat_translate2(box_position.x + PROJ_SCALE/2, box_position.y + PROJ_SCALE/2), 
                               vxo_mat_scale(PROJ_SCALE),
                               vxo_rect(vxo_mesh_style(color)));
            vx_buffer_add_back(buffer, box);
        }
    }
}