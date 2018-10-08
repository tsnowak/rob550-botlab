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


void draw_robot(const pose_xyt_t& pose, const float* color, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw robot at the pose using vxo_robot ////////////////////////////
    
    vx_object_t* robot = vxo_chain(vxo_mat_translate2(pose.x,pose.y),
                                vxo_mat_rotate_z(pose.theta),
                                vxo_mat_scale(PROJ_SCALE),
                                vxo_robot(vxo_mesh_style(vx_green)));
    
    vx_buffer_add_back(buffer, robot);

}

void draw_pose_trace(const PoseTrace& poses, const float* color, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw PoseTrace as line segments connecting consecutive poses ////////////////////////////

    //poses.at(i).x is value of the x term of the ith member of the vector poses
    //printf("Nyaa \n");
    
    float points[2*poses.size()];
    int npoints = poses.size();
    std::vector< float > arr;

    for(int i = 0; i < poses.size(); i++)
    {
        arr.push_back(poses.at(i).x);
        arr.push_back(poses.at(i).y);
        //printf("%d: %f \n",i,poses.at(i).x);
        /*vx_object_t* trace = vxo_chain(vxo_mat_translate2(poses.at(i).x,poses.at(i).y),
                                vxo_mat_rotate_z(poses.at(i).theta),
                                vxo_mat_scale(PROJ_SCALE),
                                vxo_arrow(vxo_mesh_style(vx_green)));
        vx_buffer_add_back(buffer, trace);*/
    }
    std::copy(arr.begin(), arr.end(), points);
    vx_resc_t *verts = vx_resc_copyf(points, npoints*2);
    vx_buffer_add_back(buffer, vxo_lines(verts, npoints, GL_LINES, vxo_points_style(vx_green, 2.0f)));
}


void draw_laser_scan(const rplidar_laser_t& laser, const pose_xyt_t& pose, const float* color, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw rplidar_laser_t as specified in assignment ////////////////////////////
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
            img->buf[(i*img->stride)+j] = -grid.logOdds(j,i)+127;
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
}


void draw_path(const robot_path_t& path, const float* color, vx_buffer_t* buffer)
{
    ////////////////// TODO: Draw robot_path_t as specified in assignment ////////////////////////////
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
}

/*typedef struct
{
    vx_object_t * obj;
    char * name;
} obj_data_t;


typedef struct
{
    zarray_t * obj_data;
    vx_world_t * world;

    vx_event_handler_t  vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;

    vx_application_t vxapp;

    //data to place dots at mouse click locations
    vx_object_t* points[MAX_NUM_PTS];
    int next_point;
    int num_points;

    double box_coords[3];  // (x,y,theta) **x is forward in odometry!

    //data for arrows to be placed along maebots path
    vx_object_t* path_points[PATH_SIZE];
    int next_path;
    int num_path;


    pthread_t path_thread;

    int map[PROJ_HEIGHT][PROJ_LENGTH];
    int origin;

    
} state_t;


static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state_t *state = (state_t *) vxeh->impl;

    if ((mouse->button_mask & VX_BUTTON1_MASK) &&
        !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {

        vx_ray3_t ray;
        vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

        double ground[3];
        vx_ray3_intersect_xy (&ray, 0, ground);

        //printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n",
                //mouse->x, mouse->y, ground[0], ground[1]);

        vx_buffer_t * vb = vx_world_get_buffer(state->world, "waypoints");

        vx_object_t* pt = vxo_chain(vxo_mat_translate2(ground[0],ground[1]),
                                    vxo_mat_scale(0.1),
                                    vxo_sphere(vxo_mesh_style(vx_red)));

        if(state->num_points % MAX_NUM_PTS == 0){
            state->next_point = 0;
        }

        ++state->num_points;
        state->points[state->next_point++] = pt;


        for(int i = 0; i < fmin(state->num_points,MAX_NUM_PTS); ++i){
            vx_buffer_add_back(vb,state->points[i]);
        }

        vx_buffer_swap(vb);

        printf("point placed at coords[%6.3f, %6.3f]\n", ground[0], ground[1]);
    }

    state->last_mouse_event = *mouse;

    return 0;
}


static int key_event (vx_event_handler_t* vxeh, vx_layer_t* vl, vx_key_event_t* key)
{
    state_t* state = (state_t *) vxeh->impl;

    vx_buffer_t * vb = vx_world_get_buffer(state->world, "box_buffer");

    if(!key->released){
        if(key->key_code == 'w' || key->key_code == 'W' || key->key_code == VX_KEY_UP) {
            state->box_coords[0] += .1*cos(state->box_coords[2]);
            state->box_coords[1] += .1*sin(state->box_coords[2]);
        }
        else if(key->key_code == 'a' || key->key_code == 'A' || key->key_code == VX_KEY_LEFT){
            state->box_coords[2] += .1;
        }   
        else if(key->key_code == 's' || key->key_code == 'S' || key->key_code == VX_KEY_DOWN){
            state->box_coords[0] -= .1*cos(state->box_coords[2]);
            state->box_coords[1] -= .1*sin(state->box_coords[2]);
        }
        else if(key->key_code == 'd' || key->key_code == 'D' || key->key_code == VX_KEY_RIGHT){
            state->box_coords[2] -= .1;
        }
    }
    
    vx_object_t * vo_box = vxo_chain(vxo_box(vxo_mesh_style(vx_green)));
    vx_buffer_add_back(vb, vxo_chain(vxo_mat_translate2(state->box_coords[0], state->box_coords[1]), 
                                     vxo_mat_rotate_z(state->box_coords[2]), 
                                     vxo_mat_scale(0.8),
                                     vo_box));
    vx_buffer_swap(vb);

    return 0;
}


void* path_loop(void* data)
{
    state_t* state = (state_t *) data;
    while(1){
        vx_buffer_t* vb =  vx_world_get_buffer(state->world,"path");
        if(state->num_path%PATH_SIZE == 0){
            state->next_path = 0;
        }
        
        ++state->num_path;
             
        vx_object_t* vo = vxo_chain(vxo_mat_translate2(state->box_coords[0],state->box_coords[1]),
                                    vxo_mat_rotate_z(state->box_coords[2]),
                                    vxo_mat_scale(.5),
                                    vxo_arrow(vxo_mesh_style(vx_red)));
        state->path_points[state->next_path++] = vo;

        for(int i = 0; i < fmin(state->num_path,PATH_SIZE); ++i){
            vx_buffer_add_back(vb,state->path_points[i]);
        }

        //printf("next path = %d, path_points = %d\n", state->next_path, state->num_path);

        vx_buffer_swap(vb);
        usleep(1000000);

    }
}


static void draw(state_t* state, vx_world_t * world, zarray_t * obj_data);
static void display_finished(vx_application_t * app, vx_display_t * disp);
static void display_started(vx_application_t * app, vx_display_t * disp);


int main(int argc, char **argv)
{
    vx_global_init();

    state_t* state = (state_t *) calloc(1,sizeof(state_t));
    state->world = vx_world_create();
    state->obj_data = zarray_create(sizeof(obj_data_t));
    state->vxapp.display_finished = display_finished;
    state->vxapp.display_started = display_started;
    state->vxapp.impl = state;
    state->vxeh.mouse_event = mouse_event;
    state->vxeh.key_event = key_event;
    state->vxeh.impl = state;

    state->box_coords[0] = 0;
    state->box_coords[1] = 0;
    state->box_coords[2] = M_PI/2;

    pthread_create(&state->path_thread, NULL, path_loop, state);

    gdk_threads_init ();
    gdk_threads_enter ();

    gtk_init (&argc, &argv);

    vx_gtk_display_source_t * appwrap = vx_gtk_display_source_create(&state->vxapp);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), 1024, 768);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!

    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_main (); // Blocks as long as GTK window is open
    gdk_threads_leave ();

 
    vx_gtk_display_source_destroy(appwrap);
 
    vx_global_destroy();
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    
    state_t* state = (state_t *) app->impl;
    pthread_cancel(state->path_thread);
    free(state);
}*/

/*static void display_started(vx_application_t * app, vx_display_t * disp)
{
    state_t* state = (state_t *) app->impl;

    vx_layer_t* layer = vx_layer_create(state->world);
    vx_layer_set_display(layer, disp);
    
    vx_layer_camera_op (layer, OP_PROJ_ORTHO);
    
    float eye[3]    = {  0,  0,  9 };
    float lookat[3] = {  0,  0,  0 };
    float up[3]     = {  0,  1,  0 };
    vx_layer_camera_lookat (layer, eye, lookat, up, 1);
    
    float xy0[] = {-PROJ_LENGTH*5.0*PROJ_SCALE+.5,0.5};
    float xy1[] = {PROJ_LENGTH*5.0*PROJ_SCALE - .5,PROJ_HEIGHT*10.0*PROJ_SCALE -.5};
    vx_layer_camera_fit2D(layer, xy0, xy1, 0);

    vx_layer_set_background_color(layer,vx_black);
    vx_layer_add_event_handler(layer, &state->vxeh);

    draw(state, state->world, state->obj_data);
}*/
/*
static void draw(state_t* state, vx_world_t * world, zarray_t * obj_data)
{
    vx_buffer_t * vb = vx_world_get_buffer(world, "border_spheres");

    vx_object_t * vo_sphere = vxo_chain(vxo_sphere(vxo_mesh_style(vx_white)));

    vx_buffer_add_back(vb,vxo_chain(vxo_mat_translate2(0,0),vxo_mat_scale(0.1),vo_sphere));

    vx_buffer_add_back(vb,vxo_chain(vxo_mat_translate2(PROJ_LENGTH*5.0*PROJ_SCALE,0.0),vxo_mat_scale(0.1),vo_sphere));
    vx_buffer_add_back(vb,vxo_chain(vxo_mat_translate2(PROJ_LENGTH*5.0*PROJ_SCALE,PROJ_HEIGHT*10.0*PROJ_SCALE),vxo_mat_scale(0.1),vo_sphere));
    vx_buffer_add_back(vb,vxo_chain(vxo_mat_translate2(-PROJ_LENGTH*5.0*PROJ_SCALE,PROJ_HEIGHT*10.0*PROJ_SCALE),vxo_mat_scale(0.1),vo_sphere));
    vx_buffer_add_back(vb,vxo_chain(vxo_mat_translate2(-PROJ_LENGTH*5.0*PROJ_SCALE,0.0),vxo_mat_scale(0.1),vo_sphere));

    vx_object_t * vo_box = vxo_chain(vxo_box(vxo_mesh_style(vx_green)));

    vx_buffer_t * box_buffer = vx_world_get_buffer(world, "box_buffer");
    vx_buffer_add_back(box_buffer, vxo_chain(vxo_mat_translate2(0,0),  vxo_mat_scale(0.8), vo_box));
    vx_buffer_swap(box_buffer);
    vx_buffer_swap(vb);
}*/
