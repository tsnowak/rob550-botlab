#include <maebot/maebot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/maebot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <maebot/controller_types.h>
#include <math.h>


class MotionController
{
public:
    
    /**
    * Constructor for MotionController.
    */
    MotionController(void)
    {
        printf("Initializing Values...\n");

	arrived = false;

        // initialize pose differences to 0
        delta_x = 0;
        delta_y = 0;
        delta_theta = 0;

        // initialize all pose modalities to 0
        estimated_pose.utime = utime_now();
        estimated_pose.x = 0;
        estimated_pose.y = 0;
        estimated_pose.theta = 0;

        previous_pose.utime = utime_now();
        previous_pose.x = 0;
        previous_pose.y = 0;
        previous_pose.theta = 0;

        odom.x = 0;
        odom.y = 0;
        odom.theta = 0;

        slam_pose.utime = utime_now();        // init'd time to 0
        slam_pose.x = 0;
        slam_pose.y = 0;
        slam_pose.theta = 0; 

        // to start, set path to origin
        start_pose.x = 0;
        start_pose.y = 0;
        start_pose.theta = 0;

        current_path.x = 0;
        current_path.y = 0;
        current_path.theta = 0;

        // reset counter to 0
        path_counter = 0;
        // start assuming no new slam_pose
        new_slam_pose = false;

        ori_err = 0;
        nav_stack.path_length = 0;

        printf("Initialization Complete.\n");
    }
    
    /**
    * updateCommand calculates the new motor command to send to the Maebot.
    * This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command,
    * or if the previous command should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the maebot_driver.
    */
    maebot_motor_command_t updateCommand(void)
    {
        // U(t) = Up(t) + Ud(t) + Ui(t)
        // Where xd is x desired and x is actual
        // Up(t) = kp(xd - x)
        // Ud(t) = kd(dxd(t) - dx(t))
        // dx = x(t)-x(t-1)/dt
        // Ui(t) = ki(integral(e(t)dt))
        // where e(t) is the error between current location and set point

        maebot_motor_command_t cmd;
        cmd.utime = utime_now();
        cmd.left_motor_enabled = 1;     // enable motors
        cmd.right_motor_enabled = 1;
        cmd.left_motor_speed = 0.0f;
        cmd.right_motor_speed = 0.0f;
            
        // see if a SLAM pose is in, if so, adjust estimated pose
        adjustEstimates();    

        // have we received a path?
        if (nav_stack.path_length == 0)
        {
            cmd.left_motor_speed = 0.0f;
            cmd.right_motor_speed = 0.0f;
        } 
        // are we at goal position?
        else if (path_vector.mag <= endgoal_error_tol)
        {
            // do we need to be in the end orientation?
            if (orientation_fit_flag)
            {
                // are we at the end orientation?
                if (orientation_diff <= endgoal_ori_tol)
                {
                    // if this was the last path leg, we're done
                    if (path_counter == (nav_stack.path_length -1))
                    {
                        cmd.left_motor_speed = 0.0f;
                        cmd.right_motor_speed = 0.0f;
                        arrived = true;
                    }
                    // if there is another leg, initialize it
                    else
                    {
                        path_counter++;
                        initializePath();
                    }
                }
                // spin more to get to the right orientation
                else
                {
                    ori_cmd = restingOrientationHandler();
                    // CALCULATE MOTOR_SPEEDS FROM ORIENTATION PID OUTPUT
                    restingMotorCommand(ori_cmd, &cmd.left_motor_speed, &cmd.right_motor_speed);
                }
            }
            // don't match the orientation
            else
            {
                // if this was the last path leg, we're done
                if (path_counter == (nav_stack.path_length - 1))
                {
                    cmd.left_motor_speed = 0.0f;
                    cmd.right_motor_speed = 0.0f;
                    arrived = true;
                }
                // if there is another leg, initialize it
                else
                {
                    // go to next pose once we reach desired pose
                    path_counter++;
                    initializePath();
                }
            }
        }
        else
        {
            vel_cmd = velocityHandler();
            ori_cmd = orientationHandler();        

            pathMotorCommand(ori_cmd, vel_cmd, &cmd.left_motor_speed, 
                &cmd.right_motor_speed); 
        }        

        previous_pose = estimated_pose;

        printf("Final Output Motor Command: L: %f\t R: %f\n",
            cmd.left_motor_speed, cmd.right_motor_speed);

        printf("\n");

        return cmd;
    }
    
    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    { 
        arrived = false;

        // time of path creation
        nav_stack.utime  = path->utime;

        // number of poses in the path
        nav_stack.path_length = path->path_length;

        // sequence of poses_xyt_t that make up the path
        nav_stack.path = path->path;

        // set current path to the first position in the nav_stack 
        current_path = nav_stack.path[0];
        initializePath();     
    }
    
    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        // time of odometry creation
        odom.utime = odometry->utime;

        // odometry pose orientation and position
        odom.x = odometry->x;
        odom.y = odometry->y;
        odom.theta = odometry->theta;

        // total distance each wheel has traveled, length of wheelbase
        odom.total_left_distance = odometry->total_left_distance;
        odom.total_right_distance = odometry->total_right_distance;
        odom.wheelbase = odometry->wheelbase;

        // speed of each wheel
        odom.left_wheel_speed = odometry->left_wheel_speed;
        odom.right_wheel_speed = odometry->right_wheel_speed;
    }
    
    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        if (pose->utime != slam_pose.utime)
        {
            new_slam_pose = true; 

            // time of pose creation
            slam_pose.utime = pose->utime;
            // pose orientation and position
            slam_pose.x = pose->x;
            slam_pose.y = pose->y;
            slam_pose.theta = pose->theta;
        }
    }

    bool arrived;
    
private:

    // if new SLAM pose is received, adjust estimated pose values
    void adjustEstimates(void)
    {
        if (new_slam_pose)
        {
            // difference between lidar and odometry
            delta_x = slam_pose.x - odom.x;
            delta_y = slam_pose.y - odom.y;
            delta_theta = slam_pose.theta - odom.theta;

            estimated_pose = slam_pose; 

            printf("SLAM Pose: t: %ld\tx: %f\ty: %f\tth: %f\n", 
                slam_pose.utime, slam_pose.x, slam_pose.y, slam_pose.theta);
        }
        else
        {
            // adjust odometry measurements with latest slam/odom discrepancies
            //estimated_pose.utime = odom.utime;
            estimated_pose.utime = utime_now();
            estimated_pose.x = odom.x + delta_x;
            estimated_pose.y = odom.y + delta_y;
            estimated_pose.theta = odom.theta + delta_theta;
            printf("Odom Pose: t: %ld\tx: %f\ty: %f\tth: %f\n", odom.utime, odom.x, odom.y, odom.theta);
        }

        // also calculate derivate variables
        dt = (estimated_pose.utime - previous_pose.utime); 
        dx = (estimated_pose.x - previous_pose.x)/dt; 
        dy = (estimated_pose.y - previous_pose.y)/dt; 
        vel = sqrt(pow(dx,2) + pow(dy,2));
        accel = (vel - prev_vel)/dt;  

        //printf("****************************************\n");
        //printf("EP.utime: %ld\tPP.utime: %ld\n", estimated_pose.utime, previous_pose.utime);
        //printf("dt: %ld\tdx: %f\tdy: %f\tvel: %f\taccel: %f\n", dt, dx, dy, vel, accel);
        //printf("****************************************\n");

        // account for singular to get minimum orientation distance
        dtheta = minimumDistance(estimated_pose.theta, previous_pose.theta);
        orientation_diff = minimumDistance(current_path.theta, estimated_pose.theta);

        // calculate path vector from where we are now
        path_vector.x = current_path.x - estimated_pose.x; 
        path_vector.y = current_path.y - estimated_pose.y;
        path_vector.mag = sqrt(pow(path_vector.x,2) + pow(path_vector.y,2));
        path_vector.ux = path_vector.x/path_vector.mag;
        path_vector.uy = path_vector.y/path_vector.mag;
        printf("Distance to Goal: %f\n", path_vector.mag);

        printf("Estimated Pose: t: %ld\tx: %f\ty: %f\tth: %f\n", 
            estimated_pose.utime, estimated_pose.x, estimated_pose.y, estimated_pose.theta);
    }    

    // called when a new path of poses is given
    void initializePath(void)
    {
        printf("Initializing Path: %i of %i\n", path_counter, nav_stack.path_length);

        current_path.x = nav_stack.path[path_counter].x;
        current_path.y = nav_stack.path[path_counter].y;
        current_path.theta = nav_stack.path[path_counter].theta;

        start_pose.x = estimated_pose.x;
        start_pose.y = estimated_pose.y;
        start_pose.theta = estimated_pose.theta;

        printf("Start Pose: x: %f\ty: %f\tth: %f\n", start_pose.x, start_pose.y, start_pose.theta);
        printf("End Pose: t: x: %f\ty: %f\tth: %f\n", current_path.x, current_path.y, current_path.theta);

        path_vector.mag = sqrt(pow(estimated_pose.x - current_path.x,2) + 
            pow(estimated_pose.y - current_path.y,2));

        printf("Distance: %f\n", path_vector.mag);
        ori_err = 0; 
    }

    // if we are approaching the end point of the last leg, ramp down
    float velocityHandler(void)
    {
        float p_vel, d_vel;
        float vel_des;
        float command_velocity;

        vel_des = max_vel; 
        // only implement proportional and derivative
        // vel_des = .01 to max_vel (.05)
        // vel = -inf to inf
        p_vel = kp_vel*(vel_des - vel);
        // max_accel = .01
        // accel = -inf to inf
        d_vel = kd_vel*(-1*accel);
        // i_vel = ki_vel*(vel_des - vel);

        command_velocity = p_vel + d_vel;

        printf("cmd_vel (us): %f", command_velocity);

        // saturation max value chosen arbitrarily
        command_velocity = saturateValue(command_velocity, -1, 1);

        printf("\tcmd_vel (s): %f\n", command_velocity);

        return command_velocity;
    }

    float orientationHandler(void)
    {
        float p_ori, d_ori;
        // float i_ori;
        float ori_des;
        float ori_diff;     // difference between orientation and estimated pose
        float command_orientation;

        ori_des = atan2(path_vector.y, path_vector.x);  

        // convert to 0 to 2PI
        if (ori_des < 0) ori_des = 2*PI + ori_des;

        printf("Orientation Desired: %f\tEstimated Orientation: %f\n", ori_des, estimated_pose.theta);

        ori_diff = minimumDistance(ori_des, estimated_pose.theta);
        printf("Orientation Difference: %f\n", ori_diff);

        // accumulate integral error
        //ori_err += ori_err + ori_diff;

        // ori_diff = ((-PI to PI) - (-PI to PI))
        // 0 to 2PI
        p_ori = kp_ori*(ori_diff);
        // max_dtheta = .174 deg/sec
        // dtheta = -inf to inf 
        d_ori = kd_ori*(-1*dtheta);
        // ori_err = -inf to inf
        //i_ori = ki_ori*(ori_err);

        command_orientation = p_ori + d_ori;

        printf("cmd_ori (us): %f", command_orientation);

        command_orientation = saturateValue(command_orientation, -1, 1);

        printf("\tcmd_ori (s): %f\n", command_orientation);

        return command_orientation;
    }

    // calculate orientation commands for staying in place
    float restingOrientationHandler(void)
    {
        float p_ori, d_ori, i_ori;
        float ori_des, ori_diff;
        float command_orientation;

        ori_des = current_path.theta;
 
        if (ori_des < 0) ori_des = 2*PI + ori_des;

        printf("Orientation Desired: %f\tEstimated Orientation: %f\n", ori_des, estimated_pose.theta);

        ori_diff = minimumDistance(ori_des, estimated_pose.theta);
        printf("Orientation Difference: %f\n", ori_diff);


        path_ori_err += path_ori_err + ori_diff;

        p_ori = kp_ori*(ori_diff);
        d_ori = kd_ori*(-1*dtheta);
        i_ori = ki_ori*(path_ori_err);

        command_orientation = p_ori + d_ori + i_ori;
        printf("cmd_ori_r (us): %f", command_orientation);

        // should be the same as above
        command_orientation = saturateValue(command_orientation, -1, 1);
        printf("\tcmd_ori_r (s): %f\n", command_orientation);

        return command_orientation;
    }

    float saturateValue(float value, float min, float max)
    {
        if (value > max)
            value = max;
        else if (value < min)
            value = min;

        return value;
    }

    void restingMotorCommand(float o_cmd, float* l_motor, float* r_motor)
    {
        *r_motor = max_turn*o_cmd;
        *l_motor = -1*max_turn*o_cmd;
        printf("Motor Command Resting: L: %f\tR: %f\n", *l_motor, *r_motor);
    }

    void pathMotorCommand(float o_cmd, float v_cmd, float* l_motor, float* r_motor)
    {
        // make bias point of 75% for forward reverse and leave 25% for turning
        // TODO: previously .25*o_cmd
        float cmd_bias = (max_speed*v_cmd) - .5*(abs(o_cmd)); 
        *r_motor = .5*o_cmd + cmd_bias;
        *l_motor = -.5*o_cmd + cmd_bias; 
        printf("Motor Command: L: %f\tR: %f\tBias: %f\n", *l_motor, *r_motor, cmd_bias);
    }

    // test-1: diff = 1.9PI - .01   = -small    check
    // test-2: diff = PI/4 - 1.5PI  = 3/4PI     check
    // feed 0 to 2PI values **  
    float minimumDistance(float angle_1, float angle_2)
    {
        float difference = angle_1 - angle_2;
        if (difference < -1*PI) difference += 2*PI;
        else if (difference > PI) difference -= 2*PI;
        return difference;
    }

    // store lcm messages
    pose_xyt_t slam_pose, estimated_pose, previous_pose, current_path, start_pose;
    odometry_t odom;
    robot_path_t nav_stack; 

    controller_vector path_vector;

    // set true whenever there is a new SLAM pose update
    bool new_slam_pose; 
    // count which path segement we are on
    int path_counter;

    // difference between slam and odometry
    float delta_x, delta_y, delta_theta;

    // derivative values
    float dx, dy, dtheta, vel, prev_vel, accel, prev_accel;
    uint64_t dt;

    // difference between current orientation and goal pose orientation
    float orientation_diff;

    // command from velocity and orientation handlers
    float vel_cmd;
    float ori_cmd;

    // orrientation error for integration
    float ori_err;
    float path_ori_err;

    static constexpr float max_speed = .5;
    static constexpr float max_turn = .5;

    static constexpr bool orientation_fit_flag = false;

    // acceptable error to declare we have reached the end goal
    static constexpr float PI = 3.14159;
    static constexpr float endgoal_error_tol = .01;    // 1 cm
    static constexpr float endgoal_ori_tol = .1745;    // 10 degrees
    static constexpr float ramp_down_range = .1;       // 10cm

    // constant velocities and accelerations
    static constexpr float max_dtheta = .1745; // 10 degrees/s
    static constexpr float max_vel = .05;      // 5cm/s, max possible might be 10 cm/s
    static constexpr float max_accel = .01;    // 1cm/s^2

    // PID update constants
    // velocity
    static constexpr float kp_vel = 200;
    static constexpr float kd_vel = 10;
    static constexpr float ki_vel = .01;
    // orientation
    static constexpr float kp_ori = 5;
    static constexpr float kd_ori = .5;
    static constexpr float ki_ori = .001;
};


int main(int argc, char** argv)
{
    lcm::LCM lcmInstance;
    
    MotionController controller;
    lcmInstance.subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, &controller);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, &controller);
    lcmInstance.subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, &controller);
    
    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum
        maebot_motor_command_t cmd = controller.updateCommand();
        lcmInstance.publish(MAEBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    }
    
    return 0;
}
