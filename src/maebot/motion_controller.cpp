#include <maebot/maebot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/maebot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>


class MotionController
{
public:
    
    /**
    * Constructor for MotionController.
    */
    MotionController(void)
    {
        ////////// TODO: Initialize your controller state //////////////
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
        //////////// TODO: Implement your feedback controller here. //////////////////////
        // U(t) = Up(t) + Ud(t) + Ui(t)
        // Where xd is x desired and x is actual
        // Up(t) = kp(xd - x)
        // Ud(t) = kd(dxd(t) - dx(t))
        // dx = x(t)-x(t-1)/dt
        // Ui(t) = ki(integral(e(t)dt))
        // where e(t) is the error between current location and set point
        
        maebot_motor_command_t cmd;
        cmd.utime = utime_now();
        cmd.left_motor_enabled = 1;     // make sure all motors are correctly enabled so the robot moves
        cmd.right_motor_enabled = 1;
        cmd.left_motor_speed = 0.0f;
        cmd.right_motor_speed = 0.0f;
        
        return cmd;
    }
    
    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    {
        /////// TODO: Implement your handler for new paths here ////////////////////
  
        // time of path creation
        pid_path.utime  = path->utime;

        // number of poses in the path
        pid_path.path_length = path->path_length;

        // sequence of poses_xyt_t that make up the path
        pid_path.path = path->path;
    }
    
    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        /////// TODO: Implement your handler for new odometry data ////////////////////

        // time of odometry creation
        odom.utime = odometry->utime;

        // odometry pose orientation and position
        odom.x = odometry->x;
        odom.y = odometry->y;
        odom.theta = odometry->theta;

        // TODO: total distance each wheel has traveled, length of wheelbase
        odom.total_left_distance = odometry->total_left_distance;
        odom.total_right_distance = odometry->total_right_distance;
        odom.wheelbase = odometry->wheelbase;

        // speed of each wheel
        odom.left_wheel_speed = odometry->left_wheel_speed;
        odom.right_wheel_speed = odometry->right_wheel_speed;
    }
    
    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        /////// TODO: Implement your handler for new pose data ////////////////////

        // time of pose creation
        slam_pose.utime = pose->utime;

        // pose orientation and position
        slam_pose.x = pose->x;
        slam_pose.y = pose->y;
        slam_pose.theta = pose->theta;
    }
    
private:
    
    ///////// TODO: Add private member variables that are needed for your motion controller implementation ///////////


    // make variables to store subscription values
    // whenever we get a new SLAM pose, use that and zero the odometry bias based on that
    // implement PID from last given suspected location/pose and store values necessary
    // to do the calculations  
    pose_xyt_t slam_pose;
    odometry_t odom;
    robot_path_t pid_path; 
    
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
