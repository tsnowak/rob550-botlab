#include <maebot/maebot_channels.h>
#include <lcmtypes/maebot_encoders_t.hpp>
#include <lcmtypes/maebot_imu_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include <stdio.h>

float D2R = 3.1415/180;
float R2D = 180/3.1415;
float TSpD = 131000000;           // gyro coversion factor in ticks-msec per degree , results in degrees
float errTpS = 103.395054 * 0.94 ;   // gyro error in ticks per msec measured * adjustment to 94% eliminated error
float thres_theta;

/**
* Odometry calculates the odometry for the robot. The Odometry class should be subscribed to a maebot_encoders_t and
* maebot_imu_t (for Gyrodometry) message.
* 
* Each time a new LCM message arrives, the haveNewOdometry method will be called to check if a new odometry estimate 
* is ready. If it is, then the update() method is called to compute the new estimate.
* 
* For regular odometry, only encoder data is needed. For Gyrodometry, you will need to handle time-synchronization 
* between the encoder and IMU measurements.
*/
class Odometry
{
public:
    
    Odometry(float metersPerTick, float wheelbase)
    : metersPerTick_(metersPerTick)
    , wheelbase_    (wheelbase)
    {
        ////////// TODO: Perform any additional initialization that is neededs
        trip.utime = 0;
        trip.x = 0;
        trip.y = 0;
        trip.theta = 0;
        trip.total_left_distance = 0;
        trip.total_right_distance = 0;
        trip.left_wheel_speed = 0;
        trip.right_wheel_speed = 0;
        trip.wheelbase = wheelbase;
        tripENC = tripIMU = trip;
        prevEnc.utime = currEnc.utime = 0;
        prevEnc.left_ticks_total = currEnc.left_ticks_total = 0;
        prevEnc.right_ticks_total = currEnc.right_ticks_total = 0;
        initIMU.utime = 0;
        initIMU.raw_acceleration[0] = initIMU.raw_orientation[0] = initIMU.raw_angular_velocity[0] = {0};
        initIMU.raw_acceleration[1] = initIMU.raw_orientation[1] = initIMU.raw_angular_velocity[1] = {0};
        initIMU.raw_acceleration[2] = initIMU.raw_orientation[2] = initIMU.raw_angular_velocity[2] = {0};
        currIMU = prevIMU = initIMU;

        gc = oc = 0;
        newOdomFlag = 0;
        isStart = isStartIMU = 1; // 1 if the bot has not published any messages, 0 if the bot has published at least one message
        t0e = t0imu = 0;  //timestamps when the bot started in encoder and imu clocks (different)
    }
    
    int haveNewOdometry(void) const
    {
        ///////////// TODO: Determine when you have enough data for computing a new odometry estimate /////////////////
        //if (prevEnc.left_ticks_total != currEnc.left_ticks_total || prevEnc.right_ticks_total != currEnc.right_ticks_total){
           // return true;    // only true if the maebot has moved at least one of its wheels
            //}
        
        return newOdomFlag;
    }
    
    odometry_t update()//float mpt, float wb)
    {
        ///////////////////// TODO: Compute the updated odometry estimate //////////////////////////////
        
        //odometry_t odometry;
        maebot_encoders_t deltaEnc;
        float b, dl, dr, dt, dtimu, odo_dtheta, imu_dtheta, dtheta, dx, w, v;
        int movedFlag = 0;

        if (prevEnc.left_ticks_total != currEnc.left_ticks_total || prevEnc.right_ticks_total != currEnc.right_ticks_total)
        {
        movedFlag = 1;
        }

        // calculate incremental displacement values
        deltaEnc.utime = currEnc.utime - prevEnc.utime;
        deltaEnc.left_ticks_total = currEnc.left_ticks_total - prevEnc.left_ticks_total;
        deltaEnc.right_ticks_total = currEnc.right_ticks_total - prevEnc.right_ticks_total;
        
        dt = deltaEnc.utime;
        dl = deltaEnc.left_ticks_total*metersPerTick_;   //inc distance traveled by left wheel in m
        dr = deltaEnc.right_ticks_total*metersPerTick_;  //inc distance traveled by right wheel in m
        b = wheelbase_;
        dx = (dr + dl)/2;
        v = dx/dt*1000000;      // speed in m/sec
        odo_dtheta = (dr - dl)/b;

        imu_dtheta = (currIMU.raw_orientation[2] - prevIMU.raw_orientation[2])/TSpD*D2R;  // in radians
        dtimu = currIMU.utime - prevIMU.utime;
        
        float diff = fabs(odo_dtheta - imu_dtheta)*R2D;
        
        if ( diff > thres_theta )
            {
            gc +=1;
            dtheta = imu_dtheta; 
            w = dtheta/dtimu*1000000;
            printf("TAKING IMU with diff = %1.2f \n", diff);// --- dtheta = %1.4lf in delta_t of = %1.2lf for ang speed of = %1.10lf \n", dtheta*R2D, dtimu, w*R2D);
            }
        else
            {
            oc +=1;  
            dtheta = odo_dtheta;
            w = dtheta/dt*1000000;
            //printf("TAKING ENC --- dtheta = %1.4lf in delta_t of = %1.2lf for ang speed of = %1.10lf \n", dtheta*R2D, dt, w*R2D);
            }

        // calculate aggregated displacement values
        tripENC.utime = currEnc.utime-t0e;
        tripENC.x += dx*cos(tripENC.theta);
        tripENC.y += dx*sin(tripENC.theta);
        tripENC.theta += odo_dtheta;
        tripENC.total_left_distance += dl;
        tripENC.total_right_distance += dr;

        tripIMU.utime = currIMU.utime-initIMU.utime;
        tripIMU.x += dx*cos(tripIMU.theta);
        tripIMU.y += dx*sin(tripIMU.theta);
        tripIMU.theta += imu_dtheta;
        tripIMU.total_left_distance += dl;
        tripIMU.total_right_distance += dr;

        trip.utime = currEnc.utime-t0e;
        trip.x += dx*cos(trip.theta);
        trip.y += dx*sin(trip.theta);
        trip.theta += dtheta;
        trip.total_left_distance += dl;
        trip.total_right_distance += dr;

        //need to update left and right wheel speed according to definition/use 
        //need to include forward and angular velocity in odometry_t type to publish them 
        newOdomFlag = 0;

        if (movedFlag)
        {
        printf("in func UPDATE: delta      : dl = %+1.4f       dr = %+1.4f       dx = %+1.4f             v (m/s)= %+1.3f \n", \
                dl, dr, dx, v);
        printf("in func UPDATE: THETA ENC  : dt = %1.2f          dtheta = %+1.4f   w(deg/s) = %+1.10f\n", \
                dt/1000000, odo_dtheta*R2D, odo_dtheta/dt*1000000*R2D);
        printf("in func UPDATE: THETA IMU  : dtimu = %1.2f       dtheta = %+1.4f   w(deg/s) = %+1.10f\n", \
                dtimu/1000000, imu_dtheta*R2D, imu_dtheta/dtimu*1000000*R2D);
        printf("in func UPDATE: 100%% Odometry       : utime (s)= %1.2f    x = %1.4f         y = %1.4f               theta = %1.4f    total_L_dis = %1.4f    total_R_dis = %1.4f \n" , \
                (float) tripENC.utime/1000000, tripENC.x, tripENC.y, tripENC.theta*R2D,tripENC.total_left_distance,tripENC.total_right_distance);
        printf("in func UPDATE: 100%% IMU data       : utime (s)= %1.2f    x = %1.4f         y = %1.4f               theta = %1.4f    total_L_dis = %1.4f    total_R_dis = %1.4f \n" , \
                (float) tripIMU.utime/1000000, tripIMU.x, tripIMU.y, tripIMU.theta*R2D,tripIMU.total_left_distance,tripIMU.total_right_distance);
        printf("in func UPDATE: Gyrodometry          : utime (s)= %1.2f    x = %1.4f         y = %1.4f               theta = %1.4f    total_L_dis = %1.4f    total_R_dis = %1.4f \n\n" , \
                (float) trip.utime/1000000, trip.x, trip.y, trip.theta*R2D,trip.total_left_distance,trip.total_right_distance);
        printf("Using threshold = %1.2f  Gyrodometry used gryro data = %1.2f %% of the times  \n", thres_theta, (float) gc/(gc+oc)*100);
        }

        return trip;
    }
    
    void handleEncoders(const lcm::ReceiveBuffer* buf, const std::string& channel, const maebot_encoders_t* encoders)
    {
        ///////// TODO: Implement your handler for encoder data ////////////////

        // values that need to be updates everytime that motors publish information
        newOdomFlag = 1;
        if(isStart == 1) 
            {
            prevEnc = *encoders;
            t0e = prevEnc.utime; 
            printf("initial ENC time: %li\n",t0e);
            isStart = 0;
            }
        else 
            { 
            prevEnc = currEnc;
            }
        currEnc = *encoders;  
        //trip.utime = currEnc.utime;

        //printf("utime from ENCODERS: %li time since enc start: %1.2f \n", currEnc.utime, (float) (currEnc.utime-t0e)/1000000);
        
        //ok: tested reading and uptading the values of curr enc correctly
        //if (prevEnc.left_ticks_total != currEnc.left_ticks_total || prevEnc.right_ticks_total != currEnc.right_ticks_total)
        //{
        //printf("in func ENCODER HANDLER: trip.utime = %li prevEnc.left_ticks_total = %i prevEnc.right_ticks_total = %i \n", trip.utime, prevEnc.left_ticks_total, prevEnc.right_ticks_total);
        //printf("in func ENCODER HANDLER: trip.utime = %li currEnc.left_ticks_total = %i currEnc.right_ticks_total = %i \n", trip.utime, currEnc.left_ticks_total, currEnc.right_ticks_total);
        //}

        //        currEnc.left_ticks_total = %li 
        //        currEnc.right_ticks_total = %li \n", trip.utime, currEnc.left_ticks_total, currEnc.right_ticks_total);
        
        //would need to update left wheel and right wheel speed if they are used to track average total trip speed
    }
    
    
    void handleImu(const lcm::ReceiveBuffer* buf, const std::string& channel, const maebot_imu_t* imu)
    {
        /////////// TODO: Implement your handler for IMU data ////////////////
        
        //captures baseline for IMU, starting values for the signals
        if(isStartIMU == 1) 
            {
            initIMU = *imu;
            prevIMU = initIMU;
            prevIMU.raw_orientation[2] = 0;                     // offset by initial value
            printf("initial IMU time: %li\n",initIMU.utime);
            isStartIMU = 0;
            }
        else
            {
            prevIMU = currIMU;    
            }

        currIMU = *imu;
                
        //prints all IMU data to file, used to calculate drift in gyro signal
        if(false){
            fprintf(pF,"Time: %1.2lf ", (float) (currIMU.utime-initIMU.utime)/1000000);
            fprintf(pF,"Axix X: %1.2lf %1.2lf %1.2lf ", \
            currIMU.raw_acceleration[0],currIMU.raw_orientation[0], currIMU.raw_angular_velocity[0]);    
            fprintf(pF,"Axix Y: %1.2lf %1.2lf %1.2lf ", \
            currIMU.raw_acceleration[1],currIMU.raw_orientation[1], currIMU.raw_angular_velocity[1]); 
            fprintf(pF,"Axix Z: %1.2lf %1.2lf %1.2lf \n", \
            currIMU.raw_acceleration[2],currIMU.raw_orientation[2], currIMU.raw_angular_velocity[2]); 
            }
        
        //correct IMU data: offset orientation by initial value and cumulative error 
        //only implemented in Z axis of intrest. If needed should be implemented in X and Y too 
        currIMU.raw_orientation[2] = currIMU.raw_orientation[2] - initIMU.raw_orientation[2] + errTpS*(currIMU.utime-initIMU.utime);

        //prints  IMU data to screen
        //if (prevEnc.left_ticks_total != currEnc.left_ticks_total || prevEnc.right_ticks_total != currEnc.right_ticks_total)
        if (false)    
            {printf("in func IMU HANDLER: Z orientation = %+1.4lf \n\n", currIMU.raw_orientation[2]/TSpD);}
        if (false)
            {
            printf("utime from IMU: %li time since start: %1.2f \n", currIMU.utime, (float) (currIMU.utime-initIMU.utime)/1000000);
            for (int i=0; i<=2; i++)
                {
                printf("in func IMU HANDLER: acceleration = %+1.2lf    orientation = %+1.2lf    angular velocity = %+1.2lf \n", \
                currIMU.raw_acceleration[i],currIMU.raw_orientation[i]/TSpD, currIMU.raw_angular_velocity[i]);    
                }
            printf("\n");
            }
    }

    odometry_t trip, tripIMU, tripENC;      // stores pos, dist, sppeds, time from time = 0 until the latest values published by the maebot encoders
    maebot_encoders_t prevEnc, currEnc;     // stores the last two readings from the motor encoders, used to determine incremental movement
    maebot_imu_t initIMU, prevIMU, currIMU; // stores the initial, previous and current reading from the IMU sensor
    int newOdomFlag, isStart, isStartIMU;   // 0 if no new data, 1 if new encoder data
    int64_t t0e, t0imu;    
    FILE *pF;
    int64_t gc, oc;                   

private:
    
    const float metersPerTick_;
    const float wheelbase_;
        //////////// TODO: Add any additional private member variables to support your implementation

};


int main(int argc, char** argv)
{
    ///////// TODO: Use the Maebot datasheet to fill in the robot-specific parameters //////////////////
    const float kWheelbase          = 0.08;     // in meters
    const float kWheelCircumference = 0.100528; // in meters
    const int   kTicksPerRevolution = 1200;     // in ticks
    

    printf("please input theta treshold = "); 
    //thres_theta = 1; // a large nubmer like 1 will not use gyro data
    scanf("%f",&thres_theta);             // select a threshold by trial and error
    

    Odometry odometry(kWheelCircumference / kTicksPerRevolution, kWheelbase);
    lcm::LCM lcmConnection; 
    
    odometry.pF = fopen ("gyrodata.txt", "w");


    ///////// TODO: Subscribe to the appropriate LCM channels //////////////////
    ///////// need to subscrive  Odometry::handleEncoders to maebot encoder channel
    //???//odometry_t_subscribe(lcmConnection,"MAEBOT_ENCODERS", Odometry::handleEncoders, NULL);
    
    lcmConnection.subscribe("MAEBOT_ENCODERS",&Odometry::handleEncoders,&odometry);
    lcmConnection.subscribe("MAEBOT_IMU",&Odometry::handleImu,&odometry);
    
    // Now run forever the odometry loop:
    while(true)
    {
        // Wait for the next LCM message to arrive
        lcmConnection.handle();

        
        // ok: odometry values updated correclty here
        //if (true)//(odometry.prevEnc.left_ticks_total != odometry.currEnc.left_ticks_total || odometry.prevEnc.right_ticks_total != odometry.currEnc.right_ticks_total)
        //    {
        //    printf("in func MAIN: trip.utime = %li prevEnc.left_ticks_total = %i prevEnc.right_ticks_total = %i \n", odometry.trip.utime, odometry.prevEnc.left_ticks_total, odometry.prevEnc.right_ticks_total);
        //    printf("in func MAIN: trip.utime = %li currEnc.left_ticks_total = %i currEnc.right_ticks_total = %i \n", odometry.trip.utime, odometry.currEnc.left_ticks_total, odometry.currEnc.right_ticks_total);
        //    }

        // If there's data for a new odometry estimate
        if(odometry.haveNewOdometry())
        {
            // Compute the new estimate
            auto newOdom = odometry.update();
            
            // And send it off to other modules via LCM
            lcmConnection.publish(ODOMETRY_CHANNEL, &newOdom);
        }
    }
    
    fclose(odometry.pF);

    return 0;
}
