#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/pose_trace.hpp>
#include <common/angle_functions.hpp>
#include <common/timestamp.h>
#include <cmath>
#include <random>
#include <vector>
using namespace std;

float pi = 3.141592;
float R2D = 180/pi;

float epsilon[3] = {0,0,0};
int counter = 0;
ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    previous_x = 0;     //Previous odometry pose
    previous_y = 0;
    previous_theta = 0;
    previous_time = 0;

    /*alpha1 = 0.1;       //Robot specific error parameters  
    alpha2 = 0.1;            
    alpha3 = 0.001;             
    alpha4 = 0.0001;*/

    alpha1 = 0.05;        
    alpha2 = 0.05;            
    alpha3 = 0.001;             
    alpha4 = 0.0001;     

    drot1 = 0;		//Rotation -> Translation -> Rotation to go from previous to current odometry pose
    dtrans = 0;
    drot2 = 0;

    sigma1 = 0;		//Standard deviations for the normal distributions
    sigma2 = 0;
    sigma3 = 0;

    drot1_cap = 0;		//Corrected values for drot1,dtrans,drot2
    dtrans_cap = 0;
    drot2_cap = 0;

    t = 0;      //Time

}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    if ((fabs(odometry.x - previous_x)>0.001) || (fabs(odometry.y - previous_y)>0.001) || (fabs(odometry.theta - previous_theta)>0.01))
    {
        if ((fabs(odometry.x - previous_x)<=0.001) && (fabs(odometry.y - previous_y)<=0.001))
        {   drot1 = 0;
            dtrans = 0;
            drot2 = wrap_to_pi(odometry.theta - previous_theta);
        }
        else
        {
            drot1 = wrap_to_pi(wrap_to_pi(atan2((odometry.y - previous_y),(odometry.x - previous_x))) - previous_theta);
            dtrans = sqrt((odometry.y - previous_y)*(odometry.y - previous_y) + (odometry.x - previous_x)*(odometry.x - previous_x));
            drot2 = wrap_to_pi(odometry.theta - previous_theta - drot1);
        }
        //printf("UPDATE ACTION Previous Odometry values,%lli,%f,%f,%f\n",previous_time/1000000,previous_x,previous_y,previous_theta*R2D);
        //printf("UPDATE ACTION Current  Odometry values,%lli,%f,%f,%f\n",odometry.utime/1000000,odometry.x,odometry.y,odometry.theta*R2D);
        printf("drot1 = %f,dtrans = %f,drot2 = %f\n",drot1,dtrans,drot2);
        t = odometry.utime;

        if (fabs(drot1)>pi/2)
        {
            drot1 = wrap_to_pi(-pi + drot1);
            dtrans = -sqrt((odometry.y - previous_y)*(odometry.y - previous_y) + (odometry.x - previous_x)*(odometry.x - previous_x));
            drot2 = wrap_to_pi(pi + drot2);
        }
        printf("drot1 = %f,dtrans = %f,drot2 = %f\n",drot1,dtrans,drot2);

        previous_x = odometry.x;
        previous_y = odometry.y;
        previous_theta = odometry.theta;
        previous_time = odometry.utime;
        return true;
    }

    else
        {return false;}

}


particle_t ActionModel::applyAction(const particle_t& sample, const int i)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t sample1;
    sample1.parent_pose = sample.pose;

    sigma1 = sqrt(alpha1*pow(drot1,2) + alpha2*pow(dtrans,2));        //Standard deviations for the normal distributions
    sigma2 = sqrt(alpha3*pow(dtrans,2) + alpha4*(pow(drot1,2) + pow(drot2,2)));
    sigma3 = sqrt(alpha1*pow(drot2,2) + alpha2*pow(dtrans,2));

    /*sigma1 = (alpha1*fabs(drot1) + alpha2*fabs(dtrans));        //Standard deviations for the normal distributions
    sigma2 = (alpha3*fabs(dtrans) + alpha4*(fabs(drot1) + fabs(drot2)));
    sigma3 = (alpha1*fabs(drot2) + alpha2*fabs(dtrans));
*/
    /*std::default_random_engine generator1;       //Random number
    std::default_random_engine generator2;
    std::default_random_engine generator3;*/

    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<float> epsilon1(0.0,sigma1);       //Normal distribution with mean 0 and variance sigma^2
    std::normal_distribution<float> epsilon2(0.0,sigma2);
    std::normal_distribution<float> epsilon3(0.0,sigma3);
    
    epsilon[0] = epsilon1(gen);
    epsilon[1] = epsilon2(gen);
    epsilon[2] = epsilon3(gen);

	drot1_cap = wrap_to_pi(drot1 - epsilon[0]);		//Corrected values for drot1,dtrans,drot2
	dtrans_cap = dtrans - epsilon[1];
	drot2_cap = wrap_to_pi(drot2 - epsilon[2]);

	sample1.pose.x = sample.pose.x + dtrans_cap*cos(sample.pose.theta + drot1_cap);	//New pose
	sample1.pose.y = sample.pose.y + dtrans_cap*sin(sample.pose.theta + drot1_cap);
	sample1.pose.theta = wrap_to_pi(sample.pose.theta + drot1_cap + drot2_cap);
    sample1.pose.utime = t;
    //printf("Action model theta: %f\n",sample1.pose.theta);

    
	//printf("x = %f, y = %f, theta = %f\n",sample1.pose.x,sample1.pose.y,sample1.pose.theta);
    if (i==499)
    {
        printf("Errors: %f, %f, %f\n",epsilon[0],epsilon[1],epsilon[2]);
    //printf("Variance: %f, %f, %f\n",sigma1,sigma2,sigma3);
    //printf("Action model values previous,%lli,%f,%f,%f\n",sample.pose.utime/1000000,sample.pose.x,sample.pose.y,sample.pose.theta*R2D);
    //printf("Action model values current,%lli,%f,%f,%f\n",sample1.pose.utime/1000000,sample1.pose.x,sample1.pose.y,sample1.pose.theta*R2D);
    }
    return sample1;
}
