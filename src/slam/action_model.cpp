#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/pose_trace.hpp>
#include <math.h>
#include <random>
#include <vector>
using namespace std;

//float epsilon[3] = {0,0,0};
ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    /*alpha1 = 1;		//Robot-specific error parameters
    alpha2 = 1;
    alpha3 = 1;
    alpha4 = 1;

    drot1 = 0;		//Rotation -> Translation -> Rotation to go from previous to current odometry pose
    dtrans = 0;
    drot2 = 0;

    sigma1 = 0;		//Standard deviations for the normal distributions
    sigma2 = 0;
    sigma3 = 0;

    drot1_cap = 0;		//Corrected values for drot1,dtrans,drot2
    dtrans_cap = 0;
    drot2_cap = 0;*/

}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
   /* PoseTrace poses;
    int index = poses.size() - 1;		//For previous odometry pose

    drot1 = atan2((odometry.y - poses.at(index).y),(odometry.x - poses.at(index).x)) - poses.at(index).theta;
    dtrans = sqrt((odometry.y - (poses.at(index).y)*(poses.at(index).y)) + (odometry.x - (poses.at(index).x))*(poses.at(index).x));
    drot2 = odometry.theta - poses.at(index).theta - drot1;

    sigma1 = alpha1*abs(drot1) + alpha2*abs(dtrans);		//Standard deviations for the normal distributions
    sigma2 = alpha3*abs(dtrans) + alpha4*abs(drot1 + drot2);
    sigma3 = alpha1*abs(drot2) + alpha2*abs(dtrans);

    std::default_random_engine generator;		//Random number

    std::normal_distribution<float> epsilon1(0.0,sigma1);		//Normal distribution with mean 0 and variance sigma^2
    std::normal_distribution<float> epsilon2(0.0,sigma2);
    std::normal_distribution<float> epsilon3(0.0,sigma3);
    
    epsilon[0] = epsilon1(generator);
    epsilon[1] = epsilon2(generator);
    epsilon[2] = epsilon3(generator);*/

    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
   /* particle_t sample1;

	drot1_cap = drot1 - epsilon[0];		//Corrected values for drot1,dtrans,drot2
	dtrans_cap = dtrans - epsilon[1];
	drot2_cap = drot2_cap - epsilon[2];

	sample1.pose.x = sample.parent_pose.x + dtrans_cap*cos(sample.parent_pose.theta + drot1_cap);	//New pose
	sample1.pose.y = sample.parent_pose.y + dtrans_cap*sin(sample.parent_pose.theta + drot1_cap);
	sample1.pose.theta = sample.parent_pose.theta + drot1_cap + drot2_cap;

	sample1.parent_pose = sample1.pose;		//Parent pose update*/

    
    return sample;
}
