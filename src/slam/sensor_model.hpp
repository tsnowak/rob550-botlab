#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP
#include <lcmtypes/pose_xyt_t.hpp>

class  rplidar_laser_t;
class  OccupancyGrid;
struct particle_t;

/**
* SensorModel implement a sensor model for computing the likelihood that a laser scan was measured from a
* provided pose, give a map of the environment.
* 
* A sensor model is compute the unnormalized likelihood of a particle in the proposal distribution.
*
* To use the SensorModel, a single method exists:
*
*   - double likelihood(const particle_t& particle, const rplidar_laser_t& scan, const OccupancyGrid& map)
*
* likelihood() computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
*/
class SensorModel
{
public:

    /**
    * Constructor for SensorModel.
    */
    SensorModel(void);

    /**
    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
    * 
    * \param    particle            Particle for which the log-likelihood will be calculated
    * \param    scan                Laser scan to use for estimating log-likelihood
    * \param    map                 Current map of the environment
    * \return   Likelihood of the particle given the current map and laser scan.
    */
    double likelihood(const particle_t& particle, const rplidar_laser_t& scan, const OccupancyGrid& map);

private:
    
    ///////// TODO: Add any private members for your SensorModel ///////////////////
    float sigma_hit;
    float lambda_short;
    float zhit;
    float zshort;
    float zmax;
    float zrand;
    float q;
    float p;
    float z_max;
    int xk;
    int yk;
    float dist;
    float dist_temp;
    float sum;
    pose_xyt_t begin_pose;
};

#endif // SLAM_SENSOR_MODEL_HPP
