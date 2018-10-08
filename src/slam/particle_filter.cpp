#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <math.h>
#include <common/angle_functions.hpp>
#include <vector>
using namespace std;

float pii = 3.141592;
float R2D1 = 180/pii;
float radius = 2*0.075;    //The range around the maximum weight particle to be taken into consideration (1 robot diameter)

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
    previous_x = 0;
    previous_y = 0;
    previous_theta = 0;
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    // initialize all particles to the start pose and with equal weights
    for (int i = 0; i < kNumParticles_; i++)
    {
        posterior_[i].parent_pose = pose;
        posterior_[i].pose = pose;  
        posterior_[i].weight = (1/(float)kNumParticles_);
        //printf("%d\n",kNumParticles_ );
        //printf("%f\n",posterior_[i].weight );
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const rplidar_laser_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    //printf("UPDATE FILTER Odometry values,%f,%f,%f\n",odometry.x,odometry.y,odometry.theta);
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    //std::cerr << "Update Action" << "\n";
    if(hasRobotMoved)
    {
        //std::cerr << "Robot has moved" << "\n";
        auto prior = resamplePosteriorDistribution();
        //std::cerr << "Test1" << "\n";
        auto proposal = computeProposalDistribution(prior);
        //std::cerr << "Test2" << "\n";
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        //std::cerr << "Test3" << "\n";
        posteriorPose_ = estimatePosteriorPose(posterior_);
        //std::cerr << "Test4" << "\n";
    }
    
    posteriorPose_.utime = odometry.utime;
    //std::cerr << "Posterior pose returned" << "\n";
    return posteriorPose_;
    
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    //particles.utime = utime_now();
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{ 
    ////////////////// TODO: Implement algorithm for resampling from the posterior distribution 

    std::vector<particle_t> prior;

    // always equals 1 by the end of the function
    // used to create intervals based on particle weight
    double particle_interval_sum = 0;

    for (int i = 0; i < kNumParticles_; i++)
    {
        particle_interval_sum += posterior_[i].weight;
        //printf("%f\n", posterior_[i].weight);
        posterior_[i].interval = particle_interval_sum;
        //printf("%d: %f\n",i,posterior_[i].interval);
    }

    //printf("First error check: %f\n",posterior_[kNumParticles_-1].interval );

    // friendly neighborhood error checking
    if ((int)posterior_[kNumParticles_-1].interval >= 1.1) 
        {printf("ABORT! PARTICLE FILTER SET TO SELF DESTRUCT!!!\n");}

    const double importance_interval = 1/(float)kNumParticles_;
    
    // again this should be 1 by the end of the function
    double importance_interval_sum = 0;
    int interval_counter = 0;

    for (int i = 0; i < kNumParticles_; i++)
    {
        while (importance_interval_sum < posterior_[i].interval)
        {
            importance_interval_sum += importance_interval;
            prior.push_back(posterior_[i]);
            prior[interval_counter].interval = importance_interval_sum;
            interval_counter++;
            //printf("%d: %f\n",i, importance_interval_sum);
        }
    }
    //printf("Second error check: %f\n",prior[kNumParticles_-1].interval );
    
    // friendly neighborhood error checking
    if ((int)prior[kNumParticles_-1].interval >= 1.1) 
        {printf("ABORT! PARTICLE FILTER SET TO SELF DESTRUCT (second time)!!!\n");}

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    for (int i = 0; i < kNumParticles_; i++)
    {
       //proposal[i] = actionModel_.applyAction(prior[i]);  
        proposal.push_back(actionModel_.applyAction(prior[i],i));
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const rplidar_laser_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    particle_t p;
    double sum = 0;

    // apply sensor model to get weights
    for (int i = 0; i < kNumParticles_; i++)
    {
        p = proposal[i];
        p.weight = sensorModel_.likelihood(proposal[i], laser, map); 
        //printf("weight = %f\n",p.weight );
        posterior.push_back(p);
        sum += posterior[i].weight;
    }
    //printf("Sum = %f\n",sum );

    // normalize to 1
    for (int i = 0; i < kNumParticles_; i++) 
    {
        posterior[i].weight = (posterior[i].weight/sum);
        //printf("%f\n",posterior[i].weight );
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    std::vector<particle_t> posterior_tmp = posterior;
    float x_sum = 0;
    float y_sum = 0;
    float theta_sum = 0;
    float sin_sum = 0;
    float cos_sum = 0;
    float x_sum_weight = 0;
    float y_sum_weight = 0;
    float sin_sum_weight = 0;
    float cos_sum_weight = 0;
    float theta_sum_weight = 0;
    float weight_sum = 0;
    const float percentage = .02;
    std::sort(posterior_tmp.begin(), posterior_tmp.end(), compareParticles);
    int topParticles = kNumParticles_*percentage;
    for (int i = 0; i < topParticles ; i++)
    {
        //if (sqrt(pow(posterior_tmp[i].pose.x-previous_x,2)+pow(posterior_tmp[i].pose.y-previous_y,2)) < radius)
        {
            x_sum = x_sum + posterior_tmp[i].pose.x;
            y_sum = y_sum + posterior_tmp[i].pose.y;
            sin_sum = sin_sum + sin(posterior_tmp[i].pose.theta);
            cos_sum = cos_sum + cos(posterior_tmp[i].pose.theta);

            /*x_sum_weight = x_sum_weight + posterior_tmp[i].pose.x*posterior_tmp[i].weight; 
            y_sum_weight = y_sum_weight + posterior_tmp[i].pose.y*posterior_tmp[i].weight; 
            sin_sum_weight = sin_sum_weight + sin(posterior_tmp[i].pose.theta)*posterior_tmp[i].weight;
            cos_sum_weight = cos_sum_weight + cos(posterior_tmp[i].pose.theta)*posterior_tmp[i].weight;
            weight_sum = weight_sum + posterior_tmp[i].weight;*/
            //theta_sum = theta_sum + posterior_tmp[i].pose.theta;
            //printf("Top particle[%i]: x: %f\ty: %f\ttheta: %f\n, weight: %f\n",i, posterior_tmp[i].pose.x, posterior_tmp[i].pose.y,posterior_tmp[i].pose.theta,posterior_tmp[i].weight);
        }
    }

    x_sum = x_sum/(topParticles);
    y_sum = y_sum/(topParticles);
    theta_sum = atan2(sin_sum,cos_sum);
    //theta_sum = theta_sum/(topParticles);
    pose.x = x_sum;
    pose.y = y_sum;
    pose.theta = theta_sum;
    //printf("Mean Particle: x: %f\ty: %f\tth: %f\n",x_sum, y_sum, theta_sum);

    /*x_sum_weight = x_sum_weight/weight_sum;
    y_sum_weight = y_sum_weight/weight_sum;
    theta_sum_weight = atan2(sin_sum_weight,cos_sum_weight);
    pose.x = x_sum_weight;
    pose.y = y_sum_weight;
    pose.theta = theta_sum_weight;*/

    /*///////////// PREVIOUS ALGORITHM /////////////////
    // weighted mean
    // take best 10%
    // naively find and return max probability pose as actual pose

    int max_index = 0;
    double max_weight = posterior[0].weight; 

    // find particle of maximum weight
    for (int i = 0; i < kNumParticles_; i++)
    {
        if (posterior[i].weight > max_weight)  
        {
            max_weight = posterior[i].weight;
            max_index = i;
        }
    }
    //printf("Maximum weight: %f\n",max_weight);
   

    double weight_thresh = posterior[max_index].weight*.9;      // must be within 10% of max weight
    //int index_thresh = kNumParticles_*.1;                        // must be within 10% of # tot. particles from max_index
    float max_x = posterior[max_index].pose.x;
    float max_y = posterior[max_index].pose.y;

    std::vector<particle_t> pose_distribution;

    // find particles within 10% distance and 10% weight of particle of maximum weight
    for (int i = 0; i < kNumParticles_; i++) 
    {
        float distance = sqrt(pow((posterior[i].pose.x-max_x),2) + pow((posterior[i].pose.y-max_y),2));
        if ((posterior[i].weight > weight_thresh) && (distance <= radius))
        {
            pose_distribution.push_back(posterior[i]);
            //printf("Valid particle %d: Weight = %f\n",i,posterior[i].weight);
        }    
    }

    double x_mean = 0;
    double y_mean = 0;
    double theta_mean = 0;

    // get the mean pose values out of the particles found above
    for (uint i = 0; i < pose_distribution.size(); i++)
    {
        x_mean += pose_distribution[i].pose.x;
        y_mean += pose_distribution[i].pose.y;
        theta_mean += pose_distribution[i].pose.theta;
        //printf("%f\n",pose_distribution[i].pose.theta );
    }

    x_mean = x_mean/pose_distribution.size();
    y_mean = y_mean/pose_distribution.size();
    theta_mean = theta_mean/pose_distribution.size();           // this might have bad values because of theta singularity

    pose.x = x_mean;
    pose.y = y_mean;
    pose.theta = theta_mean;

    pose.x = posterior[max_index].pose.x;
    pose.y = posterior[max_index].pose.y;
    pose.theta = posterior[max_index].pose.theta;
    
*/
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    
    /*previous_x = pose.x;
    previous_y = pose.y;
    previous_theta = pose.theta;*/
    printf("SLAM Pose: x = %f, y = %f, theta = %f\n\n",pose.x,pose.y,pose.theta);
    return pose;
}
