#include <slam/sensor_model.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <lcmtypes/rplidar_laser_t.hpp>
#include <math.h>
#include <random>
#include <vector>
#include <slam/moving_laser_scan.hpp>
using namespace std;


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
    sigma_hit = 0.4;		//Error parameters
    lambda_short = 0;
    zhit = 1;
    zshort = 0;
    zmax = 0;
    zrand = 0;
    q = 1;			//Temporary probability
    p = 1;
    z_max = 5;		//Maximum range of sensor
    xk = 0;			//Projection of end point of sensor scan into global coordinates in cells
    yk = 0;
    dist = 100;	//Initialize this as some big value 
    dist_temp = 0;
    sum = 0;
}


double SensorModel::likelihood(const particle_t& sample, const rplidar_laser_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    MovingLaserScan mls(scan,sample.parent_pose,sample.pose,1);
    adjusted_ray_t ray;
    q = 1;
    float sigma = 0.03;
    float dividor = 45720;
    sum = 0;
    for (auto& ray : mls)
    {
      float current_pose_x = ray.origin.x;
      float current_pose_y = ray.origin.y;
      xk = (int)(floor((current_pose_x + ray.range*cos(ray.theta))/map.metersPerCell()) + map.widthInCells()/2);
      yk = (int)(floor((current_pose_y + ray.range*sin(ray.theta))/map.metersPerCell()) + map.heightInCells()/2);
      int xk1 = (int)(floor((current_pose_x + (ray.range + 0.05)*cos(ray.theta))/map.metersPerCell()) + map.widthInCells()/2);
      int yk1 = (int)(floor((current_pose_y + (ray.range + 0.05)*sin(ray.theta))/map.metersPerCell()) + map.heightInCells()/2);
      int xk2 = (int)(floor((current_pose_x + (ray.range - 0.05)*cos(ray.theta))/map.metersPerCell()) + map.widthInCells()/2);
      int yk2 = (int)(floor((current_pose_y + (ray.range - 0.05)*sin(ray.theta))/map.metersPerCell()) + map.heightInCells()/2);

      
    	if (ray.range<z_max)
    	{
        dist = 100;
        float log_odds = 0;
        //theta + or - confirm
    		/*xk = (int)floor((sample.pose.x + (scan.ranges[k])*cos(sample.pose.theta - scan.thetas[k]))/map.metersPerCell()) + map.widthInCells()/2;	
    		yk = (int)floor((sample.pose.y + (scan.ranges[k])*sin(sample.pose.theta - scan.thetas[k]))/map.metersPerCell()) + map.heightInCells()/2;
        int xk1 = (int)floor((sample.pose.x + (scan.ranges[k] + 0.05)*cos(sample.pose.theta - scan.thetas[k]))/map.metersPerCell()) + map.widthInCells()/2; 
        int yk1 = (int)floor((sample.pose.y + (scan.ranges[k] + 0.05)*sin(sample.pose.theta - scan.thetas[k]))/map.metersPerCell()) + map.heightInCells()/2;
        int xk2 = (int)floor((sample.pose.x + (scan.ranges[k] - 0.05)*cos(sample.pose.theta - scan.thetas[k]))/map.metersPerCell()) + map.widthInCells()/2; 
        int yk2 = (int)floor((sample.pose.y + (scan.ranges[k] - 0.05)*sin(sample.pose.theta - scan.thetas[k]))/map.metersPerCell()) + map.heightInCells()/2;*/
        /*xk_ = (int)xk;
        yk_ = (int)yk;
        xk1_ = (int)xk1;
        yk1_ = (int)yk1;
        xk2_ = (int)xk2;
        yk2_ = (int)yk2;*/

        if(map.logOdds(xk,yk)>0)
        {
          dist = 0;
          log_odds = map.logOdds(xk,yk);
        }  
          
        else if((map.logOdds(xk1,yk1)>0)||(map.logOdds(xk1,yk1)>0))
        {
          float dist1 = sqrt(pow(xk1 - xk,2) + pow(yk1 - yk,2))*map.metersPerCell();
          float dist2 = sqrt(pow(xk2 - xk,2) + pow(yk2 - yk,2))*map.metersPerCell();
          dist = min(dist1,dist2);
          if (dist == dist1)
            log_odds = map.logOdds(xk1,yk1);
          else 
            log_odds = map.logOdds(xk2,yk2);
        }
        
        p = exp(-pow(dist,2)/pow(sigma,2)); 
       
        /*if(map.logOdds(xk,yk)>0)
          sum = sum + map.logOdds(xk,yk);*/
        //printf("xk = %d, yk = %d\n",xk,yk);

        /*for(int j=-1;j<2;++j)
        {
          for(int i=-1;i<2;++i)
          {
            if(map.logOdds((xk+i),(yk+j))>0)
            {
              dist_temp = (sqrt(i*i + j*j))*map.metersPerCell();
              //printf("Dist_sqr_temp = %f\n",dist_sqr_temp );
              if (dist_temp<dist)
                dist = dist_temp;
              else
                dist = dist;
            }
          }
        }*/
        //printf("Distance = %f\n",dist);

    		/*for(int j = 0; j < map.heightInCells(); ++j)
       		{
           		for(int i = 0; i < map.widthInCells(); ++i)
           			{
               			if(map.logOdds(i,j)>0)
               			{	
               				float x = (i-(map.widthInCells()/2))*(map.metersPerCell());
               				float y = (j-(map.heightInCells()/2))*(map.metersPerCell());
               				dist_sqr_temp = (xk-x)*(xk-x) + (yk-y)*(yk-y);
               				if (dist_sqr_temp<dist_sqr)
               					{dist_sqr = dist_sqr_temp;}
               				else
               					{dist_sqr = dist_sqr;}
               			}
           			}
       		}*/
        /*if (dist != 100)
        {
          p = (1/(sigma_hit*sqrt(2*3.14159)))*exp(-(dist)/(2*sigma_hit*sigma_hit));
        }
        //printf("p%d = %f\n",k,p);
       	q = q*zhit*p;
        */
    	}
      q = q+p;
    }

    //printf("q = %f\n",q );  
    return q;
    //printf("Probability of particle: %f \n", sum/dividor);
    //return (sum/dividor);
}
