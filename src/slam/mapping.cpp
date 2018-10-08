#include <slam/mapping.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/rplidar_laser_t.hpp>
#include <math.h>
#include <slam/moving_laser_scan.hpp>
#include <common/point.hpp>

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


//void Mapping::updateMap(const rplidar_laser_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)

void Mapping::updateMap(const rplidar_laser_t& scan, const pose_xyt_t& begSPose,  const pose_xyt_t& endSPose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

	MovingLaserScan movingScan(scan, begSPose, endSPose, 1);  //object contains an array of scan beams with different starting points adjusted for movement of lidar
	printf("\n begSPose x,y,theta = %1.3f,%1.3f,%1.3f endSpose x,y,theta = %1.3f,%1.3f,%1.3f \n", begSPose.x, begSPose.y, begSPose.theta, endSPose.x, endSPose.y, endSPose.theta);

	adjusted_ray_t beam;
	float xi,yi,xh,yh,xs,ys;
	float ang;
	
	float dr  = 0.05; // steps of 5cm along the beam 
	float csize = 0.05; // this is the cell size 5cm
	float Xg, Yg;
	Xg = Yg = 5;  // this is the offset of the corner of the map to the center of the map. x,y are generally 0,0 in the center of the map and can take neg values
	int row, col, hrow, hcol, prevrow, prevcol;
	hrow = hcol = -1;
	int b=0;
	char c;

	Point<int> p;
	std::vector<Point<int>> Phits; //will store row,col that have been hit on a single lidar scan
		
	// for (std::vector<adjusted_ray_t>::iterator it = movingScan.begin() ; it != movingScan.end(); ++it);
    // process each laser beam in the adjustedRays vector
    for (auto it : movingScan)
	{
		b++;
		beam = it; ///may need to be a pointer ?
		xi  = Xg + beam.origin.x;
		yi  = Yg + beam.origin.y;
		ang = beam.theta;
		
		//update odds for final/hit positions 
		//map is a vector of cells, access element with indexes starting at 0 up to 199 for both rows and columns (map of size 10m, cells of size 0.05m) 
		if (beam.range < kMaxLaserDistance_)
		{ 
			xh  = xi + beam.range * cos(ang);
			yh  = yi + beam.range * sin(ang);
			hrow = floor(xh/csize);
			hcol = floor(yh/csize);
			p.x = hrow;
			p.y = hcol;
			Phits.push_back(p);
			if( (map.logOdds(hrow,hcol)<=(127-kHitOdds_)) )
				 { map.setLogOdds(hrow,hcol, map.operator()(hrow,hcol) += kHitOdds_); }
			else { map.setLogOdds(hrow,hcol, map.operator()(hrow,hcol) =127); }
			
			if(fabs(fmod(ang,1.57))<0.025){ // use 0.025 to print only quadrants 
				printf("Beam # = %i  xi,yi = %1.3f,%1.3f range = %1.3f angle = %1.3f xh,yh = %1.3f,%1.3f hit row/col = %i,%i \n", b,xi,yi,beam.range,ang,xh,yh,hrow,hcol);}
		}

		// update as free everything along the beam trajectory line except the final/hit point 
		row = floor(xi/csize);
		col = floor(yi/csize);
		prevrow = prevcol = -1; //used to censure we only update once every free cell. (-1) ensures we at least update our initial position xi,yi as free
		int i = 1;
		do  
		{
			//find cell of next point using dr along the line (expensive but accurate) or using bersenham algorithm if error of +/- 2.5cm is ok;
			//if((row!=prevrow || col!=prevcol) && (map[row,col]>-127)){ map[row,col] -= kMissOdds_;} 
			//if((row!=prevrow || col!=prevcol) && (row != hrow || col != hcol) && (map.logOdds(row,col)>-127)){ map.setLogOdds(row,col,map.operator()(row,col) -= kMissOdds_);} 
			if((row!=prevrow || col!=prevcol) && (!isHitPoint(row,col,Phits)) && (map.logOdds(row,col)>-127)){ map.setLogOdds(row,col,map.operator()(row,col) -= kMissOdds_);} 
			prevrow = row;
			prevcol = col; 
			xs  = xi + i*dr*cos(ang);
			ys  = yi + i*dr*sin(ang);
			//printf("Step free # = %i xs,ys = %1.2f,%1.2f free row/col = %i,%i \n", i,xs,ys,row,col);		
			row = floor(xs/csize);
			col = floor(ys/csize);
			i++;
		} while ((i*dr<beam.range) && (i*dr<kMaxLaserDistance_) && (row!=hrow || col!=hcol));
		//printf("Beam # = %i   Hit point row/col = %i , %i \n", b, hrow, hcol);
	}

}

bool Mapping::isHitPoint(const int row, const int col, const std::vector<Point<int>> Phits)
{
	int b=0;
	Point<int> p;
	for (auto it : Phits)
	{
		b++;
		p = it;
		if(row == p.x && col == p.y)
		{			
			//printf("Point row,col = %i %i was a hit on beam %i and we will NOT MARK FREE \n", p.x,p.y,b);
			return true;
		}
	}
	//printf("Point row,col = %i %i was NOT a hit and we will are allowed to update as FREE \n", p.x,p.y);
	return false;
}
