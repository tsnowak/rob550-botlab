#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>

const int kWallWeight = 5000;
const float kGradientFactor = 1.6;


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

int ObstacleDistanceGrid::saveToFile()
{
	FILE *pC;
    pC = fopen ("inflatedMap.txt" , "w");

    //printf("ObstacleDistanceGrid::saveToFile()\n");
    if (pC==NULL)
    {
        perror ("Error opening file");
        return 0;
    }
    for(int y=0; y<200; y++) // for all rows in the grid
    {
    	for(int x=0; x<200; x++)
    	{
			fprintf(pC,"%0.0f ",operator()(x,y));
		}
		fprintf(pC,"\n");
	}
    fclose(pC);
    return 1;
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    std::vector< Point<int> > wallsV, borderV, tempV;
    Point<int> gridP, tempP;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = { 0, 1, -1, 0, 1, -1, 1, -1 };
    const int kNumNeighbors = 8;
    int x,y,k;
    float n = kWallWeight;

    resetGrid(map);
    
    wallsV.clear();
    borderV.clear();
    //printf("wallsV.size(): %lu\n", wallsV.size());
    //printf("borderV.size(): %lu\n", borderV.size());

    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    // returns an inflated map if there is at least one cell marked as a wall (log>0), else the inflated map is not modified

    // for all cells in map, it finds all grid cells part of a wall (>=0), keeps them in a vector and marks them in the inflates map
    for(y = 0; y <200; y++) // originally 1 - 200
    {
        for(x = 0; x <200; x++)
        {
             if (map.logOdds(x, y)>0)  // >=0 
             {
                operator()(x,y) = map.logOdds(x, y) + n;	// applies a weight in the range of 200 to 327 to initial walls
                gridP.x = x;
                gridP.y = y;
                wallsV.push_back(gridP);
             }
             else
             {
                operator()(x,y) = -1;                       // erase all other cells in the previous obstacle distance grid map
             }
        }
    }

    // for all wall points, it finds all free (<0) surrounding cells, keeps them in a vector and marks them in the inflated map
    // applies a weight that lower as the cell is further away from the wall
    // this grows the walls progressively until no more free points exist
    // lowest cell value is never less than 0 since the map width and length in our case is max 200
    borderV = wallsV;
    int counterN = 0 ;
    while (borderV.size()!=0)
    {
        // TED changed 1 to 10
        counterN ++;
        n = floor(n/pow(kGradientFactor,counterN)) +1;
        tempV.clear();
        //printf("borderV.size(): %lu\n", borderV.size());
        for(auto it:borderV)
        {
            gridP = it;
            for(k = 0; k < (kNumNeighbors); k++)
            {
                x = gridP.x+xDeltas[k];
                y = gridP.y+yDeltas[k];
                //printf("Delta x y = %i %i \n", xDeltas[k], yDeltas[k]);
                //printf("neighbor k = %i kNumNeigh = %i n = %i coordinates x, y = %i %i Log odds %i \n", k, kNumNeighbors, n, x, y, map.logOdds(x,y));
                if (x>=0 && x<200 && y>=0 && y<200 && (map.logOdds(x,y)<=0) && (operator()(x,y)<=0))    //logOdss originlly <0 strict
                {
                    /*if(n>100)
                    {
                        operator()(x,y) = n;
                    }
                    else
                    {
                        operator()(x,y) = 100;
                    }*/
                    operator()(x,y) = n;
                    tempP.x = x;
                    tempP.y = y;
                    tempV.push_back(tempP);
                    //printf("pushed point into temp\n\n");
                }
            }
        }
        //printf("counter N = %i should equal tempV.size()  AFTR NEIGHBORS : %lu\n", counterN, tempV.size());
        borderV = tempV;
    }
    saveToFile();		// for debugging purposes only
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
