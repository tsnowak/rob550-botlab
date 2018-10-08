#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <planning/astar_types.h>
#include <common/angle_functions.hpp>
#include <queue>
#include <math.h>
#include <vector>
#include <list>

const float kNonNavigableA = 3000; 
const uint maxIters = 6000;
const uint spline = 1;

// starting at 0 radians, the various movement options
static const int dirmap[8][2] = {
	{1,0}, {1,1}, {0,1}, {-1,1}, 
	{-1,0}, {-1,-1}, {-1,-1}, {1,-1}
};

// declare node structure
// * cost - 	weight associated with moving to the node
// * self -	x,y location of node, use theta for pointing to next node 
// * parent -	node from which the current node came from (used to back track the search path)
typedef struct node
{
	float cost;				// stores total cost of getting to this node
	pose_xyt_t self;		// stores grid frame coordinates
	node* parent;			// stores pointer to the node which came before this one
	pose_xyt_t pose;		// stores actual pose information at this node

	node() {}
	node(float xint, float yint, float costint)
	{
		self.x = xint;
		self.y = yint;
		pose.x = xint;
		pose.y = yint;
		cost = costint;
	}
	node(float xint, float yint, float thetaint, float costint)
	{
		self.x = xint;
		self.y = yint;
		self.theta = thetaint;	
		pose.x = xint;
		pose.y = yint;
		pose.theta = thetaint;
		cost = costint;
	}

} node_t;

// operator for the priority queue, used to place the lowest cost node
// at the top of the list
struct prioritizeShortest {
	bool operator()(const node & a, const node & b) const 
	{
		return a.cost > b.cost;
	}
};

// applies the Manhattan distance heuristic and outputs the added weight
double heuristic(pose_xyt_t goal, pose_xyt_t current)
{
	return sqrt(pow(abs(goal.x - current.x),2) + pow(abs(goal.y - current.y),2)); 
}

// goes back through the vector of visited nodes and goes backwards through the shortest path
// to extract the path of poses. Additionally sets the consecutive theta values at each pose
// to point to the next pose
void recoverPath(const std::list<node>& nodes, std::vector<node>& tmp_nodes, const node& start)
{
	//printf("Back-tracking path...\n"); //GC removed

	// testing
	/*
	for (uint i = 1; i < nodes.size(); i++)
	{
		printf("Node[%i]: x: %f\ty: %f\n", i, nodes[i].self.x, nodes[i].self.y);	
		printf("Parent of Node[%i]: x: %f\ty: %f\n", i, nodes[i].parent->self.x, nodes[i].parent->self.y);
	}
	*/

	//printf("\n");

	tmp_nodes.insert(tmp_nodes.begin(), nodes.back());
    //printf("Front Node: x: %f\t y: %f\n", tmp_nodes.front().self.x, tmp_nodes.front().self.y);   //GC::added
	//printf("Front node: x: %f\ty: %f\n", tmp_nodes[0].self.x, tmp_nodes[0].self.y);

	// starting from the goal node, backtrack to the start node
	// and insert each sequential node at the from of a vector
	//printf("Start node: x: %f\ty: %f\n", start.self.x, start.self.y);	
	while ((tmp_nodes.front().self.x != start.self.x) || (tmp_nodes.front().self.y != start.self.y))
	{		
		node tmp = *(tmp_nodes.front().parent);
		tmp_nodes.insert(tmp_nodes.begin(), tmp);
		//printf("Parent Node: x: %f\t y: %f\n", tmp_nodes.front().self.x, tmp_nodes.front().self.y);

	}

	//printf("Node vector path: length: %lu\n", tmp_nodes.size());
	//printf("\n");
	// for all of the nodes in the vector, assign a theta value pointing to the next node
    /*
	for (uint i = 0; i < (tmp_nodes.size()-1); i++)
	{
		float diffx = tmp_nodes[i+1].pose.x - tmp_nodes[i].pose.x;
		float diffy = tmp_nodes[i+1].pose.y - tmp_nodes[i].pose.y; 
		tmp_nodes[i].pose.theta = wrap_to_2pi(atan2(diffy, diffx));
		//printf("tmp_nodes Path[%i]: x: %f\ty: %f\ttheta: %f\n", i, tmp_nodes[i].pose.x, tmp_nodes[i].pose.y, tmp_nodes[i].pose.theta);  //GC : removed
	}
    */
	//uint idx_tmp = tmp_nodes.size()-1;
	//printf("tmp_nodes Path[%i]: x: %f\ty: %f\ttheta: %f\n", idx_tmp, tmp_nodes[idx_tmp].pose.x, tmp_nodes[idx_tmp].pose.y, tmp_nodes[idx_tmp].pose.theta);
}

// appends the recovered node path to the robot_path_t of poses
void appendPath(const std::vector<node>& nodes, robot_path_t& path, const pose_xyt_t& start, const pose_xyt_t& goal)
{
    robot_path_t tmp_path;
	printf("\n");
	//printf("Appending path of length: %lu\n", (nodes).size());

	path.path.clear();
	path.path.shrink_to_fit();

    uint tmp_spline;

	//path.path.push_back(start);
	//for (uint i = 1; i < (nodes.size()); i+=3) 
	//{
	//	path.path.push_back(nodes[i].pose);	
	//}			
	//path.path.push_back(goal);

    if (nodes.size() <= spline)
    {
       tmp_spline = 1; 
    }
    else
        tmp_spline = spline;

    //tmp_path.path.push_back(start);
	for (uint i = tmp_spline; i < (nodes.size()); i+= tmp_spline) 
	{
		tmp_path.path.push_back(nodes[i].pose);	
	}			
	tmp_path.path.push_back(goal);

    printf("Path size: %ld\n", tmp_path.path.size());

    for (uint i = 0; i < (tmp_path.path.size()-1); i++)
	{
		float diffx = tmp_path.path[i+1].x - tmp_path.path[i].x;
		float diffy = tmp_path.path[i+1].y - tmp_path.path[i].y; 
		tmp_path.path[i].theta = wrap_to_2pi(atan2(diffy, diffx));
		//printf("Path[%u]: x: %f\ty: %f\ttheta: %f\n", i, tmp_path.path[i].x, tmp_path.path[i].y, tmp_path.path[i].theta);  //GC : removed
	}

    path.path = tmp_path.path;
    printf("Size of path %lu\n", path.path.size());

    /*
	for (uint i = 0; i < path.path.size(); i++) 
	{
		printf("Output path: x: %f\ty: %f\n", path.path[i].x, path.path[i].y);  // GC removed
	}
    */
	printf("\n");
}

// overload == operator for std::find in vector of poses
/*
bool operator==(const pose_xyt_t& current, const pose_xyt_t& comparison)
{
	return (current.x == comparison.x) && (current.y == comparison.y);	
}
*/

// overload == operator for std::find in vector of nodes
bool operator==(const node& current, const node& comparison)
{
	return (current.pose.x == comparison.pose.x) && (current.pose.y == comparison.pose.y);
}

// sees if a given pose is in a vector
int findPose(const pose_xyt_t& current, const std::vector<pose_xyt_t>& vector)
{
	int val = -1;

	for (uint i = 0; i < vector.size(); i++)
	{
		if ((vector[i].x == current.x) && (vector[i].y == current.y))	
			val = i;
	}

	return val;
}

int findNode(const node& current, const std::vector<node>& vector)
{
	int val = -1;

	for (uint i = 0; i < vector.size(); i++)
	{
		if ((vector[i].pose.x == current.pose.x) && (vector[i].pose.y == current.pose.y))	
			val = i;
	}

	return val;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{

    // MAKE SURE TO SHIFT START AND GOAL INTO DISTANCEGRID COORDINATES
    // Top Left (0,0) -> (10,10)?

    std::priority_queue<node, std::vector<node>, prioritizeShortest> openpq;
    std::vector<node> node_path;
    std::list<node> nodes;
	// start at centered 0,0; add 5 meters,
	// divide it into 5 cm grids [ROUNDED DOWN]
    node current;
    node test;

    uint iter = 0;
    float xg = 5;
    float yg = 5;
    float csize = .05;

    node fake;
    fake.self.x = -1;
    fake.self.y = -1;
    fake.parent = NULL;
    nodes.push_back(fake);

    node start_n;
    start_n.self.x = (int)floor((start.x + xg)/csize);
    start_n.self.y = (int)floor((start.y + yg)/csize);
    printf("Astar :: Start.self: x: %0.0f\ty: %0.0f", start_n.self.x, start_n.self.y);
    start_n.pose = start; 
    //printf("Start.pose: x: %f\ty: %f\n", start_n.pose.x, start_n.pose.y);
    start_n.parent = &nodes.front();
    start_n.cost = 0;

    node goal_n;
    goal_n.self.x = (int)floor((goal.x + xg)/csize);
    goal_n.self.y = (int)floor((goal.y + yg)/csize);
    printf("    End.self: x: %0.0f\ty: %0.0f", goal_n.self.x, goal_n.self.y);
    goal_n.pose = goal;
    //printf("End.pose: x: %f\ty: %f\n", goal_n.pose.x, goal_n.pose.y);
    printf("    Cost end: %0.4f\n", distances(goal_n.self.x, goal_n.self.y));

    std::vector<pose_xyt_t> closed;
    std::vector<pose_xyt_t> open;		// note whenever we add to openpq, we need to add to open

   	robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    

    // assign the current to the start and
    // add it to the priority queue
    current = start_n;
    openpq.push(current);
    open.push_back(current.self);

    while (!openpq.empty())
    {
    	// handle the top node of the priority queue
    	current = openpq.top();
    	openpq.pop();

    	// add our current grid pose location to the closed map
    	closed.push_back(current.self);	
		nodes.push_back(current);		// push the current visited node to the end of nodes vector
    	node *parent_address = &nodes.back();        

		//printf("\n");
		//printf("Size of Nodes: %lu\n", nodes.size());
		//printf("Current Node: x: %f\ty: %f\tcost: %f\n", current.self.x, current.self.y,
        //        distances(current.self.x, current.self.y));
		//printf("Current Node Parent: x: %f\ty: %f\n", current.parent->self.x, current.parent->self.y);
		//printf("\n");
        if (iter > maxIters)
        {
            printf("NO PATH FOUND! AHHHHHH!!!\n\n");
            break;
        }

    	// make sure to compare grid goal node and grid current node
    	if ((current.self.x == goal_n.self.x) && (current.self.y == goal_n.self.y))
    	{
    		printf("Path found!!!!!!");
    		recoverPath(nodes, node_path, start_n);
    		appendPath(node_path, path, start, goal);
    		break;
    	}

        // count how many loops we've run to handle giving up
        iter++;
        //printf("Iter: %u\n", iter);

    	for (int i = 0; i < 8; i++)
    	{
    		// apply dirmap to get next possible position
    		test.self.x = current.self.x + dirmap[i][0];
    		test.self.y = current.self.y + dirmap[i][1]; 
    		//printf("Test: x: %f\ty: %f\n", test.self.x, test.self.y);

			// if it's not in the closed set, the open set, nor is the weight > 198, then add it to the pq
			// open and closed vectors<pose> to handle improperly filled nodes comparison

    		//printf("findPose Closed: %i\t findPose Open: %i\t distances: %i\n", (findPose(test.self, closed)<=0), 
    		//		(findPose(test.self, open) <= 0), (distances(test.self.x,test.self.y) < 198));

    		if ((findPose(test.self, closed) <= 0) && (findPose(test.self, open) <= 0)
    				&& (distances(test.self.x,test.self.y) < kNonNavigableA))	// DOUBLE CHECK DISTANCES OPERATOR
    		{
    			// make node and fill it
    			node tmp;
    			tmp.self.x = test.self.x;
    			tmp.self.y = test.self.y;
    			tmp.cost = current.cost + distances(test.self.x, test.self.y) + heuristic(goal, tmp.self);			
    			tmp.parent = parent_address;		// needed to make nodes for this
    			tmp.pose.x = (tmp.self.x*.05)-5;		
    			tmp.pose.y = (tmp.self.y*.05)-5;

    			//printf("Added node to priority queue.\n");
    			//printf("Node: x: %f\ty: %f\n", tmp.self.x, tmp.self.y);
    			//printf("Parent: x: %f\ty: %f\n", tmp.parent->self.x, tmp.parent->self.y);
    			openpq.push(tmp);
    			open.push_back(tmp.self);
    		}
    	}
    }
     
    path.path_length = path.path.size(); 
    if (path.path_length == 1)
    {
            printf("NO PATH FOUND because unreachable!!\n\n");
    }
    //printf("SENDING PATH OF SIZE %i\n", path.path_length);     //GC removed
    return path;
}
