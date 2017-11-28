/* Finderbots: Implementation of A-star algorithm */
#include <finderbot/planner.h>


struct Coordinates
{
	int x;
	int y;
};

Planner* path_finding_planner;

Planner::Planner(nav_msgs::OccupancyGrid global_map) : global_map_(global_map)
{
	nodes_.resize(global_map_.size(),0);
}
// INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
//          an x,y destination
// OUTPUT:  command velocities... angular and linear velocities
//      geometry_msgs/Twist.h
//      at any given time you can only rotate or go forward or backward
Node * Planner::aStar(int goal_row, int goal_col,
					  int source_row, int source_col) {
	if (!pointInMap(source_row, source_col)) {
        ROS_INFO("Error: source out of map");
		return NULL;
	}
	if (!pointInMap(goal_row, goal_col)) {
        ROS_INFO("Error: goal out of map");
		return NULL;
	}
	std::priority_queue<Node*, std::vector<Node*>, Compare> visit_queue; // set of pointers to nodes sorted by the f_scores

    for (int i = 0; i < global_map_.size(); ++i)
    {
        nodes_[i].row = i / global_width;
        nodes_[i].col = i % global_width;
        nodes_[i].parent = NULL;
        nodes_[i].visited = false;
        nodes_[i].g_score = INT_MAX;
        // h_score should stay the same as long as the goal does not change as we are moving towards goal
    	// 50 is an arbitrary probability of an obstacle
    	if (global_map_[i] > 50.0) {
    		nodes_[i].h_score = INT_MAX;
    	}
    	else {
        	nodes_[i].h_score = distance(nodes_[i].row, nodes_[i].col, goal_row, goal_col); // heuristic is just euclidean distance
    	}
        nodes_[i].f_score = INT_MAX;
    }

    size_t source_idx = getOffsetRowCol(source_row, source_col);
    nodes_[source_idx].parent = NULL;
    nodes_[source_idx].visited = true;
    nodes_[source_idx].g_score = 0;
    nodes_[source_idx].h_score = distance(source_row, source_col, goal_row, goal_col);
    nodes_[source_idx].f_score = nodes_[source_idx].g_score + nodes_[source_idx].h_score;

    visit_queue.push(&nodes_[source_idx]);

    std::vector<Node*> neighbors;
    Node * current_node = NULL;
    while (!visit_queue.empty() && !isGoal(current_node, goal_row, goal_col)) {
        current_node = visit_queue.top();
        current_node->visited = true;

        visit_queue.pop();
        getNeighbors(*current_node, neighbors);
        for (int i = 0; i < neighbors.size(); ++i) {
        	// If this is a neighbor that has not already in the visit_queue, add it to the visit_queue
        	visit_queue.push(neighbors[i]);
            // if there exists a neighbor to the n, e, s, w, and ne, se, sw, nw.
            int tentative_g_score = current_node->g_score + distance(*current_node, *neighbors[i]);
            if (neighbors[i]->g_score > tentative_g_score) {
				neighbors[i]->g_score = tentative_g_score;
				neighbors[i]->parent = current_node;
				if (INT_MAX == neighbors[i]->h_score) {
					neighbors[i]->f_score = INT_MAX;
				}
				else {
					neighbors[i]->f_score = neighbors[i]->g_score + neighbors[i]->h_score;
				}
            }
        }
    }
    // at this point, current_node is the goal
    return current_node;
}

void Planner::getNeighbors(const Node & node, std::vector<Node*> &neighbors) {
	// clear the neighbors vector
	neighbors.clear();
	// Add neighbord to vector for evaluation if they have not been visited
    // check north neighbor
    if (pointInMap(node.row + 1, node.col) && nodes_[getOffsetRowCol(node.row + 1, node.col)].visited == false) {
        neighbors.push_back(&nodes_[getOffsetRowCol(node.row + 1, node.col)]);
    }
    // check east neighbor
    if (pointInMap(node.row, node.col + 1) && nodes_[getOffsetRowCol(node.row, node.col + 1)].visited == false) {
        neighbors.push_back(&nodes_[getOffsetRowCol(node.row, node.col + 1)]);
    }
    // check south neighbor
    if (pointInMap(node.row - 1, node.col) && nodes_[getOffsetRowCol(node.row - 1, node.col)].visited == false) {
        neighbors.push_back(&nodes_[getOffsetRowCol(node.row - 1, node.col)]);
    }
    // check west neighbor
    if (pointInMap(node.row, node.col - 1) && nodes_[getOffsetRowCol(node.row, node.col - 1)].visited == false) {
        neighbors.push_back(&nodes_[getOffsetRowCol(node.row, node.col - 1)]);
    }
    // check northeast neighbor
    if (pointInMap(node.row + 1, node.col + 1) && nodes_[getOffsetRowCol(node.row + 1, node.col + 1)].visited == false) {
        neighbors.push_back(&nodes_[getOffsetRowCol(node.row + 1, node.col + 1)]);
    }
    // check southeast neighbor
    if (pointInMap(node.row - 1, node.col + 1) && nodes_[getOffsetRowCol(node.row - 1, node.col + 1)].visited == false) {
        neighbors.push_back(&nodes_[getOffsetRowCol(node.row - 1, node.col + 1)]);
    }
    // check southwest neighbor
    if (pointInMap(node.row - 1, node.col - 1) && nodes_[getOffsetRowCol(node.row - 1, node.col - 1)].visited == false) {
        neighbors.push_back(&nodes_[getOffsetRowCol(node.row - 1, node.col - 1)]);
    }
    // check northwest neighbor
    if (pointInMap(node.row + 1, node.col - 1) && nodes_[getOffsetRowCol(node.row + 1, node.col - 1)].visited == false) {
        neighbors.push_back(&nodes_[getOffsetRowCol(node.row + 1, node.col - 1)]);
    }
    return;
}

// Populate the path_coordinates_ with the path as an array of tuples (coordinates)
void Planner::getPath(Node * goal) {
	// Start pushing from the goal to the source, so will be reverse path
	Node * current_node;
	ROS_INFO("Coordinates:");
	int num_points_in_path = 0;
	while (NULL != current_node) {
		path_coordinates_.push_back(current_node->row);
		path_coordinates_.push_back(current_node->col);
		ROS_INFO("(%d, %d)", (int)path_coordinates_[num_points_in_path*2], (int)path_coordinates_[num_points_in_path*2+1]);

		current_node = current_node->parent;
		++num_points_in_path;
	}
	// path_coordinates member is a 1D array/stack with the path coordinates
	// pop from top to bottom to go start to finish
	return;
}

bool Planner::isGoal(Node * node, int goal_row, int goal_col) {
	return (node->row == goal_row) && (node->col == goal_col);
}

bool Planner::pathCb(finderbot::getPath::Request  &req,
		  			 finderbot::getPath::Response &res) {
	// Hard code source and destination for tests
	// TEST 1: 	s -> g:	(0,0) -> (0,0)
	// 			Path: 	[[ 0 , 0 ]]
	// TEST 2: 	s -> g:	(0,0) -> (1,1)
	// 			Path: 	[[ 0 , 0 ], [ 1 , 1 ]]
	req.source_x = 0;
	req.source_y = 0;
	req.goal_x = 0;
	req.goal_y = 0;

	Node * goal = aStar(req.goal_x, req.goal_y, req.source_x, req.source_y);
	if (goal == NULL) {
		ROS_INFO("Error: invalid pathfinding attempt");
		return 1;
	}
	getPath(goal);

	// path_coordinates_ member is a 1D array with the path coordinates
	res.path = path_coordinates_;
	for (int i = 0; i < path_coordinates_.size()/2; ++i)
		ROS_INFO("(%d, %d)", (int)res.path[2*i], (int)res.path[2*i + 1]);
	return true;
}

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "planner");
//     ros::NodeHandle nh;

//     /* THIS IS WHERE WE SUBSCRIBE TO THE GLOBAL MAP, MAY NEED TO CHANGE */
//     // ros::Subscriber globalMapHandler = nh.subscribe<nav_msgs::OccupancyGrid>("global_map", 1, true);
//     // nav_msgs::OccupancyGrid global_map = ???;
//     nav_msgs::OccupancyGrid global_map = global_map_builder->global_map_;
//     /* ALSO, HOW TO GET nav_msgs::OccupancyGrid global_map FROM HERE */
//     // Initialize nodes array to the size of the map

// 	Planner* planner = new Planner(global_map);

// 	ros::ServiceServer service = nh.advertiseService("get_path", planner->pathCb);
// 	ROS_INFO("Ready to print path coordinates.");
//     ros::spin();

//     return 0;
// }