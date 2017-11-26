/* Finderbots: Implementation of A-star algorithm */
#include <finderbot/planner.h>


struct Coordinates
{
	int x;
	int y;
};

struct Node
{
    int row;
    int col;
    struct Node * parent;
    bool visited;
    int g_score;
    int h_score;
    int f_score;
};


Planner::Planner(nav_msgs::OccupancyGrid _global_map) {
	global_map = _global_map;
}
// INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
//          an x,y destination
// OUTPUT:  command velocities... angular and linear velocities
//      geometry_msgs/Twist.h
//      at any given time you can only rotate or go forward or backward
Node * Planner::aStar(int goal_row, int goal_col,
					  int source_row, int source_col) {
	if (!pointInMap(source_row, source_col, global_map)) {
        ROS_INFO("Error: source out of map");
		return;
	}
	if (!pointInMap(goal_row, goal_col, global_map)) {
        ROS_INFO("Error: goal out of map");
		return;
	}
    int N = global_map.data.size();
    std::vector<Node> nodes(N);
	std::priority_queue<Node*, std::vector<Node*>, Compare> visit_queue; // set of pointers to nodes sorted by the f_scores

    for (int i = 0; i < N; ++i)
    {
        nodes[i].row = i / global_map.info.width;
        nodes[i].col = i % global_map.info.width;
        nodes[i].parent = NULL;
        nodes[i].visited = false;
        nodes[i].g_score = INT_MAX;
        // h_score should stay the same as long as the goal does not change as we are moving towards goal
        nodes[i].h_score = distance(nodes[i].row, nodes[i].col, goal_row, goal_col); // heuristic is just euclidean distance
        nodes[i].f_score = INT_MAX;
    }

    size_t source_idx = getOffsetRowCol(source_row, source_col, global_map);
    nodes[source_idx].parent = NULL;
    nodes[source_idx].visited = true;
    nodes[source_idx].g_score = 0;
    nodes[source_idx].h_score = distance(source_row, source_col, goal_row, goal_col);
    nodes[source_idx].f_score = nodes[source_idx].g_score + nodes[source_idx].h_score;

    visit_queue.push(nodes[source_idx]);

    std::vector<Node*> neighbors;
    Node * current_node = NULL;
    while (!visit_queue.isEmpty() && !isGoal(current_node, goal_row, goal_col)) {
        current_node = visit_queue.top();
        current_node.visited = true;

        visit_queue.pop();
        getNeighbors(current_node, neighbors, global_map);
        for (int i = 0; i < neighbors.size; ++i) {
        	// If this is a neighbor that has not already in the visit_queue, add it to the visit_queue
        	visit_queue.push(neighbors[i]);
            // if there exists a neighbor to the n, e, s, w, and ne, se, sw, nw.
            int tentative_g_score = current_node.g_score + distance(current_node, *neighbors[i]);
            if (neighbors[i]->g_score > tentative_g_score) {
				neighbors[i]->g_score = tentative_g_score;
				neighbors[i]->parent = current_node;
				neighbors[i]->f_score = neighbors[i].g_score + neighbors[i].h_score;
            }
        }
    }
    // at this point, current_node is the goal
    return current_node;
}

int Planner::getNeighbors(const Node & node, std::vector<Node*> &neighbors) {
	// clear the neighbors vector
	neighbors.clear();
	// Add neighbord to vector for evaluation if they have not been visited
    // check north neighbor
    if (pointInMap(node.row + 1, node.col) && nodes[getOffsetRowCol(node.row + 1, node.col, global_map)].visited == false) {
        neighbors.push_back(&nodes[getOffsetRowCol(node.row + 1, node.col, global_map)]);
    }
    // check east neighbor
    if (pointInMap(node.row, node.col + 1) && nodes[getOffsetRowCol(node.row, node.col + 1, global_map)].visited == false) {
        neighbors.push_back(&nodes[getOffsetRowCol(node.row, node.col + 1, global_map)]);
    }
    // check south neighbor
    if (pointInMap(node.row - 1, node.col) && nodes[getOffsetRowCol(node.row - 1, node.col, global_map)].visited == false) {
        neighbors.push_back(&nodes[getOffsetRowCol(node.row - 1, node.col, global_map)]);
    }
    // check west neighbor
    if (pointInMap(node.row, node.col - 1) && nodes[getOffsetRowCol(node.row, node.col - 1, global_map)].visited == false) {
        neighbors.push_back(&nodes[getOffsetRowCol(node.row, node.col - 1, global_map)]);
    }
    // check northeast neighbor
    if (pointInMap(node.row + 1, node.col + 1) && nodes[getOffsetRowCol(node.row + 1, node.col + 1, global_map)].visited == false) {
        neighbors.push_back(&nodes[getOffsetRowCol(node.row + 1, node.col + 1, global_map)]);
    }
    // check southeast neighbor
    if (pointInMap(node.row - 1, node.col + 1) && nodes[getOffsetRowCol(node.row - 1, node.col + 1, global_map)].visited == false) {
        neighbors.push_back(&nodes[getOffsetRowCol(node.row - 1, node.col + 1, global_map)]);
    }
    // check southwest neighbor
    if (pointInMap(node.row - 1, node.col - 1) && nodes[getOffsetRowCol(node.row - 1, node.col - 1, global_map)].visited == false) {
        neighbors.push_back(&nodes[getOffsetRowCol(node.row - 1, node.col - 1, global_map)]);
    }
    // check northwest neighbor
    if (pointInMap(node.row + 1, node.col - 1) && nodes[getOffsetRowCol(node.row + 1, node.col - 1, global_map)].visited == false) {
        neighbors.push_back(&nodes[getOffsetRowCol(node.row + 1, node.col - 1, global_map)]);
    }

}

// Publish the path as an array of tuples (coordinates)
void Planner::getPath(Node * goal) {
	// Start pushing from the goal to the source, so will be reverse path
	std::vector<int> tuple(2);
	Node * current_node;
	while (NULL != current_node) {
		tuple[0] = current_node->row;
		tuple[1] = current_node->col;
		path_coordinates.append(tuple);
		current_node = current_node->parent;
	}
	// path_coordinates member is a 2D STACK with the path coordinates
	// pop from top to bottom to go start to finish
	return;
}

bool Planner::isGoal(Node * node, int goal_row, int goal_col) {
	return (node->row == goal_row) && (node->col == goal_col);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    /* THIS IS WHERE WE SUBSCRIBE TO THE GLOBAL MAP, MAY NEED TO CHANGE */
    // ros::Subscriber globalMapHandler = nh.subscribe<nav_msgs::OccupancyGrid>("global_map", 1, true);
    /* ALSO, HOW TO GET nav_msgs::OccupancyGrid global_map FROM HERE */
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("matrix_pub", 1);
    // For now, loop once every ten seconds
    ros::Rate loop_rate(10000);
    std_msgs::Int32MultiArray dat;
	// path_coordinates member is a 2D array with the path coordinates
	std::vector< std::vector<int> > path_coordinates;
	Node * goal = aStar(goal_row, goal_col, source_row, source_col);
	getPath(goal, path_coordinates);

    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "points";
    dat.layout.dim[1].label = "coordinates";
    dat.layout.dim[0].size = path_coordinates.size();
    dat.layout.dim[1].size = 2;
    dat.layout.dim[0].stride = 2*path_coordinates.size();
    dat.layout.dim[1].stride = 2;
    dat.layout.data_offset = 0;
    std::vector<int> vec(2*path_coordinates.size(), 0);
    for (int i=0; i<path_coordinates.size(); i++) {
        for (int j=0; j<2; j++) {
            vec[i*2 + j] = path_coordinates[i][j];
        }
    }
    dat.data = vec;

    while (ros::ok())
    {
        pub.publish(dat);
        loop_rate.sleep();
    }


    ros::spin();

    return 0;
}