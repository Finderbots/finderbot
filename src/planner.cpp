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
    int dist;
    struct Node * parent;
    bool visited;
    int g_score;
    int h_score;
    int f_score;
};


// INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
//          an x,y destination
// OUTPUT:  command velocities... angular and linear velocities
//      geometry_msgs/Twist.h
//      at any given time you can only rotate or go forward or backward
void Planner::a_star(const nav_msgs::OccupancyGrid & global_map,
					 int goal_row, int goal_col,
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
        nodes[i].dist = INT_MAX;
        nodes[i].parent = NULL;
        nodes[i].visited = false;
        nodes[i].g_score = INT_MAX;
        nodes[i].h_score = distance(nodes[i].row, nodes[i].col, goal_row, goal_col); // heuristic is just euclidean distance
        nodes[i].f_score = INT_MAX;
    }

    size_t source_idx = getOffsetRowCol(source_row, source_col, global_map);
    nodes[source_idx].dist = 0;
    nodes[source_idx].parent = NULL;
    nodes[source_idx].visited = false;
    nodes[source_idx].g_score = 0;
    nodes[source_idx].h_score = distance(source_row, source_col, goal_row, goal_col);
    nodes[source_idx].f_score = nodes[source_idx].g_score + nodes[source_idx].h_score;

    visit_queue.push_back(nodes[source_idx]);

    std::vector<Node*> neighbors;
    while ((false == visit_queue.isEmpty()) && (current_node != goal)) {
        Node * current_node = nodes.top();
        current_node.visited = true;

        visit_queue.pop();
        this->get_neighbors(current_node, neighbors, global_map);
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
    // output parent, distance? OR PBR the nodes and update their elements
}

int Planner::get_neighbors(Node & node, std::vector<Node*> &neighbors, const nav_msgs::OccupancyGrid & global_map) {
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

/*
// Might need this function to know command velocities of which way to turn, how many degrees and how much to go forward
function reconstruct_path(cameFrom, current)
    total_path := [current]
    while current in cameFrom.Keys:
        current := cameFrom[current]
        total_path.append(current)
    return total_path
*/

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle nh;

	//...

    ros::Subscriber globalMapHandler = nh.subscribe<nav_msgs::OccupancyGrid>("global_map", 1, handleGlobalMap);

    ros::spin();
}