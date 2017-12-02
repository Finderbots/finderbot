/* Finderbots: Implementation of A-star algorithm */
#include <finderbot/planner.h>
#include <iostream>

using namespace map_utils;

Planner* path_finding_planner;

Planner::Planner(std::vector<double> global_map) : global_map_(global_map) , obstacle_distance_map_((size_t)global_width, (size_t)global_width)
{
	nodes_.resize(global_map_.size());

    obstacle_distance_map_.setDistances(global_map_);
}
// INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
//          an x,y destination
// OUTPUT:  command velocities... angular and linear velocities
//      geometry_msgs/Twist.h
//      at any given time you can only rotate or go forward or backward
std::vector<size_t> * Planner::aStar(size_t goal_row, size_t goal_col) {
	
	std::priority_queue<Node*, std::vector<Node*>, Compare> visit_queue; // set of pointers to nodes sorted by the f_scores

    for (size_t i = 0; i < global_map_.size(); ++i)
    {
        nodes_[i].row = i / global_width;
        nodes_[i].col = i % global_width;
        nodes_[i].parent = NULL;
        nodes_[i].visited = false;
        nodes_[i].g_score = DBL_MAX;
        // h_score should stay the same as long as the goal does not change as we are moving towards goal
    	// 50 is an arbitrary probability of an obstacle
    	if (global_map_[i] > 50.0) {
    		nodes_[i].h_score = DBL_MAX;
            std::cerr << "OBSTACLE AT (" << nodes_[i].row << "," << nodes_[i].col << ")\n";
    	}
    	else {
        	nodes_[i].h_score = distance(nodes_[i].row, nodes_[i].col, goal_row, goal_col); // heuristic is just euclidean distance
    	}
        nodes_[i].f_score = DBL_MAX;
    }

    size_t source_idx = getOffsetRowCol(source_row, source_col, global_width);
    nodes_[source_idx].parent = NULL;
    nodes_[source_idx].visited = true;
    nodes_[source_idx].g_score = 0;
    nodes_[source_idx].h_score = distance(source_row, source_col, goal_row, goal_col);
    nodes_[source_idx].f_score = nodes_[source_idx].g_score + nodes_[source_idx].h_score;

    visit_queue.push(&nodes_[source_idx]);

    std::vector<Node*> neighbors;
    Node * current_node = &nodes_[source_idx];
    while (!visit_queue.empty() && !isGoal(current_node, goal_row, goal_col)) {
        current_node = visit_queue.top();
        current_node->visited = true;

        visit_queue.pop();
        getNeighbors(*current_node, neighbors);
        for (size_t i = 0; i < neighbors.size(); ++i) {
        	// If this is a neighbor that has not already in the visit_queue, add it to the visit_queue
            visit_queue.push(neighbors[i]);
            // std::cerr << "EVALUATING (" << neighbors[i]->row << "," << neighbors[i]->col << ")\n";
            // if there exists a neighbor to the n, e, s, w, and ne, se, sw, nw.
            size_t tentative_g_score = current_node->g_score + distance(*current_node, *neighbors[i]);
            if (neighbors[i]->g_score > tentative_g_score) {
				neighbors[i]->g_score = tentative_g_score;
				neighbors[i]->parent = current_node;
				if (DBL_MAX == neighbors[i]->h_score) {
					neighbors[i]->f_score = DBL_MAX;
				}
				else {
					neighbors[i]->f_score = neighbors[i]->g_score + neighbors[i]->h_score;
				}
            }
        }
    }
    if (!isGoal(current_node, goal_row, goal_col)) {
        ROS_INFO("ERROR: failed pathfinding attempt");
        return NULL;
    }

    // at this point, current_node is the goal
    return getPath(current_node);
}

bool Planner::isValidNeighbor(size_t idx) {
    size_t row = rowFromOffset(idx, global_width);
    size_t col = colFromOffset(idx, global_width);
    if (row > 11 || col > 11) return false;
    return pointInMap(row, col, global_width, global_width)
                      && (nodes_[idx].visited == false)
                      && (global_map_[idx] <= 50.0);
}

void printNode(const Node & node) {
    std::cerr << "(" << node.row << "," << node.col << ")\n";
    return;
}

void Planner::getNeighbors(const Node & node, std::vector<Node*> &neighbors) {
	// clear the neighbors vector
	neighbors.clear();
	// Add neighbord to vector for evaluation if they have not been visited
    //   and if the occupancy is less than 50.0
    size_t n_idx = getOffsetRowCol(node.row + 1, node.col, global_width);
    size_t e_idx = getOffsetRowCol(node.row, node.col + 1, global_width);
    size_t s_idx = getOffsetRowCol(node.row - 1, node.col, global_width);
    size_t w_idx = getOffsetRowCol(node.row, node.col - 1, global_width);
    size_t ne_idx = getOffsetRowCol(node.row + 1, node.col + 1, global_width);
    size_t se_idx = getOffsetRowCol(node.row - 1, node.col + 1, global_width);
    size_t sw_idx = getOffsetRowCol(node.row - 1, node.col - 1, global_width);
    size_t nw_idx = getOffsetRowCol(node.row + 1, node.col - 1, global_width);

    std::cerr << "ADDING NEIGHBORS OF:";
    printNode(node);
    std::cerr << "\n";

    // check north neighbor
    if (isValidNeighbor(n_idx)) {
        neighbors.push_back(&nodes_[n_idx]);
        printNode(nodes_[n_idx]);
    }
    // check east neighbor
    if (isValidNeighbor(e_idx)) {
        neighbors.push_back(&nodes_[e_idx]);
        printNode(nodes_[e_idx]);
    }
    // check south neighbor
    if (isValidNeighbor(s_idx)) {
        neighbors.push_back(&nodes_[s_idx]);
        printNode(nodes_[s_idx]);
    }
    // check west neighbor
    if (isValidNeighbor(w_idx)) {
        neighbors.push_back(&nodes_[w_idx]);
        printNode(nodes_[w_idx]);
    }
    // check northeast neighbor
    if (isValidNeighbor(ne_idx)) {
        neighbors.push_back(&nodes_[ne_idx]);
        printNode(nodes_[ne_idx]);
    }
    // check southeast neighbor
    if (isValidNeighbor(se_idx)) {
        neighbors.push_back(&nodes_[se_idx]);
        printNode(nodes_[se_idx]);
    }
    // check southwest neighbor
    if (isValidNeighbor(sw_idx)) {
        neighbors.push_back(&nodes_[sw_idx]);
        printNode(nodes_[sw_idx]);
    }
    // check northwest neighbor
    if (isValidNeighbor(nw_idx)) {
        neighbors.push_back(&nodes_[nw_idx]);
        printNode(nodes_[nw_idx]);
    }
    return;
}

// Populate the path_coordinates_ with the path as an array of tuples (coordinates)
std::vector<size_t> * Planner::getPath(Node * goal) {
	// Start pushing from the goal to the source, so will be reverse path
	Node * current_node = goal;
	ROS_INFO("Coordinates:");
	size_t num_points_in_path = 0;
	while (NULL != current_node) {
		size_t offset = getOffsetRowCol(current_node->row, current_node->col, global_width);
		path_coordinates_.push_back(offset);
		ROS_INFO("(%d, %d)", (size_t)rowFromOffset(offset, global_width), (int)colFromOffset(offset, global_width));

		current_node = current_node->parent;
		++num_points_in_path;
	}
	// path_coordinates member is a 1D array/stack with the path coordinates
	// pop from top to bottom to go start to finish
	return &path_coordinates_;
}

bool Planner::isGoal(Node * node, size_t goal_row, size_t goal_col) {
	return (node->row == goal_row) && (node->col == goal_col);
}




void Planner::setPose(size_t row, size_t col)
{
    assert(pointInMap(row, col, global_width, global_width));
        

    source_row = row;
    source_col = col;
}
// bool Planner::pathCb(finderbot::getPath::Request  &req,
// 		  			 finderbot::getPath::Response &res) {
// 	// Hard code source and destination for tests
// 	// TEST 1: 	s -> g:	(0,0) -> (0,0)
// 	// 			Path: 	[[ 0 , 0 ]]
// 	// TEST 2: 	s -> g:	(0,0) -> (10,10)
// 	// 			Path: 	[[ 0 , 0 ], ..., [ 10 , 10 ]]
// 	req.source_x = 0;
// 	req.source_y = 0;
// 	req.goal_x = 0;
// 	req.goal_y = 0;

// 	Node * goal = aStar(req.goal_x, req.goal_y, req.source_x, req.source_y);
// 	if (goal == NULL) {
// 		ROS_INFO("Error: invalid pathfinding attempt");
// 		return 1;
// 	}
// 	getPath(goal);

// 	// path_coordinates_ member is a 1D array with the path coordinates
// 	res.path = path_coordinates_;
// 	for (int i = 0; i < path_coordinates_.size(); ++i)
// 		ROS_INFO("(%d, %d)", (int)rowFromOffset(i, global_width), (int)colFromOffset(i, global_width));
// 	return true;
// }
