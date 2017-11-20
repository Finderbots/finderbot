/* Finderbots: Implementation of A-star algorithm */

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>
#include <cassert>

namespace global_mapping
{

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
	vector<Node*> visited_queue;
    vector<Node*> nodes;
    int N = global_map.data.size();
    for (int i = 0; i < N; ++i)
    {
        nodes[i].row = i / global_map.info.width;
        nodes[i].col = i % global_map.info.width;
        nodes[i].dist = INT_MAX;
        nodes[i].parent = NULL;
        nodes[i].visited = false;
        nodes[i].g_score = INT_MAX;
        nodes[i].h_score = INT_MAX;
        nodes[i].f_score = INT_MAX;
    }

    size_t source_idx = getOffsetRowCol(source_row, source_col, global_map);
    nodes[source_idx].dist = 0;
    nodes[source_idx].parent = NULL;
    nodes[source_idx].visited = false;
    nodes[i].g_score = 0;
    nodes[i].h_score = 0;
    nodes[i].f_score = 0;

    visited_queue.push_back(nodes[source_idx]);

    int j = 0;
    vector<Node*> neighbors;
    while ((false == visited_queue.isEmpty()) && (current_node != goal)) {
        current_node = nodes.pop_back(); // calculate f_score()
        current_node.visited = true;
        this->get_neighbors(current_node, neighbors, global_map);
        for (int i = 0; i < neighbors.size; ++i) {
            // if there exists a neighbor to the top, bottom, left, right (needs to change if we do ne/se/sw/nw)
            neighbors[i].g_score = current_node.g_score + distance(current_node, *neighbors[i]);
            if (neighbors[i].dist > current_node.dist + distance(neighbor, current_node)) {
                neighbors[i].parent = current_node;
                neighbors[i].dist = current_node.dist + distance(neighbor, current_node);
                neighbors[i].f_score = neighbors[i].g_score + neighbors[i].h_score;
            }
        }
    }
    // output parent, distance? OR PBR the nodes and update their elements
}

int Planner::get_neighbors(Node & node, vector<Node*> &neighbors, const nav_msgs::OccupancyGrid & global_map) {
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
    // if (pointInMap(node.row + 1, node.col + 1)) {
    //     neighbors.push_back(&nodes[getOffsetRowCol(node.row + 1, node.col, global_map)]);
    // }
    // // check southeast neighbor
    // if (pointInMap(node.row - 1, node.col + 1)) {
    //     neighbors.push_back(&nodes[getOffsetRowCol(node.row + 1, node.col, global_map)]);
    // }
    // // check southwest neighbor
    // if (pointInMap(node.row - 1, node.col - 1)) {
    //     neighbors.push_back(&nodes[getOffsetRowCol(node.row + 1, node.col, global_map)]);
    // }
    // // check northwest neighbor
    // if (pointInMap(node.row + 1, node.col - 1)) {
    //     neighbors.push_back(&nodes[getOffsetRowCol(node.row + 1, node.col, global_map)]);
    // }

}

int Planner::distance(Node & node1, Node & node2) {
    // Euclidean distance
    return sqrt(pow(node1.row-node2.row,2) + pow(node1.col-node2.col,2));
}

}
