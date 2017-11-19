#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <limits.h>
#include <math.h>

using namespace std;

struct Node
{
    int row;
    int col;
    int dist;
    struct Node * parent;
    bool visited;
    int g_score = 0;
    int h_score = INT_MAX;
    int f_score;
};

/* Return true if the point lies in the map
 */
inline bool pointInMap(const int row, const int col, const nav_msgs::OccupancyGrid & global_map)
{
    return ((0 <= col) && ((size_t) col < global_map.info.width) && (0 <= row) && ((size_t) row < global_map.info.height));
}

inline size_t getOffsetRowCol(size_t row, size_t col, const nav_msgs::OccupancyGrid & global_map)
{
    return (row * global_map.info.width) + col;
}

int main() {
    return 0;
}

// INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
//          an x,y destination
// OUTPUT:  command velocities... angular and linear velocities
//      geometry_msgs/Twist.h
//      at any given time you can only rotate or go forward or backward
void a_star(const nav_msgs::OccupancyGrid global_map, int goal_row, int goal_col) {
    // where N is the number of nodes/size of global_map
    // vector<Node*> visit_queue;
    vector<Node*> nodes;
    int N = global_map.data.size();
    for (int i = 0; i < N; ++i)
    {
        nodes[i].row = i / global_map.info.width;
        nodes[i].col = i % global_map.info.width;
        nodes[i].dist = INT_MAX;
        nodes[i].parent = NULL;
        nodes[i].visited = false;
    }
    nodes[0].dist = INT_MAX;
    nodes[0].parent = NULL;
    nodes[0].visited = false;

    // visit_queue.append(nodes[0]);

    int j = 0;
    vector<Node*> neighbors;
    while (/*(visit_queue.isEmpty() == false) && */(current_node != goal)) {
        current_node = nodes.pop_back(); // calculate f_score()
        current_node.visited = true;
        for () {
            // if there exists a neighbor to the top, bottom, left, right
            neighbor.g_score = current_node.g_score + 1;
            // visit_queue.push(neighbor);
            if (neighbor.dist > current_node.dist + distance(neighbor, current_node)) {
                neighbor.parent = current_node;
                neighbor.dist = current_node.dist + distance(neighbor, current_node);
                int f_score = neighbor.g_score + neighbor.h_score;
            }
        }
    }
    // output parent, distance? OR PBR the nodes and update their elements
}

int get_neighbor(Node node, vector<Node*> &neighbors) {
    // check north neighbor
    if (pointInMap(node.row + 1, node.col)) {
        neighbors.push_back();
    }
    // check east neighbor
    if (getOffsetRowCol(node.row + 1, node.col) < global_map.data.size()) {
        neighbors.push_back();
    }
    // check south neighbor
    // check west neighbor
}

int distance(Node node1, Node node2) {
    // Euclidean distance
    return sqrt(pow(node1.row-node2.row,2) + pow(node1.col-node2.col,2));
}

int h_score(Node node, Node goal) {
    //  Euclidean distance
    return distance(node, goal);
}