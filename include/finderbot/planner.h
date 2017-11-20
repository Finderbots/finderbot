#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <limits.h>
#include <math.h>

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

Class Planner {
    nav_msgs::OccupancyGrid global_map;

    // INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
    //          an x,y destination
    // OUTPUT:  command velocities... angular and linear velocities
    //      geometry_msgs/Twist.h
    //      at any given time you can only rotate or go forward or backward
    void a_star(const nav_msgs::OccupancyGrid & global_map, int goal_row, int goal_col);

    int get_neighbor(Node & node, vector<Node*> & neighbors);

    int distance(Node & node1, Node & node2);

    int h_score(Node & node, Node & goal);
}

/* Return true if the point lies in the map */
inline bool pointInMap(const int row, const int col, const nav_msgs::OccupancyGrid & global_map)
{
    return ((0 <= col) && ((size_t) col < global_map.info.width) && (0 <= row) && ((size_t) row < global_map.info.height));
}

inline size_t getOffsetRowCol(size_t row, size_t col, const nav_msgs::OccupancyGrid & global_map)
{
    return (row * global_map.info.width) + col;
}
