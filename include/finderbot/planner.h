/* Finderbots: Implementation of A-star algorithm */
#include <limits.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string>
#include <cassert>

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

class Planner {
    nav_msgs::OccupancyGrid global_map;

    // A 2-D vector with the coordinates of each point in the map to visit
    std::vector< std::vector<int> > & path_coordinates;
    // Output command velocities based on plan
    ros::Publisher command_velocities;

    // INPUT:   nav_messages_occupancy_grid as a 1-D vector (graph)
    //          an x,y destination
    // OUTPUT:  command velocities... angular and linear velocities
    //      geometry_msgs/Twist.h
    //      at any given time you can only rotate or go forward or backward
  public:
    void aStar(int goal_row, int goal_col, int source_row, int source_col);

    int getNeighbors(const Node & node, vector<Node*> & neighbors);

    void getPath(Node * goal);

    bool isGoal(Node * node, int goal_row, int goal_col);
};

class Compare {
  public:
    bool operator() (const Node * node1, const Node * node2) {
        return node1->f_score < node2->f_score;
    }
};

// Pass 2 nodes
inline int distance(Node & node1, Node & node2) {
    // Euclidean distance
    return sqrt(pow(node1.row-node2.row,2) + pow(node1.col-node2.col,2));
}

// Pass 2 sets of row and column
inline int distance(int row1, int col1, int row2, int col2) {
    // Euclidean distance
    return sqrt(pow(row1-row2,2) + pow(col1-col2,2));
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